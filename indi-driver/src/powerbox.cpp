#include "powerbox.h"
#include <defaultdevice.h>

using json = nlohmann::json;

static std::unique_ptr<Powerbox> shelyakDriver(new Powerbox());

Powerbox::Powerbox() {
  setDefaultPollingPeriod(1000);
  setVersion(1, 2);
}

Powerbox::~Powerbox() {
}

const char *Powerbox::getDefaultName() {
  return "Powerbox";
}

void Powerbox::cmdCrc(const char *cmd, char *out) {
  uint16_t crc = crcCalc(cmd);
  int len = strlen(cmd);
  memcpy(out+1, cmd, len);
  out[0] = '#';
  out[len+1] = '\0';
  out[len+2] = ((char *) &crc)[0];
  if ( out[len+2] == '$' ) {
    out[len+2] = '1';
  }
  out[len+3] = ((char *) &crc)[1];
  if ( out[len+3] == '$' ) {
    out[len+3] = '1';
  }
  out[len+4] = '$';
  out[len+5] = '\0';
}

bool Powerbox::checkCrc(const char *rsp) {
  uint16_t crcGotten;
  size_t len = strlen(rsp);
  ((char *) &crcGotten)[0] = rsp[len+1];
  ((char *) &crcGotten)[1] = rsp[len+2];
  uint16_t crcCalculated = crcCalc((const void *) rsp, len);
  char *crcChar = (char *) &crcCalculated;
  if ( crcChar[0] == '$' ) {
    crcChar[0] = '1';
  }
  if ( crcChar[1] == '$' ) {
    crcChar[1] = '1';
  }
  if ( crcGotten == crcCalculated ) {
    return true;
  } else {
    LOG_ERROR("Checksum error");
    LOGF_ERROR("Message: %s", rsp);
    LOGF_ERROR("Checksum: %d (0x%02x 0x%02x), expected %d (0x%02x 0x%02x)",
	crcCalculated, ((unsigned char *) &crcCalculated)[0], ((unsigned char *) &crcCalculated)[1],
	crcGotten, ((unsigned char *) &crcGotten)[0], ((unsigned char *) &crcGotten)[1]);
    return false;
  }
}

bool Powerbox::sendCommand(const char *cmd, char *rsp) {
  LOGF_DEBUG("Sending command: %s", cmd);
  int nbytes_written = 0, nbytes_read = 0, rc = -1;
  int PortFD = serialConnection->getPortFD();
  LOGF_DEBUG("PortFD: %d", PortFD);
  char buff[CMDBUFF];
  char err[ERRBUFF];
  char rspBuff[RSPBUFF];
  cmdCrc(cmd, buff);

  tcflush(PortFD, TCIOFLUSH);
  if ( (rc = tty_write(PortFD, buff, strlen(buff)+4, &nbytes_written)) != TTY_OK ) {
    tty_error_msg(rc, err, ERRBUFF);
    LOGF_ERROR("Error writing command %s: %s", cmd, err);
    return false;
  }

  LOG_DEBUG("RCV");

  // Somtimes there's garbage on the line so read until the next #
  rspBuff[0] = '\0';
  while ( rspBuff[0] != '#' ) {
    if ( (rc = tty_read(PortFD, rspBuff, 1, TIMEOUT, &nbytes_read) ) != TTY_OK ) {
      tty_error_msg(rc, err, ERRBUFF);
      LOGF_ERROR("Error reading response: %s", err);
      return false;
    }
  }

  if ( (rc = tty_read_section(PortFD, rspBuff+1, '$', TIMEOUT, &nbytes_read)) != TTY_OK ) {
    tty_error_msg(rc, err, ERRBUFF);
    LOGF_ERROR("Error reading response: %s", err);
    return false;
  }

  memcpy(rsp, rspBuff+1, strlen(rspBuff)+2);
  if ( ! checkCrc(rsp) ) {
    return false;
  }
  return true;
}


bool Powerbox::Handshake() {
  char cmd[] = "status";
  char rsp[RSPBUFF];
  if ( ! sendCommand(cmd, rsp) ) {
    return false;
  }
  json data;
  try {
    data = json::parse(rsp);
  } catch (...) {
    LOG_ERROR("JSON parse error");
    return false;
  }
  if ( data.contains("Error") ) {
    LOGF_ERROR("Device error: %s", data["Error"].template get<std::string>().c_str());
    return false;
  }
  return true;
}

bool Powerbox::initProperties() {
  INDI::DefaultDevice::initProperties();
  setDriverInterface(AUX_INTERFACE);
  addDebugControl();
  addConfigurationControl();
  setDefaultPollingPeriod(500);
  addPollPeriodControl();
  serialConnection = new Connection::Serial(this);
  serialConnection->setDefaultBaudRate(Connection::Serial::B_115200);
  serialConnection->registerHandshake([&]() {
      return Handshake();
      });
  registerConnection(serialConnection);

  IUFillNumber(&VoltageN[0], "VOLTAGE", "Voltage [V]", "%0.2f", 0.0, 99.0, 0.01, 0.0);
  IUFillNumberVector(&VoltageNP, VoltageN, 1, getDeviceName(), "VOLTAGE", "Power",
      MAIN_CONTROL_TAB, IP_RO, TIMEOUT, IPS_IDLE);

  IUFillNumber(&EnvN[TEMPERATURE], "TEMPERATURE", "Temperature [°C]", "%.1f", -100.0, 100.0, 0.1, 0.0);
  IUFillNumber(&EnvN[HUMIDITY], "HUMIDITY", "Rel. Humidity [%]", "%.1f", 0.0, 100.0, 0.1, 50.0);
  IUFillNumber(&EnvN[PRESSURE], "PRESSURE", "Pressure [mbar]", "%.1f", 0.0, 2000.0, 0.1, 1013.0);
  IUFillNumber(&EnvN[DEWPOINT], "DEWPOINT", "Dewpoint [°C]", "%.1f", -100.0, 100.0, 0.1, 0.0);
  IUFillNumberVector(&EnvNP, EnvN, 4, getDeviceName(), "ENVIRONMENT", "Environment",
      MAIN_CONTROL_TAB, IP_RO, TIMEOUT, IPS_IDLE);

  IUFillNumber(&EnvOffsetN[0], "ENVOFFSET", "Temperature offset [°C]", "%.1f", -10.0, 10.0, 0.1, 0.0);
  IUFillNumberVector(&EnvOffsetNP, EnvOffsetN, 1, getDeviceName(), "ENVOFFSET",
      "Environment", MAIN_CONTROL_TAB, IP_RW, TIMEOUT, IPS_IDLE);

  IUFillSwitch(&RailS[ON], "RAIL_ON", "On", ISS_OFF);
  IUFillSwitch(&RailS[OFF], "RAIL_OFF", "Off", ISS_ON);
  IUFillSwitchVector(&RailSP, RailS, 2, getDeviceName(), "RAIL", "12V Rail",
      MAIN_CONTROL_TAB, IP_RW, ISR_1OFMANY, TIMEOUT, IPS_IDLE);

  IUFillSwitch(&AdjS[ON], "RAIL_ON", "On", ISS_OFF);
  IUFillSwitch(&AdjS[OFF], "RAIL_OFF", "Off", ISS_ON);
  IUFillSwitchVector(&AdjSP, AdjS, 2, getDeviceName(), "ADJSTATE", "Adjustable state",
      MAIN_CONTROL_TAB, IP_RW, ISR_1OFMANY, TIMEOUT, IPS_IDLE);

  IUFillNumber(&AdjN[0], "ADJV", "V", "%.1f", 5.0, 12.0, 0.1, 0.0);
  IUFillNumberVector(&AdjNP, AdjN, 1, getDeviceName(), "ADJV", "Adjustable voltage",
      MAIN_CONTROL_TAB, IP_RW, TIMEOUT, IPS_IDLE);

  IUFillSwitch(&DH1ModeS[MODE_FIXED], "DH1_MODE_FIXED", "Fixed", ISS_OFF);
  IUFillSwitch(&DH1ModeS[MODE_DEWPOINT], "DH1_MODE_DEWPOINT", "Dewpoint", ISS_OFF);
  IUFillSwitch(&DH1ModeS[MODE_AMBIENT], "DH1_MODE_AMBIENT", "Ambient", ISS_OFF);
  IUFillSwitch(&DH1ModeS[MODE_MIDPOINT], "DH1_MODE_MIDPOINT", "Midpoint", ISS_OFF);
  IUFillSwitch(&DH1ModeS[MODE_SLAVE], "DH1_MODE_SLAVE", "Slave", ISS_OFF);
  IUFillSwitchVector(&DH1ModeSP, DH1ModeS, 5, getDeviceName(), "DH1_MODE",
      "Dewheater 1 mode", DEW_TAB, IP_RW, ISR_1OFMANY, TIMEOUT, IPS_IDLE);

  IUFillNumber(&DH1StatusN[DH_DC], "DH1_DC", "Dutycycle [%]", "%.1f", 0.0, 100.0, 0.1, 0.0);
  IUFillNumber(&DH1StatusN[DH_TEMPERATURE], "DH1_TEMPERATURE", "Temperature [°C]", "%.1f", -100, 200, 0.1, 0.0);
  IUFillNumberVector(&DH1StatusNP, DH1StatusN, 2, getDeviceName(), "DH1_STATUS",
      "Dewheater 1 status", DEW_TAB, IP_RO, TIMEOUT, IPS_IDLE);

  IUFillNumber(&DH1TemperatureOffsetN[0], "DH1_TEMPERATURE_OFFSET",
      "Temperature offset [°C]", "%.1f", -10.0, 10.0, 0.1, 0.0);
  IUFillNumberVector(&DH1TemperatureOffsetNP, DH1TemperatureOffsetN, 1, getDeviceName(),
      "DH1_TEMPERATURE_OFFSET", "Dewheater 1", DEW_TAB, IP_RW, TIMEOUT, IPS_IDLE);

  IUFillNumber(&DH1FixedN[0], "DH1_FIXED",
      "Fixed [%]", "%.1f", 0.0, 100.0, 0.1, 0.0);
  IUFillNumberVector(&DH1FixedNP, DH1FixedN, 1, getDeviceName(),
      "DH1_FIXED", "Dewheater 1", DEW_TAB, IP_RW, TIMEOUT, IPS_IDLE);

  IUFillNumber(&DH1DewpointOffsetN[0], "DH1_DEWPOINT_OFFSET",
      "DewpointOffset [°C]", "%.1f", 0.0, 10.0, 0.1, 0.0);
  IUFillNumberVector(&DH1DewpointOffsetNP, DH1DewpointOffsetN, 1, getDeviceName(),
      "DH1_DEWPOINT_OFFSET", "Dewheater 1", DEW_TAB, IP_RW, TIMEOUT, IPS_IDLE);

  IUFillNumber(&DH1AmbientOffsetN[0], "DH1_AMBIENT_OFFSET",
      "AmbientOffset [°C]", "%.1f", 0.0, 10.0, 0.1, 0.0);
  IUFillNumberVector(&DH1AmbientOffsetNP, DH1AmbientOffsetN, 1, getDeviceName(),
      "DH1_AMBIENT_OFFSET", "Dewheater 1", DEW_TAB, IP_RW, TIMEOUT, IPS_IDLE);

  IUFillNumber(&DH1MidpointOffsetN[0], "DH1_MIDPOINT_OFFSET",
      "MidpointOffset [°C]", "%.1f", -10.0, 10.0, 0.1, 0.0);
  IUFillNumberVector(&DH1MidpointOffsetNP, DH1MidpointOffsetN, 1, getDeviceName(),
      "DH1_MIDPOINT_OFFSET", "Dewheater 1", DEW_TAB, IP_RW, TIMEOUT, IPS_IDLE);

  IUFillSwitch(&DH2ModeS[MODE_FIXED], "DH2_MODE_FIXED", "Fixed", ISS_OFF);
  IUFillSwitch(&DH2ModeS[MODE_DEWPOINT], "DH2_MODE_DEWPOINT", "Dewpoint", ISS_OFF);
  IUFillSwitch(&DH2ModeS[MODE_AMBIENT], "DH2_MODE_AMBIENT", "Ambient", ISS_OFF);
  IUFillSwitch(&DH2ModeS[MODE_MIDPOINT], "DH2_MODE_MIDPOINT", "Midpoint", ISS_OFF);
  IUFillSwitch(&DH2ModeS[MODE_SLAVE], "DH2_MODE_SLAVE", "Slave", ISS_OFF);
  IUFillSwitchVector(&DH2ModeSP, DH2ModeS, 5, getDeviceName(), "DH2_MODE",
      "Dewheater 2 mode", DEW_TAB, IP_RW, ISR_1OFMANY, TIMEOUT, IPS_IDLE);

  IUFillNumber(&DH2StatusN[DH_DC], "DH2_DC", "Dutycycle [%]", "%.1f", 0.0, 100.0, 0.1, 0.0);
  IUFillNumber(&DH2StatusN[DH_TEMPERATURE], "DH2_TEMPERATURE", "Temperature [°C]", "%.1f", -100, 200, 0.1, 0.0);
  IUFillNumberVector(&DH2StatusNP, DH2StatusN, 2, getDeviceName(), "DH2_STATUS",
      "Dewheater 2 status", DEW_TAB, IP_RO, TIMEOUT, IPS_IDLE);

  IUFillNumber(&DH2TemperatureOffsetN[0], "DH2_TEMPERATURE_OFFSET",
      "Temperature offset [°C]", "%.1f", -10.0, 10.0, 0.1, 0.0);
  IUFillNumberVector(&DH2TemperatureOffsetNP, DH2TemperatureOffsetN, 1, getDeviceName(),
      "DH2_TEMPERATURE_OFFSET", "Dewheater 2", DEW_TAB, IP_RW, TIMEOUT, IPS_IDLE);

  IUFillNumber(&DH2FixedN[0], "DH2_FIXED",
      "Fixed [%]", "%.1f", 0.0, 100.0, 0.1, 0.0);
  IUFillNumberVector(&DH2FixedNP, DH2FixedN, 1, getDeviceName(),
      "DH2_FIXED", "Dewheater 2", DEW_TAB, IP_RW, TIMEOUT, IPS_IDLE);

  IUFillNumber(&DH2DewpointOffsetN[0], "DH2_DEWPOINT_OFFSET",
      "DewpointOffset [°C]", "%.1f", 0.0, 10.0, 0.1, 0.0);
  IUFillNumberVector(&DH2DewpointOffsetNP, DH2DewpointOffsetN, 1, getDeviceName(),
      "DH2_DEWPOINT_OFFSET", "Dewheater 2", DEW_TAB, IP_RW, TIMEOUT, IPS_IDLE);

  IUFillNumber(&DH2AmbientOffsetN[0], "DH2_AMBIENT_OFFSET",
      "AmbientOffset [°C]", "%.1f", 0.0, 10.0, 0.1, 0.0);
  IUFillNumberVector(&DH2AmbientOffsetNP, DH2AmbientOffsetN, 1, getDeviceName(),
      "DH2_AMBIENT_OFFSET", "Dewheater 2", DEW_TAB, IP_RW, TIMEOUT, IPS_IDLE);

  IUFillNumber(&DH2MidpointOffsetN[0], "DH2_MIDPOINT_OFFSET",
      "MidpointOffset [°C]", "%.1f", -10.0, 10.0, 0.1, 0.0);
  IUFillNumberVector(&DH2MidpointOffsetNP, DH2MidpointOffsetN, 1, getDeviceName(),
      "DH2_MIDPOINT_OFFSET", "Dewheater 2", DEW_TAB, IP_RW, TIMEOUT, IPS_IDLE);

  return true;
}

uint16_t Powerbox::crc16_update(uint16_t crc, uint8_t a) {
  // Code from crc16.h from the Arduino core tools
  crc ^= a;
  for (int ii=0; ii<8; ii++) {
    if ( crc & 1 ) {
      crc = (crc >> 1) ^ 0xA001;
    } else {
      crc = (crc >> 1);
    }
  }
  return crc;
}

uint16_t Powerbox::crcCalc(const void *data, size_t n) {
  const uint8_t *ptr = static_cast<const uint8_t *>(data);
  uint16_t crc = 0;
  for (size_t ii=0; ii<n; ii++) {
    crc = crc16_update(crc, ptr[ii]);
  }
  return crc;
}

uint16_t Powerbox::crcCalc(const char *str) {
  return crcCalc(static_cast<const void *>(str), strlen(str));
}

bool Powerbox::loadConfig(bool silent, const char *property) {
  // Everything ignored here, all parameters come from the device
  (void) silent;
  (void) property;
  return true;
}

bool Powerbox::updateProperties() {
  INDI::DefaultDevice::updateProperties();
  if ( isConnected() ) {
    defineProperty(&VoltageNP);
    defineProperty(&EnvNP);
    defineProperty(&EnvOffsetNP);
    defineProperty(&RailSP);
    defineProperty(&AdjSP);
    defineProperty(&AdjNP);
    defineProperty(&DH1ModeSP);
    defineProperty(&DH1StatusNP);
    defineProperty(&DH1TemperatureOffsetNP);
    defineProperty(&DH1FixedNP);
    defineProperty(&DH1DewpointOffsetNP);
    defineProperty(&DH1AmbientOffsetNP);
    defineProperty(&DH1MidpointOffsetNP);
    defineProperty(&DH2ModeSP);
    defineProperty(&DH2StatusNP);
    defineProperty(&DH2TemperatureOffsetNP);
    defineProperty(&DH2FixedNP);
    defineProperty(&DH2DewpointOffsetNP);
    defineProperty(&DH2AmbientOffsetNP);
    defineProperty(&DH2MidpointOffsetNP);
    update();
  } else {
    deleteProperty(VoltageNP.name);
    deleteProperty(EnvNP.name);
    deleteProperty(EnvOffsetNP.name);
    deleteProperty(RailSP.name);
    deleteProperty(AdjSP.name);
    deleteProperty(AdjNP.name);
    deleteProperty(DH1ModeSP.name);
    deleteProperty(DH1StatusNP.name);
    deleteProperty(DH1TemperatureOffsetNP.name);
    deleteProperty(DH1FixedNP.name);
    deleteProperty(DH1DewpointOffsetNP.name);
    deleteProperty(DH1AmbientOffsetNP.name);
    deleteProperty(DH1MidpointOffsetNP.name);
    deleteProperty(DH2ModeSP.name);
    deleteProperty(DH2StatusNP.name);
    deleteProperty(DH2TemperatureOffsetNP.name);
    deleteProperty(DH2FixedNP.name);
    deleteProperty(DH2DewpointOffsetNP.name);
    deleteProperty(DH2AmbientOffsetNP.name);
    deleteProperty(DH2MidpointOffsetNP.name);
  }
  return true;
}

void Powerbox::setVoltage(const json& data) {
  try {
    VoltageN[0].value = data["V"].template get<double>();
    VoltageNP.s = IPS_OK;
    IDSetNumber(&VoltageNP, nullptr);
  } catch (...) {
    VoltageNP.s = IPS_ALERT;
    IDSetNumber(&VoltageNP, nullptr);
    throw;
  }
}

void Powerbox::setEnvironment(const json& data) {
  try {
    EnvN[TEMPERATURE].value = data["E"]["T"].template get<double>();
    EnvN[HUMIDITY].value = data["E"]["H"].template get<double>();
    EnvN[PRESSURE].value = data["E"]["P"].template get<double>();
    EnvN[DEWPOINT].value = data["E"]["D"].template get<double>();
    EnvNP.s = IPS_OK;
    IDSetNumber(&EnvNP, nullptr);
  } catch (...) {
    EnvNP.s = IPS_ALERT;
    IDSetNumber(&EnvNP, nullptr);
    throw;
  }
}

void Powerbox::setEnvironmentOffset(const json& data) {
  try {
    EnvOffsetN[0].value = data["E"]["dT"].template get<double>();
    EnvOffsetNP.s = IPS_OK;
    IDSetNumber(&EnvOffsetNP, nullptr);
  } catch (...) {
    EnvOffsetNP.s = IPS_ALERT;
    IDSetNumber(&EnvOffsetNP, nullptr);
    throw;
  }
}

void Powerbox::setRail(const json& data) {
  try {
    bool isOn = data["R"].template get<bool>();
    if ( isOn ) {
      RailS[ON].s = ISS_ON;
      RailS[OFF].s = ISS_OFF;
    } else {
      RailS[ON].s = ISS_OFF;
      RailS[OFF].s = ISS_ON;
    }
    RailSP.s = IPS_OK;
    IDSetSwitch(&RailSP, nullptr);
  } catch (...) {
    RailSP.s = IPS_ALERT;
    IDSetSwitch(&RailSP, nullptr);
  }
}

void Powerbox::setAdjState(const json& data) {
  try {
    bool isOn = data["A"]["ON"].template get<bool>();
    if ( isOn ) {
      AdjS[ON].s = ISS_ON;
      AdjS[OFF].s = ISS_OFF;
    } else {
      AdjS[ON].s = ISS_OFF;
      AdjS[OFF].s = ISS_ON;
    }
    AdjSP.s = IPS_OK;
    IDSetSwitch(&AdjSP, nullptr);
  } catch (...) {
    AdjSP.s = IPS_ALERT;
    IDSetSwitch(&AdjSP, nullptr);
    throw;
  }
}

void Powerbox::setAdjVoltage(const json& data) {
  try {
    AdjN[0].value = data["A"]["V"].template get<double>();
    AdjNP.s = IPS_OK;
    IDSetNumber(&AdjNP, nullptr);
  } catch (...) {
    AdjNP.s = IPS_ALERT;
    IDSetNumber(&AdjNP, nullptr);
    throw;
  }
}

void Powerbox::setDH1Mode(const json& data) {
  try {
    int mode = data["DH1"]["M"].template get<int>();
    for (int ii=0; ii<5; ii++) {
      DH1ModeS[ii].s = mode == ii ? ISS_ON : ISS_OFF;
    }
    DH1ModeSP.s = IPS_OK;
    IDSetSwitch(&DH1ModeSP, nullptr);
  } catch (...) {
    DH1ModeSP.s = IPS_ALERT;
    IDSetSwitch(&DH1ModeSP, nullptr);
    throw;
  }
}

void Powerbox::setDH1Status(const json& data) {
  try {
    DH1StatusN[DH_DC].value = data["DH1"]["DC"].template get<double>();
    DH1StatusN[DH_TEMPERATURE].value = data["DH1"]["T"].template get<double>();
    DH1StatusNP.s = IPS_OK;
    IDSetNumber(&DH1StatusNP, nullptr);
  } catch (...) {
    DH1StatusNP.s = IPS_ALERT;
    IDSetNumber(&DH1StatusNP, nullptr);
    throw;
  }
}

void Powerbox::setDH1Params(const json& data) {
  try {
    DH1TemperatureOffsetN[0].value = data["DH1"]["dT"].template get<double>();
    DH1TemperatureOffsetNP.s = IPS_OK;
    IDSetNumber(&DH1TemperatureOffsetNP, nullptr);
  } catch (...) {
    DH1TemperatureOffsetNP.s = IPS_ALERT;
    IDSetNumber(&DH1TemperatureOffsetNP, nullptr);
    throw;
  }
  try {
    DH1FixedN[0].value = data["DH1"]["F"].template get<double>();
    DH1FixedNP.s = IPS_OK;
    IDSetNumber(&DH1FixedNP, nullptr);
  } catch (...) {
    DH1FixedNP.s = IPS_ALERT;
    IDSetNumber(&DH1FixedNP, nullptr);
    throw;
  }
  try {
    DH1DewpointOffsetN[0].value = data["DH1"]["OD"].template get<double>();
    DH1DewpointOffsetNP.s = IPS_OK;
    IDSetNumber(&DH1DewpointOffsetNP, nullptr);
  } catch (...) {
    DH1DewpointOffsetNP.s = IPS_ALERT;
    IDSetNumber(&DH1DewpointOffsetNP, nullptr);
    throw;
  }
  try {
    DH1AmbientOffsetN[0].value = data["DH1"]["OA"].template get<double>();
    DH1AmbientOffsetNP.s = IPS_OK;
    IDSetNumber(&DH1AmbientOffsetNP, nullptr);
  } catch (...) {
    DH1AmbientOffsetNP.s = IPS_ALERT;
    IDSetNumber(&DH1AmbientOffsetNP, nullptr);
    throw;
  }
  try {
    DH1MidpointOffsetN[0].value = data["DH1"]["OM"].template get<double>();
    DH1MidpointOffsetNP.s = IPS_OK;
    IDSetNumber(&DH1MidpointOffsetNP, nullptr);
  } catch (...) {
    DH1MidpointOffsetNP.s = IPS_ALERT;
    IDSetNumber(&DH1MidpointOffsetNP, nullptr);
    throw;
  }
}

void Powerbox::setDH2Mode(const json& data) {
  try {
    int mode = data["DH2"]["M"].template get<int>();
    for (int ii=0; ii<5; ii++) {
      DH2ModeS[ii].s = mode == ii ? ISS_ON : ISS_OFF;
    }
    DH2ModeSP.s = IPS_OK;
    IDSetSwitch(&DH2ModeSP, nullptr);
  } catch (...) {
    DH2ModeSP.s = IPS_ALERT;
    IDSetSwitch(&DH2ModeSP, nullptr);
    throw;
  }
}

void Powerbox::setDH2Status(const json& data) {
  try {
    DH2StatusN[DH_DC].value = data["DH2"]["DC"].template get<double>();
    DH2StatusN[DH_TEMPERATURE].value = data["DH2"]["T"].template get<double>();
    DH2StatusNP.s = IPS_OK;
    IDSetNumber(&DH2StatusNP, nullptr);
  } catch (...) {
    DH2StatusNP.s = IPS_ALERT;
    IDSetNumber(&DH2StatusNP, nullptr);
    throw;
  }
}

void Powerbox::setDH2Params(const json& data) {
  try {
    DH2TemperatureOffsetN[0].value = data["DH2"]["dT"].template get<double>();
    DH2TemperatureOffsetNP.s = IPS_OK;
    IDSetNumber(&DH2TemperatureOffsetNP, nullptr);
  } catch (...) {
    DH2TemperatureOffsetNP.s = IPS_ALERT;
    IDSetNumber(&DH2TemperatureOffsetNP, nullptr);
    throw;
  }
  try {
    DH2FixedN[0].value = data["DH2"]["F"].template get<double>();
    DH2FixedNP.s = IPS_OK;
    IDSetNumber(&DH2FixedNP, nullptr);
  } catch (...) {
    DH2FixedNP.s = IPS_ALERT;
    IDSetNumber(&DH2FixedNP, nullptr);
    throw;
  }
  try {
    DH2DewpointOffsetN[0].value = data["DH2"]["OD"].template get<double>();
    DH2DewpointOffsetNP.s = IPS_OK;
    IDSetNumber(&DH2DewpointOffsetNP, nullptr);
  } catch (...) {
    DH2DewpointOffsetNP.s = IPS_ALERT;
    IDSetNumber(&DH2DewpointOffsetNP, nullptr);
    throw;
  }
  try {
    DH2AmbientOffsetN[0].value = data["DH2"]["OA"].template get<double>();
    DH2AmbientOffsetNP.s = IPS_OK;
    IDSetNumber(&DH2AmbientOffsetNP, nullptr);
  } catch (...) {
    DH2AmbientOffsetNP.s = IPS_ALERT;
    IDSetNumber(&DH2AmbientOffsetNP, nullptr);
    throw;
  }
  try {
    DH2MidpointOffsetN[0].value = data["DH2"]["OM"].template get<double>();
    DH2MidpointOffsetNP.s = IPS_OK;
    IDSetNumber(&DH2MidpointOffsetNP, nullptr);
  } catch (...) {
    DH2MidpointOffsetNP.s = IPS_ALERT;
    IDSetNumber(&DH2MidpointOffsetNP, nullptr);
    throw;
  }
}

bool Powerbox::updateFromResponse(const char *rsp) {
  LOGF_DEBUG("RSP: %s", rsp);
  json data;
  try {
    data = json::parse(rsp);
  } catch (...) {
    LOGF_ERROR("Error parsing JSON: %s", rsp);
    return false;
  }
  if ( data.contains("Error") ) {
    LOGF_ERROR("Device error: %s", data["Error"].template get<std::string>().c_str());
    return false;
  }
  try {
    setVoltage(data);
    setEnvironment(data);
    setEnvironmentOffset(data);
    setRail(data);
    setAdjState(data);
    setAdjVoltage(data);
    setDH1Mode(data);
    setDH1Status(data);
    setDH1Params(data);
    setDH2Mode(data);
    setDH2Status(data);
    setDH2Params(data);
  } catch (...) {
    LOG_ERROR("Could not decode values from device");
    return false;
  }
  return true;
}

bool Powerbox::update() {
  char cmd[] = "status";
  char rsp[RSPBUFF];
  if ( ! sendCommand(cmd, rsp) ) {
    return false;
  }
  return updateFromResponse(rsp);
}

void Powerbox::TimerHit() {
  if ( isConnected() ) {
    update();
  }
  SetTimer(getCurrentPollingPeriod());
}

bool Powerbox::processRailSP(ISState * states, char * names[], int n) {
  bool commandStatus;
  char rsp[RSPBUFF];
  const char *actionName = IUFindOnSwitchName(states, names, n);
  int currentStatus = IUFindOnSwitchIndex(&RailSP);
  if ( strcmp(actionName, RailS[currentStatus].name) == 0 ) {
    RailSP.s = IPS_OK;
    IDSetSwitch(&RailSP, nullptr);
    return true;
  }
  if ( strcmp(actionName, RailS[ON].name) == 0 ) {
    commandStatus = sendCommand("rail on", rsp);
  } else if ( strcmp(actionName, RailS[OFF].name) == 0 ) {
    commandStatus = sendCommand("rail off", rsp);
  } else {
    RailSP.s = IPS_ALERT;
    IDSetSwitch(&RailSP, nullptr);
    LOGF_ERROR("Unknown requested switch state: %s", actionName);
    return false;
  }
  if ( ! commandStatus || ! updateFromResponse(rsp) ) {
    RailSP.s = IPS_ALERT;
    IDSetSwitch(&RailSP, nullptr);
    return false;
  }
  RailSP.s = IPS_OK;
  IDSetSwitch(&RailSP, nullptr);
  return true;
}

bool Powerbox::processAdjSP(ISState * states, char * names[], int n) {
  bool commandStatus;
  char rsp[RSPBUFF];
  const char *actionName = IUFindOnSwitchName(states, names, n);
  int currentStatus = IUFindOnSwitchIndex(&AdjSP);
  if ( strcmp(actionName, AdjS[currentStatus].name) == 0 ) {
    AdjSP.s = IPS_OK;
    IDSetSwitch(&AdjSP, nullptr);
    return true;
  }
  if ( strcmp(actionName, AdjS[ON].name) == 0 ) {
    commandStatus = sendCommand("adj on", rsp);
  } else if ( strcmp(actionName, AdjS[OFF].name) == 0 ) {
    commandStatus = sendCommand("adj off", rsp);
  } else {
    AdjSP.s = IPS_ALERT;
    IDSetSwitch(&AdjSP, nullptr);
    LOGF_ERROR("Unknown requested switch state: %s", actionName);
    return false;
  }
  if ( ! commandStatus || ! updateFromResponse(rsp) ) {
    AdjSP.s = IPS_ALERT;
    IDSetSwitch(&AdjSP, nullptr);
    return false;
  }
  AdjSP.s = IPS_OK;
  IDSetSwitch(&AdjSP, nullptr);
  return true;
}

bool Powerbox::processDH1Mode(ISState * states, char * names[], int n) {
  char rsp[RSPBUFF];
  char cmd[CMDBUFF];
  int mode;
  const char *actionName = IUFindOnSwitchName(states, names, n);
  int currentStatus = IUFindOnSwitchIndex(&DH1ModeSP);
  if ( strcmp(actionName, DH1ModeS[currentStatus].name) == 0 ) {
    DH1ModeSP.s = IPS_OK;
    IDSetSwitch(&DH1ModeSP, nullptr);
    return true;
  }
  if ( strcmp(actionName, DH1ModeS[MODE_FIXED].name) == 0 ) {
    mode = MODE_FIXED;
  } else if ( strcmp(actionName, DH1ModeS[MODE_DEWPOINT].name) == 0 ) {
    mode = MODE_DEWPOINT;
  } else if ( strcmp(actionName, DH1ModeS[MODE_AMBIENT].name) == 0 ) {
    mode = MODE_AMBIENT;
  } else if ( strcmp(actionName, DH1ModeS[MODE_MIDPOINT].name) == 0 ) {
    mode = MODE_MIDPOINT;
  } else if ( strcmp(actionName, DH1ModeS[MODE_SLAVE].name) == 0 ) {
    mode = MODE_SLAVE;
  } else {
    LOGF_ERROR("Unknown requested switch state: %s", actionName);
    DH1ModeSP.s = IPS_ALERT;
    IDSetSwitch(&DH1ModeSP, nullptr);
    return false;
  }
  snprintf(cmd, CMDBUFF, "DH1 mode %d", mode);
  LOGF_DEBUG("CMD: %s", cmd);
  if ( ! sendCommand(cmd, rsp) || ! updateFromResponse(rsp) ) {
    DH1ModeSP.s = IPS_ALERT;
    IDSetSwitch(&DH1ModeSP, nullptr);
    return false;
  }
  DH1ModeSP.s = IPS_OK;
  IDSetSwitch(&DH1ModeSP, nullptr);
  return true;
}

bool Powerbox::processDH1TemperatureOffset(double *values) {
  char rsp[RSPBUFF];
  char cmd[CMDBUFF];
  snprintf(cmd, CMDBUFF, "DH1 offset %d", static_cast<int>(100 * values[0]));
  if ( ! sendCommand(cmd, rsp) || ! updateFromResponse(rsp) ) {
    DH1TemperatureOffsetNP.s = IPS_ALERT;
    IDSetNumber(&DH1TemperatureOffsetNP, nullptr);
    return false;
  }
  DH1TemperatureOffsetNP.s = IPS_OK;
  IDSetNumber(&DH1TemperatureOffsetNP, nullptr);
  return true;
}

bool Powerbox::processDH1Fixed(double *values) {
  char rsp[RSPBUFF];
  char cmd[CMDBUFF];
  snprintf(cmd, CMDBUFF, "DH1 fixed %d", static_cast<int>(100 * values[0]));
  if ( ! sendCommand(cmd, rsp) || ! updateFromResponse(rsp) ) {
    DH1FixedNP.s = IPS_ALERT;
    IDSetNumber(&DH1FixedNP, nullptr);
    return false;
  }
  DH1FixedNP.s = IPS_OK;
  IDSetNumber(&DH1FixedNP, nullptr);
  return true;
}

bool Powerbox::processDH1DewpointOffset(double *values) {
  char rsp[RSPBUFF];
  char cmd[CMDBUFF];
  snprintf(cmd, CMDBUFF, "DH1 oD %d", static_cast<int>(100 * values[0]));
  if ( ! sendCommand(cmd, rsp) || ! updateFromResponse(rsp) ) {
    DH1DewpointOffsetNP.s = IPS_ALERT;
    IDSetNumber(&DH1DewpointOffsetNP, nullptr);
    return false;
  }
  DH1DewpointOffsetNP.s = IPS_OK;
  IDSetNumber(&DH1DewpointOffsetNP, nullptr);
  return true;
}

bool Powerbox::processDH1AmbientOffset(double *values) {
  char rsp[RSPBUFF];
  char cmd[CMDBUFF];
  snprintf(cmd, CMDBUFF, "DH1 oA %d", static_cast<int>(100 * values[0]));
  if ( ! sendCommand(cmd, rsp) || ! updateFromResponse(rsp) ) {
    DH1AmbientOffsetNP.s = IPS_ALERT;
    IDSetNumber(&DH1AmbientOffsetNP, nullptr);
    return false;
  }
  DH1AmbientOffsetNP.s = IPS_OK;
  IDSetNumber(&DH1AmbientOffsetNP, nullptr);
  return true;
}

bool Powerbox::processDH1MidpointOffset(double *values) {
  char rsp[RSPBUFF];
  char cmd[CMDBUFF];
  snprintf(cmd, CMDBUFF, "DH1 oM %d", static_cast<int>(100 * values[0]));
  if ( ! sendCommand(cmd, rsp) || ! updateFromResponse(rsp) ) {
    DH1MidpointOffsetNP.s = IPS_ALERT;
    IDSetNumber(&DH1MidpointOffsetNP, nullptr);
    return false;
  }
  DH1MidpointOffsetNP.s = IPS_OK;
  IDSetNumber(&DH1MidpointOffsetNP, nullptr);
  return true;
}

bool Powerbox::processDH2Mode(ISState * states, char * names[], int n) {
  char rsp[RSPBUFF];
  char cmd[CMDBUFF];
  int mode;
  const char *actionName = IUFindOnSwitchName(states, names, n);
  int currentStatus = IUFindOnSwitchIndex(&DH2ModeSP);
  if ( strcmp(actionName, DH2ModeS[currentStatus].name) == 0 ) {
    DH2ModeSP.s = IPS_OK;
    IDSetSwitch(&DH2ModeSP, nullptr);
    return true;
  }
  if ( strcmp(actionName, DH2ModeS[MODE_FIXED].name) == 0 ) {
    mode = MODE_FIXED;
  } else if ( strcmp(actionName, DH2ModeS[MODE_DEWPOINT].name) == 0 ) {
    mode = MODE_DEWPOINT;
  } else if ( strcmp(actionName, DH2ModeS[MODE_AMBIENT].name) == 0 ) {
    mode = MODE_AMBIENT;
  } else if ( strcmp(actionName, DH2ModeS[MODE_MIDPOINT].name) == 0 ) {
    mode = MODE_MIDPOINT;
  } else if ( strcmp(actionName, DH2ModeS[MODE_SLAVE].name) == 0 ) {
    mode = MODE_SLAVE;
  } else {
    LOGF_ERROR("Unknown requested switch state: %s", actionName);
    DH2ModeSP.s = IPS_ALERT;
    IDSetSwitch(&DH2ModeSP, nullptr);
    return false;
  }
  snprintf(cmd, CMDBUFF, "DH2 mode %d", mode);
  LOGF_DEBUG("CMD: %s", cmd);
  if ( ! sendCommand(cmd, rsp) || ! updateFromResponse(rsp) ) {
    DH2ModeSP.s = IPS_ALERT;
    IDSetSwitch(&DH2ModeSP, nullptr);
    return false;
  }
  DH2ModeSP.s = IPS_OK;
  IDSetSwitch(&DH2ModeSP, nullptr);
  return true;
}

bool Powerbox::processDH2TemperatureOffset(double *values) {
  char rsp[RSPBUFF];
  char cmd[CMDBUFF];
  snprintf(cmd, CMDBUFF, "DH2 offset %d", static_cast<int>(100 * values[0]));
  if ( ! sendCommand(cmd, rsp) || ! updateFromResponse(rsp) ) {
    DH2TemperatureOffsetNP.s = IPS_ALERT;
    IDSetNumber(&DH2TemperatureOffsetNP, nullptr);
    return false;
  }
  DH2TemperatureOffsetNP.s = IPS_OK;
  IDSetNumber(&DH2TemperatureOffsetNP, nullptr);
  return true;
}

bool Powerbox::processDH2Fixed(double *values) {
  char rsp[RSPBUFF];
  char cmd[CMDBUFF];
  snprintf(cmd, CMDBUFF, "DH2 fixed %d", static_cast<int>(100 * values[0]));
  if ( ! sendCommand(cmd, rsp) || ! updateFromResponse(rsp) ) {
    DH2FixedNP.s = IPS_ALERT;
    IDSetNumber(&DH2FixedNP, nullptr);
    return false;
  }
  DH2FixedNP.s = IPS_OK;
  IDSetNumber(&DH2FixedNP, nullptr);
  return true;
}

bool Powerbox::processDH2DewpointOffset(double *values) {
  char rsp[RSPBUFF];
  char cmd[CMDBUFF];
  snprintf(cmd, CMDBUFF, "DH2 oD %d", static_cast<int>(100 * values[0]));
  if ( ! sendCommand(cmd, rsp) || ! updateFromResponse(rsp) ) {
    DH2DewpointOffsetNP.s = IPS_ALERT;
    IDSetNumber(&DH2DewpointOffsetNP, nullptr);
    return false;
  }
  DH2DewpointOffsetNP.s = IPS_OK;
  IDSetNumber(&DH2DewpointOffsetNP, nullptr);
  return true;
}

bool Powerbox::processDH2AmbientOffset(double *values) {
  char rsp[RSPBUFF];
  char cmd[CMDBUFF];
  snprintf(cmd, CMDBUFF, "DH2 oA %d", static_cast<int>(100 * values[0]));
  if ( ! sendCommand(cmd, rsp) || ! updateFromResponse(rsp) ) {
    DH2AmbientOffsetNP.s = IPS_ALERT;
    IDSetNumber(&DH2AmbientOffsetNP, nullptr);
    return false;
  }
  DH2AmbientOffsetNP.s = IPS_OK;
  IDSetNumber(&DH2AmbientOffsetNP, nullptr);
  return true;
}

bool Powerbox::processDH2MidpointOffset(double *values) {
  char rsp[RSPBUFF];
  char cmd[CMDBUFF];
  snprintf(cmd, CMDBUFF, "DH2 oM %d", static_cast<int>(100 * values[0]));
  if ( ! sendCommand(cmd, rsp) || ! updateFromResponse(rsp) ) {
    DH2MidpointOffsetNP.s = IPS_ALERT;
    IDSetNumber(&DH2MidpointOffsetNP, nullptr);
    return false;
  }
  DH2MidpointOffsetNP.s = IPS_OK;
  IDSetNumber(&DH2MidpointOffsetNP, nullptr);
  return true;
}

bool Powerbox::ISNewSwitch(const char * dev, const char * name, ISState * states, char * names[], int n) {
  if (dev != nullptr && strcmp(dev, getDeviceName()) == 0) {
    if ( strcmp(name, RailSP.name) == 0 ) {
      return processRailSP(states, names, n);
    }
    if ( strcmp(name, AdjSP.name) == 0 ) {
      return processAdjSP(states, names, n);
    }
    if ( strcmp(name, DH1ModeSP.name) == 0 ) {
      return processDH1Mode(states, names, n);
    }
    if ( strcmp(name, DH2ModeSP.name) == 0 ) {
      return processDH2Mode(states, names, n);
    }
  }
  return INDI::DefaultDevice::ISNewSwitch(dev, name, states, names, n);
}

bool Powerbox::processEnvOffsetNP(double *values) {
  char cmd[CMDBUFF];
  char rsp[RSPBUFF];
  snprintf(cmd, CMDBUFF, "env offset %d", static_cast<int>(values[0] * 100));
  if ( ! sendCommand(cmd, rsp) || ! updateFromResponse(rsp) ) {
    EnvOffsetNP.s = IPS_ALERT;
    IDSetNumber(&EnvOffsetNP, nullptr);
    return false;
  }
  EnvOffsetNP.s = IPS_OK;
  IDSetNumber(&EnvOffsetNP, nullptr);
  return true;
}

bool Powerbox::processAdjNP(double *values) {
  char cmd[CMDBUFF];
  char rsp[RSPBUFF];
  snprintf(cmd, CMDBUFF, "adj %d", static_cast<int>(values[0] * 100));
  if ( ! sendCommand(cmd, rsp) || ! updateFromResponse(rsp) ) {
    AdjNP.s = IPS_ALERT;
    IDSetNumber(&AdjNP, nullptr);
    return false;
  }
  AdjNP.s = IPS_OK;
  IDSetNumber(&AdjNP, nullptr);
  return true;
}

bool Powerbox::ISNewNumber(const char *dev, const char *name, double *values, char *names[], int n) {
  if (dev != nullptr && strcmp(dev, getDeviceName()) == 0) {
    if ( strcmp(name, EnvOffsetNP.name) == 0 ) {
      return processEnvOffsetNP(values);
    }
    if ( strcmp(name, AdjNP.name) == 0 ) {
      return processAdjNP(values);
    }
    // DH1
    if ( strcmp(name, DH1TemperatureOffsetNP.name) == 0 ) {
      return processDH1TemperatureOffset(values);
    }
    if ( strcmp(name, DH1FixedNP.name) == 0 ) {
      return processDH1Fixed(values);
    }
    if ( strcmp(name, DH1DewpointOffsetNP.name) == 0 ) {
      return processDH1DewpointOffset(values);
    }
    if ( strcmp(name, DH1AmbientOffsetNP.name) == 0 ) {
      return processDH1AmbientOffset(values);
    }
    if ( strcmp(name, DH1MidpointOffsetNP.name) == 0 ) {
      return processDH1MidpointOffset(values);
    }
    // DH2
    if ( strcmp(name, DH2TemperatureOffsetNP.name) == 0 ) {
      return processDH2TemperatureOffset(values);
    }
    if ( strcmp(name, DH2FixedNP.name) == 0 ) {
      return processDH2Fixed(values);
    }
    if ( strcmp(name, DH2DewpointOffsetNP.name) == 0 ) {
      return processDH2DewpointOffset(values);
    }
    if ( strcmp(name, DH2AmbientOffsetNP.name) == 0 ) {
      return processDH2AmbientOffset(values);
    }
    if ( strcmp(name, DH2MidpointOffsetNP.name) == 0 ) {
      return processDH2MidpointOffset(values);
    }
  }
  return INDI::DefaultDevice::ISNewNumber(dev, name, values, names, n);
}
