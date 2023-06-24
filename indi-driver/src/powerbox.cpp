#include "powerbox.h"
#include <defaultdevice.h>

using json = nlohmann::json;

static std::unique_ptr<Powerbox> shelyakDriver(new Powerbox());

Powerbox::Powerbox() {
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
  out[len+3] = ((char *) &crc)[1];
  out[len+4] = '$';
  out[len+5] = '\0';
}

bool Powerbox::checkCrc(const char *rsp) {
  uint16_t crcGotten;
  size_t len = strlen(rsp);
  ((char *) &crcGotten)[0] = rsp[len+1];
  ((char *) &crcGotten)[1] = rsp[len+2];
  uint16_t crcCalculated = crcCalc((const void *) rsp, len);
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
    update();
  } else {
    deleteProperty(VoltageNP.name);
    deleteProperty(EnvNP.name);
    deleteProperty(EnvOffsetNP.name);
    deleteProperty(RailSP.name);
    deleteProperty(AdjSP.name);
    deleteProperty(AdjNP.name);
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

bool Powerbox::updateFromResponse(const char *rsp) {
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

bool Powerbox::ISNewSwitch(const char * dev, const char * name, ISState * states, char * names[], int n) {
  if (dev != nullptr && strcmp(dev, getDeviceName()) == 0) {
    if ( strcmp(name, RailSP.name) == 0 ) {
      return processRailSP(states, names, n);
    }
    if ( strcmp(name, AdjSP.name) == 0 ) {
      return processAdjSP(states, names, n);
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
  }
  return INDI::DefaultDevice::ISNewNumber(dev, name, values, names, n);
}
