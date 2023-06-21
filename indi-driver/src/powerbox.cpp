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
  out[len+1] = ((char *) &crc)[0];
  out[len+2] = ((char *) &crc)[1];
  out[len+3] = '$';
  out[len+4] = '\0';
}

bool Powerbox::checkCrc(const char *rsp) {
  uint16_t crcGotten;
  size_t len = strlen(rsp) - 2;
  ((char *) &crcGotten)[0] = rsp[len+0];
  ((char *) &crcGotten)[1] = rsp[len+1];
  uint16_t crcCalculated = crcCalc((const void *) rsp, len);
  if ( crcGotten == crcCalculated ) {
    return true;
  } else {
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
  if ( (rc = tty_write(PortFD, buff, strlen(buff), &nbytes_written)) != TTY_OK ) {
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
  rspBuff[nbytes_read+1] = '\0';

  LOGF_DEBUG("RSP: %s", rspBuff);
  memcpy(rsp, rspBuff+1, strlen(rspBuff)-2);
  rsp[strlen(rspBuff)-3] = '\0';
  if ( ! checkCrc(rsp) ) {
    LOG_ERROR("Checksum error");
    return false;
  }
  rsp[strlen(rsp)-2] = '\0';
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
      MAIN_CONTROL_TAB, IP_RO, 0, IPS_IDLE);

  IUFillNumber(&EnvN[TEMPERATURE], "TEMPERATURE", "Temperature [°C]", "%.1f", -100.0, 100.0, 0.1, 0.0);
  IUFillNumber(&EnvN[HUMIDITY], "HUMIDITY", "Rel. Humidity [%]", "%.1f", 0.0, 100.0, 0.1, 50.0);
  IUFillNumber(&EnvN[PRESSURE], "PRESSURE", "Pressure [mbar]", "%.1f", 0.0, 2000.0, 0.1, 1013.0);
  IUFillNumber(&EnvN[DEWPOINT], "DEWPOINT", "Dewpoint [°C]", "%.1f", -100.0, 100.0, 0.1, 0.0);
  IUFillNumberVector(&EnvNP, EnvN, 4, getDeviceName(), "ENVIRONMENT", "Environment",
      MAIN_CONTROL_TAB, IP_RO, 0, IPS_IDLE);
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
    update();
  } else {
    deleteProperty(VoltageNP.name);
    deleteProperty(EnvNP.name);
  }
  return true;
}

void Powerbox::setVoltage(const json& data) {
  VoltageNP.s = IPS_BUSY;
  IDSetNumber(&VoltageNP, nullptr);
  try {
    VoltageN[0].value = data["V"].template get<double>();
    VoltageNP.s = IPS_OK;
  } catch (...) {
    VoltageNP.s = IPS_ALERT;
    IDSetNumber(&VoltageNP, nullptr);
    throw;
  }
  IDSetNumber(&VoltageNP, nullptr);
}

void Powerbox::setEnvironment(const json& data) {
  EnvNP.s = IPS_BUSY;
  IDSetNumber(&EnvNP, nullptr);
  try {
    EnvN[TEMPERATURE].value = data["E"]["T"].template get<double>();
    EnvN[HUMIDITY].value = data["E"]["H"].template get<double>();
    EnvN[PRESSURE].value = data["E"]["P"].template get<double>();
    EnvN[DEWPOINT].value = data["E"]["D"].template get<double>();
    EnvNP.s = IPS_OK;
  } catch (...) {
    EnvNP.s = IPS_ALERT;
    IDSetNumber(&EnvNP, nullptr);
    throw;
  }
  IDSetNumber(&EnvNP, nullptr);
}

bool Powerbox::update() {
  char cmd[] = "status";
  char rsp[RSPBUFF];
  if ( ! sendCommand(cmd, rsp) ) {
    return false;
  }
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
  } catch (...) {
    LOG_ERROR("Could not decode values from device");
    return false;
  }
  return true;
}

void Powerbox::TimerHit() {
  if ( isConnected() ) {
    update();
  }
  SetTimer(getCurrentPollingPeriod());
}
