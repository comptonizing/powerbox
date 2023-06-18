#include "powerbox.h"

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

  if ( (rc = tty_read_section(PortFD, rspBuff, '$', 5, &nbytes_read)) != TTY_OK ) {
    tty_error_msg(rc, err, ERRBUFF);
    LOGF_ERROR("Error reading response: %s", err);
    return false;
  }

  LOGF_DEBUG("RSP: %s", rspBuff);
  memcpy(rsp, rspBuff+1, strlen(rspBuff)-2);
  rsp[strlen(rspBuff)-3] = '\0';
  if ( ! checkCrc(rsp) ) {
    LOG_ERROR("Checksum error");
    return false;
  }
  rsp[strlen(rspBuff)-5] = '\0';

  return true;
}


bool Powerbox::Handshake() {
  char cmd[] = "status";
  char rsp[RSPBUFF];
  if ( ! sendCommand(cmd, rsp) ) {
    return false;
  }
  return true;
}

bool Powerbox::initProperties() {
  DefaultDevice::initProperties();
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
  (void) silent;
  (void) property;
  return true;
}
