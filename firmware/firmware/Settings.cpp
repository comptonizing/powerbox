#include "Settings.h"

#define OFFSET_MAGIC 0
static uint16_t __magic = 0b1110110101010010;

#define OFFSET_CRC (OFFSET_MAGIC + sizeof(__magic))

#define OFFSET_DATA (OFFSET_CRC + sizeof(uint16_t))

static inline uint16_t crc16_update(uint16_t crc, uint8_t a) {
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

Settings &Settings::i() {
  static Settings theInstance;
  return theInstance;
}

void Settings::readEEPROM(uint16_t address, uint8_t *buff, uint16_t n) {
  for (int ii=0; ii<n; ii++) {
    buff[ii] = EEPROM.read(address+ii);
  }
}

void Settings::writeEEPROM(uint16_t address, uint8_t *buff, uint16_t n) {
  for (int ii=0; ii<n; ii++) {
    EEPROM.write(address+ii, buff[ii]);
  }
}

uint16_t Settings::crcCalc(uint8_t *data, uint16_t n) {
  uint16_t crc = 0;
  for (uint16_t ii=0; ii<n; ii++) {
    crc = crc16_update(crc, data[ii]);
  }
  return crc;
}

uint16_t Settings::crcCalc(const char *str) {
  uint16_t crc = 0;
  while ( *str != '\0' ) {
    crc = crc16_update(crc, *str);
    str++;
  }
  return crc;
}

uint16_t Settings::crcCalc(const __FlashStringHelper *data, uint16_t n) {
  const char *ptr = reinterpret_cast<const char *>(data);
  uint16_t crc = 0;
  for (uint16_t ii=0; ii<n; ii++) {
    crc = crc16_update(crc, pgm_read_byte(ptr+ii));
  }
  return crc;
}

bool Settings::loadFromEEPROM() {
  uint16_t magic;
  uint16_t crc;
  uint8_t data[sizeof(Settings)];

  readEEPROM(OFFSET_MAGIC, (uint8_t *) &magic, sizeof(magic));
  if ( magic != __magic ) {
    return false;
  }

  readEEPROM(OFFSET_CRC, (uint8_t *) &crc, sizeof(crc));
  readEEPROM(OFFSET_DATA, data, sizeof(Settings));

  if ( crc != crcCalc(data, sizeof(Settings)) ) {
    return false;
  }

  memcpy((void *) this, (void *) data, sizeof(Settings));
  return true;
}

void Settings::saveToEEPROM() {
  uint16_t crc = crcCalc((uint8_t *) this, sizeof(Settings));
  writeEEPROM(OFFSET_DATA, (uint8_t *) this, sizeof(Settings));
  writeEEPROM(OFFSET_CRC, (uint8_t *) &crc, sizeof(crc));
  writeEEPROM(OFFSET_MAGIC, (uint8_t *) &__magic, sizeof(__magic));
}

Settings::Settings() {
  if ( ! loadFromEEPROM() ) {
    saveToEEPROM();
  }
  setDevices();
}

Settings::~Settings() {
}

void Settings::setRail12V() {
  if ( m_RAIL12V_on ) {
    Devices::i().rail12V.on();
  } else {
    Devices::i().rail12V.off();
  }
}

void Settings::setAdj() {
  Devices::i().adj.setVoltage(m_ADJ_voltage);
  if ( m_ADJ_on ) {
    Devices::i().adj.on();
  } else {
    Devices::i().adj.off();
  }
}

void Settings::setDewHeater(DewHeater *heater, DewHeaterSettings *settings, DewHeater *master) {
  heater->setFixedValue(settings->fixed);
  heater->setDewpointOffset(settings->offsetDewpoint);
  heater->setAmbientOffset(settings->offsetAmbient);
  heater->setMidpointOffset(settings->offsetMidpoint);
  heater->setOffset(settings->offset);
  switch (settings->mode) {
    case DewHeater::FIXED:
      heater->setFixed(settings->fixed);
      break;
    case DewHeater::DEWPOINT:
      heater->setDewpoint(settings->offsetDewpoint);
      break;
    case DewHeater::AMBIENT:
      heater->setAmbient(settings->offsetAmbient);
      break;
    case DewHeater::MIDPOINT:
      heater->setMidpoint(settings->offsetMidpoint);
      break;
    case DewHeater::SLAVE:
      if ( master != nullptr ) {
	heater->setSlave(master);
      }
      break;
  }
}

void Settings::setDewHeaters() {
  setDewHeater1();
  setDewHeater2();
}

void Settings::setDewHeater1() {
  DewHeater *dh1 = &Devices::i().dewHeater1;
  DewHeater *dh2 = &Devices::i().dewHeater2;
  if ( DH1.mode == DewHeater::SLAVE && DH2.mode == DewHeater::SLAVE ) {
    // Both can't be slaves!
    dh1->setFixed((float) 0.0);
  }
  setDewHeater(dh1, &DH1, dh2);
}

void Settings::setDewHeater2() {
  DewHeater *dh1 = &Devices::i().dewHeater1;
  DewHeater *dh2 = &Devices::i().dewHeater2;
  if ( DH1.mode == DewHeater::SLAVE && DH2.mode == DewHeater::SLAVE ) {
    // Both can't be slaves!
    dh2->setFixed((float) 0.0);
  }
  setDewHeater(dh2, &DH2, dh1);
}

void Settings::setEnvironmentSensor() {
  Devices::i().environmentSensor.setOffset(m_environmentSensorOffset);
}

void Settings::setDevices() {
  setRail12V();
  setAdj();
  setDewHeaters();
  setEnvironmentSensor();
}

void Settings::sendMessage(char *msg) {
  uint16_t crc = crcCalc(msg);
  char crcMsg[3];
  crcMsg[0] = ((char *) &crc)[0];
  crcMsg[1] = ((char *) &crc)[1];
  crcMsg[2] = '\0';
  Serial.print(MSG_PREFIX);
  Serial.print(msg);
  Serial.print('\0');
  Serial.print(crcMsg);
  Serial.print(MSG_POSTFIX);
}

void Settings::sendMessage(const __FlashStringHelper *msg) {
  uint16_t crc = crcCalc(msg, strlen_P(msg));
  char crcMsg[3];
  crcMsg[0] = ((char *) &crc)[0];
  crcMsg[1] = ((char *) &crc)[1];
  crcMsg[2] = '\0';
  Serial.print(MSG_PREFIX);
  Serial.print(msg);
  Serial.print('\0');
  Serial.print(crcMsg);
  Serial.print(MSG_POSTFIX);
}

void Settings::sendErrorMessage(char *msg) {
  char buff[128];
  DynamicJsonDocument json(128);
  json[F("Error")] = msg;
  serializeJson(json, buff, 128);
  sendMessage(buff);
}

void Settings::sendErrorMessage(const __FlashStringHelper *msg) {
  char buff[128];
  DynamicJsonDocument json(128);
  json[F("Error")] = msg;
  serializeJson(json, buff, 128);
  sendMessage(buff);
}

void Settings::sendStatus() {
  char buff[BUFFSIZE];
  Devices::i().state(buff, BUFFSIZE);
  sendMessage(buff);
}

bool Settings::runStatus(const char *cmd) {
  if ( strcmp_P(cmd, F("status") ) ) {
    return false;
  }
  sendStatus();
  return true;
}

bool Settings::runRail(const char *cmd) {
  bool found = false;
  if ( strcmp_P(cmd, F("rail on")) == 0 ) {
    m_RAIL12V_on = true;
    found = true;
  }
  if ( strcmp_P(cmd, F("rail off")) == 0 ) {
    m_RAIL12V_on = false;
    found = true;
  }

  if ( found ) {
    setRail12V();
    saveAndAck();
    return true;
  }
  return false;
}

bool Settings::runAdj(const char *cmd) {
  bool found = false;
  int voltage;
  if ( strcmp_P(cmd, F("adj on")) == 0 ) {
    m_ADJ_on = true;
    found = true;
  }
  if ( strcmp_P(cmd, F("adj off")) == 0 ) {
    m_ADJ_on = false;
    found = true;
  }
  if ( sscanf_P(cmd, PSTR("adj %d"), &voltage) == 1 ) {
    m_ADJ_voltage = static_cast<float>(voltage) / 100.;
    found = true;
  }

  if ( found ) {
    setAdj();
    saveAndAck();
    return true;
  }
  return false;
}
bool Settings::runEnv(const char *cmd) {
  bool found = false;
  int offset;
  if ( sscanf(cmd, "env offset %d", &offset) == 1 ) {
    m_environmentSensorOffset = static_cast<float>(offset) / 100.;
    found = true;
  }

  if ( found ) {
    setEnvironmentSensor();
    saveAndAck();
    return true;
  }
  return false;
}

bool Settings::runDH(const char *cmd) {
  int dh;
  DewHeaterSettings *settings;
  bool found = false;
  int tmpInt;
  unsigned char tmpChar;
  if ( sscanf_P(cmd, PSTR("DH%d"), &dh) != 1 ) {
    return false;
  }
  if ( dh == 1 ) {
    settings = &DH1;
  } else if ( dh == 2 ) {
    settings = &DH2;
  } else {
    return false;
  }
  Serial.println("DH: " + String(dh));

  if ( sscanf_P(cmd, PSTR("DH%*d mode %d"), &tmpChar) == 1 ) {
    if ( tmpChar == DewHeater::SLAVE ) {
      DewHeater::Mode other;
      if ( dh == 1 ) {
	other = Devices::i().dewHeater2.currentMode();
      }
      if ( dh == 2 ) {
	other = Devices::i().dewHeater1.currentMode();
      }
      if ( other == DewHeater::SLAVE ) {
	sendErrorMessage(F("Both dew heaters can't be slaves"));
	return false;
      }
    }
    settings->mode = DewHeater::Mode(tmpChar);
    found = true;
  }

  if ( sscanf_P(cmd, PSTR("DH%*d offset %d"), &tmpInt) == 1 ) {
    settings->offset = static_cast<float>(tmpInt) / 100.;
    found = true;
  }

  if ( sscanf_P(cmd, PSTR("DH%*d fixed %d"), &tmpInt) == 1 ) {
    settings->fixed = static_cast<float>(tmpInt) / 100.;
    found = true;
  }

  if ( sscanf_P(cmd, PSTR("DH%*d oD %d"), &tmpInt) == 1 ) {
    settings->offsetDewpoint = static_cast<float>(tmpInt) / 100.;
    found = true;
  }

  if ( sscanf_P(cmd, PSTR("DH%*d oA %d"), &tmpInt) == 1 ) {
    settings->offsetAmbient = static_cast<float>(tmpInt) / 100.;
    found = true;
  }

  if ( sscanf(cmd, PSTR("DH%*d oM %d"), &tmpInt) == 1 ) {
    settings->offsetMidpoint = static_cast<float>(tmpInt) / 100.;
    found = true;
  }

  if ( found ) {
    if ( dh == 1 ) {
      setDewHeater1();
    }
    if ( dh == 2 ) {
      setDewHeater2();
    }
    saveAndAck();
    return true;
  }
  return false;
}

void Settings::runUnknownCommand() {
  sendErrorMessage(F("Unknown command"));
}

void Settings::saveAndAck() {
  saveToEEPROM();
  sendStatus();
}

bool Settings::runCommand(const char *cmd) {
  if ( runStatus(cmd) ) {
    return true;
  }
  if ( runRail(cmd) ) {
      return true;
  }
  if ( runAdj(cmd) ) {
    return true;
  }
  if ( runEnv(cmd) ) {
    return true;
  }
  if ( runDH(cmd) ) {
    return true;
  }
  runUnknownCommand();
  return false;
}

void Settings::setup() {
  Serial.begin(115200);
  Serial.setTimeout(1000);
}

void Settings::loop() {
  static bool inCommand = false;
  Devices::i().update();
  while ( Serial.available() ) {
    char c = Serial.read();
    switch (c) {
      case MSG_PREFIX:
	inCommand = true;
	CommandBuffer::i().clear();
	CommandBuffer::i().add(c);
	break;
      case MSG_POSTFIX:
	if ( inCommand ) {
	  inCommand = false;
	  if ( ! CommandBuffer::i().add(c) ) {
	    // Overflow
	    CommandBuffer::i().clear();
	    break;
	  }
	  if ( CommandBuffer::i().verifyChecksum() ) {
	    runCommand(CommandBuffer::i().getCommand());
	  } else {
	    sendErrorMessage(F("Checksum error"));
	  }
	  CommandBuffer::i().clear();
	}
	break;
      default:
	if ( inCommand ) {
	  if ( ! CommandBuffer::i().add(c) ) {
	    // Overflow
	    CommandBuffer::i().clear();
	    inCommand = false;
	  }
	}
    }
  }
}
