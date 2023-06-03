#include "Settings.h"

#define OFFSET_MAGIC 0
static uint16_t __magic = 0b1110110101010010;

#define OFFSET_CRC (OFFSET_MAGIC + sizeof(__magic))

#define OFFSET_DATA (OFFSET_CRC + sizeof(uint16_t))

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
    crc = _crc16_update(crc, data[ii]);
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
  if ( loadFromEEPROM() ) {
    return;
  }
  saveToEEPROM();
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
  if ( DH1.slave && DH2.slave ) {
    // Both can't be slaves!
    dh1->setFixed((float) 0.0);
  }
  setDewHeater(dh1, &DH1, dh2);
}

void Settings::setDewHeater2() {
  DewHeater *dh1 = &Devices::i().dewHeater1;
  DewHeater *dh2 = &Devices::i().dewHeater2;
  if ( DH1.slave && DH2.slave ) {
    // Both can't be slaves!
    dh2->setFixed((float) 0.0);
  }
  setDewHeater(dh2, &DH1, dh1);
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
