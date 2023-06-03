#pragma once

#include <HardwareSerial.h>
#include <string.h>
#include <EEPROM.h>
#include <util/crc16.h>

#include "Devices.h"

typedef struct {
  DewHeater::Mode mode = DewHeater::DEWPOINT;
  float offset = 0.0;
  float fixed = 0.0;
  float offsetDewpoint = 2.0;
  float offsetAmbient = 5.0;
  float offsetMidpoint = 5.0;
  bool slave = false;
} DewHeaterSettings;

class Settings {
  public:
    static Settings &i();
    void setDevices();
    void setRail12V();
    void setAdj();
    void setDewHeaters();
    void setDewHeater1();
    void setDewHeater2();
    void setEnvironmentSensor();
  private:
    Settings();
    ~Settings();
    Settings(const Settings&);
    Settings& operator=(const Settings&);
    void readEEPROM(uint16_t address, uint8_t *buff, uint16_t n);
    void writeEEPROM(uint16_t address, uint8_t *buff, uint16_t n);
    bool loadFromEEPROM();
    void saveToEEPROM();
    uint16_t crcCalc(uint8_t *data, uint16_t n);

    bool m_RAIL12V_on = false;

    bool m_ADJ_on = false;
    float m_ADJ_voltage = 0.0;

    float m_environmentSensorOffset = 0.0;

    DewHeaterSettings DH1;
    DewHeaterSettings DH2;
    void setDewHeater(DewHeater *heater, DewHeaterSettings *setting, DewHeater *master = nullptr);
};
