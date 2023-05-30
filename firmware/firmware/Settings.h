#pragma once

#include <HardwareSerial.h>
#include <string.h>
#include <EEPROM.h>
#include <util/crc16.h>

class Settings {
  public:
    static Settings &i();
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
    float m_ADJ_voltage = 5.0;
};
