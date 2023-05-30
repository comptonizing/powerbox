#pragma once

#include <inttypes.h>
#include <Arduino.h>
#include <Adafruit_MCP4725.h>

class Adj {
  public:
    Adj(uint8_t address, unsigned char pin);
    void on();
    void off();
    bool isOn();
    void setVoltage(float voltage);
    float voltage();
  private:
    unsigned int output2dac(float voltage);
    uint8_t m_address = -1;
    unsigned char m_pin = -1;
    bool m_isOn = false;
    float m_targetVoltage = 0.0;
    Adafruit_MCP4725 dac;
};
