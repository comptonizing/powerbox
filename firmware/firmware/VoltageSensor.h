#pragma once

#include <inttypes.h>
#include <Arduino.h>

class VoltageSensor {
  public:
    VoltageSensor(uint8_t pin);
    void setAref(uint8_t mode);
    float voltage();
  private:
    uint8_t m_pin = -1;
    float m_vref = 5.0;
};
