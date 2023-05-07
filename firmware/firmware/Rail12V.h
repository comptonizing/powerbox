#pragma once

#include <inttypes.h>
#include <Arduino.h>

class Rail12V {
  public:
    Rail12V(uint8_t pin);
    void on();
    void off();
    uint8_t state();
  private:
    uint8_t m_pin = -1;
    uint8_t m_state = LOW;
};
