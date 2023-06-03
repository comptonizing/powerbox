#include "Devices.h"

Rail12V::Rail12V(uint8_t pin) : m_pin(pin) {
  off();
}

void Rail12V::on() {
  digitalWrite(m_pin, HIGH);
  m_state = HIGH;
}

void Rail12V::off() {
  digitalWrite(m_pin, LOW);
  m_state = LOW;
}

uint8_t Rail12V::state() {
  return m_state;
}

bool Rail12V::isOn() {
  return m_state == HIGH ? true : false;
}
