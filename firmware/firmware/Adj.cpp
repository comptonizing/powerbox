#include "Devices.h"

Adj::Adj(uint8_t address, unsigned char pin) : m_address(address), m_pin(pin) {
  pinMode(pin, OUTPUT);
  dac.begin(m_address);
  off();
}

void Adj::off() {
  digitalWrite(m_pin, LOW);
  m_isOn = false;
}

void Adj::on() {
  digitalWrite(m_pin, HIGH);
  m_isOn = true;
}

bool Adj::isOn() {
  return m_isOn;
}

unsigned int Adj::output2dac(float voltage) {
  return (unsigned int) ((voltage - 12.1961878023845) / -0.00292561438695341);
}

void Adj::setVoltage(float voltage) {
  float out;
  unsigned int adu;
  if ( voltage < 4.5 ) {
    voltage = 0.0;
  } else if ( voltage > 12.0 ) {
    voltage = 12.0;
  }
  m_targetVoltage = voltage;
  if ( voltage == 0.0 ) {
    off();
    return;
  }
  adu = output2dac(voltage);
  dac.setVoltage(adu, false, 100000);
}

float Adj::voltage() {
  return m_targetVoltage;
}
