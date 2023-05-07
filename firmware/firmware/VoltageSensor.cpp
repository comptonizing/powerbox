#include "VoltageSensor.h"

VoltageSensor::VoltageSensor(uint8_t pin) : m_pin(pin) {
  setAref(EXTERNAL);
}

void VoltageSensor::setAref(uint8_t mode) {
  switch (mode) {
    case DEFAULT:
      m_vref = 5.0;
      break;
    case INTERNAL:
      m_vref = 1.1;
      break;
    case EXTERNAL:
      // Assuming a 4.7k resistor in series, and the 32k resistor as pull down
      m_vref = 4.9;
      // m_vref = 5.0 * 32. / (32. + 4.7);
      break;
  }
}

float VoltageSensor::voltage() {
  uint16_t raw = analogRead(m_pin);
  // Serial.println("Raw: " + String(raw));
  float measured = m_vref * raw / 1024.;
  // Serial.println("Measured: " + String(measured));
  return (100.+33.)/33. * measured;
}
