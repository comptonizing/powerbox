#include "DewHeater.h"

DewHeater::DewHeater(unsigned char pin) : m_pin(pin) {
  pinMode(m_pin, OUTPUT);
}

void DewHeater::setDutyCycle(unsigned char dc) {
  m_currentDutyCycle = dc;
  analogWrite(m_pin, m_currentDutyCycle);
}

void DewHeater::setDutyCyclePercent(float dc) {
  if ( dc < 0.0 ) {
    dc = 0.0;
  }
  if ( dc > 100. ) {
    dc = 100.;
  }
  unsigned char dcChar =  255. / 100. * dc;
  setDutyCycle(dcChar);
}

unsigned char DewHeater::currentDutyCycle() {
  return m_currentDutyCycle;
}

float DewHeater::currentDutyCyclePercent() {
  return currentDutyCycle() * 100. / 255;
}
