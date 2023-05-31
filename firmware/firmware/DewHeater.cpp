#include "Devices.h"

DewHeater::DewHeater(unsigned char pin, unsigned char temperaturePin) :
  m_pin(pin), m_temperaturePin(temperaturePin), m_sensor(m_temperaturePin) {
  pinMode(m_pin, OUTPUT);
  setDutyCycle(0);
}

void DewHeater::setDutyCycle(unsigned char dc) {
  m_currentDutyCycle = dc;
  analogWrite(m_pin, m_currentDutyCycle);
}

unsigned char DewHeater::p2c(float percent) {
  return 255. / 100. * percent;
}

float DewHeater::c2p(unsigned char dc) {
  return dc * 100. / 255;
}

void DewHeater::setDutyCyclePercent(float dc) {
  if ( dc < 0.0 ) {
    dc = 0.0;
  }
  if ( dc > 100. ) {
    dc = 100.;
  }
  unsigned char dcChar =  p2c(dc);
  setDutyCycle(dcChar);
}

unsigned char DewHeater::currentDutyCycle() {
  return m_currentDutyCycle;
}

float DewHeater::currentDutyCyclePercent() {
  return c2p(currentDutyCycle());
}

DewHeater::Mode DewHeater::currentMode() {
  return m_mode;
}

void DewHeater::setFixed(unsigned char dutyCycle) {
  m_mode = FIXED;
  m_currentDutyCycle = dutyCycle;
  setDutyCycle(dutyCycle);
}

void DewHeater::setFixed(float dutyCycle) {
  setFixed(p2c(dutyCycle));
}

void DewHeater::setDewpoint(float offset) {
  m_offsetDewpoint = offset;
  m_mode = DEWPOINT;
}

void DewHeater::setAmbient(float offset) {
  m_offsetAmbient = offset;
  m_mode = AMBIENT;
}

void DewHeater::setMidpoint(float offset) {
  m_offsetMidpoint = offset;
  m_mode = MIDPOINT;
}

void DewHeater::setSlave(unsigned char (*callback)()) {
  m_slaveCallback = callback;
  m_mode = SLAVE;
}

void DewHeater::updateDewpoint() {
}

void DewHeater::updateAmbient() {
}

void DewHeater::updateMidpoint() {
}

void DewHeater::updateSlave() {
  setDutyCycle(m_slaveCallback());
}

void DewHeater::update() {
  switch (m_mode) {
    case FIXED:
      break;
    case DEWPOINT:
      updateDewpoint();
      break;
    case AMBIENT:
      updateAmbient();
      break;
    case MIDPOINT:
      updateMidpoint();
      break;
    case SLAVE:
      updateSlave();
      break;
  }
}
