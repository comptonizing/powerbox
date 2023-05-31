#include "Devices.h"

DewHeater::DewHeater(unsigned char pin, unsigned char temperaturePin) :
  m_pin(pin), m_temperaturePin(temperaturePin), m_sensor(m_temperaturePin),
  m_pid(&m_lastTemperature, &m_pidDutyCycle, &m_targetTemperature, 4., 1.0, 1.0, DIRECT) {
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
  update();
}

void DewHeater::setFixed(float dutyCycle) {
  setFixed(p2c(dutyCycle));
  update();
}

void DewHeater::setDewpoint(float offset) {
  m_offsetDewpoint = offset;
  m_pid.SetMode(AUTOMATIC);
  m_mode = DEWPOINT;
  update();
}

void DewHeater::setAmbient(float offset) {
  m_offsetAmbient = offset;
  m_pid.SetMode(AUTOMATIC);
  m_mode = AMBIENT;
  update();
}

void DewHeater::setMidpoint(float offset) {
  m_offsetMidpoint = offset;
  m_pid.SetMode(AUTOMATIC);
  m_mode = MIDPOINT;
  update();
}

void DewHeater::setSlave(unsigned char (*callback)()) {
  m_slaveCallback = callback;
  m_mode = SLAVE;
  update();
}

void DewHeater::updateWithPID(float targetTemperature) {
  m_targetTemperature = targetTemperature;
  m_lastTemperature = m_sensor.currentTemperature();
  m_pid.Compute();
  setDutyCycle(static_cast<unsigned char>(m_pidDutyCycle));
}

void DewHeater::updateDewpoint() {
  float dewpoint = Devices::i().environmentSensor.currentDewpoint();
  float heaterTemperature = m_sensor.currentTemperature();
  if ( isnan(dewpoint) || isnan(heaterTemperature) ) {
    // Something's wrong with the sensor(s), let's go to full power
    setDutyCycle(255);
    return;
  }
  updateWithPID(dewpoint + m_offsetDewpoint);
}

void DewHeater::updateAmbient() {
  float ambient = Devices::i().environmentSensor.currentTemperature();
  float heaterTemperature = m_sensor.currentTemperature();
  if ( isnan(ambient) || isnan(heaterTemperature) ) {
    // Something's wrong with the sensor(s), let's go to full power
    setDutyCycle(255);
    return;
  }
  updateWithPID(ambient - m_offsetAmbient);
}

void DewHeater::updateMidpoint() {
  float dewpoint = Devices::i().environmentSensor.currentDewpoint();
  float ambient = Devices::i().environmentSensor.currentTemperature();
  float heaterTemperature = m_sensor.currentTemperature();
  if ( isnan(dewpoint) || isnan(ambient) || isnan(heaterTemperature) ) {
    // Something's wrong with the sensor(s), let's go to full power
    setDutyCycle(255);
    return;
  }
  updateWithPID( 0.5 * (ambient + dewpoint) + m_offsetMidpoint);
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

float DewHeater::currentTemperature() {
  return m_sensor.currentTemperature();
}

float DewHeater::currentTargetTemperature() {
  return m_targetTemperature;
}
