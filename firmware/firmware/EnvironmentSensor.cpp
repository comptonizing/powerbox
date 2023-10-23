#include "Devices.h"

EnvironmentSensor::EnvironmentSensor(unsigned char address, float offset) :
  m_address(address), m_offset(offset) {
  connect();
  update(true);
}

bool EnvironmentSensor::isReachable() {
  Wire.beginTransmission(m_address);
  return Wire.endTransmission() == 0 ? true : false;
}

void EnvironmentSensor::connect() {
  device.begin(m_address);
}

bool EnvironmentSensor::validTemperature(float temp) {
  return ! ( isnan(temp) || temp > 150. || temp < - 150. );
}

bool EnvironmentSensor::validPressure(float pressure) {
  return ! ( isnan(pressure) || pressure > 200000. || pressure < -10000. );
}

bool EnvironmentSensor::validHumidty(float humidity) {
  return ! ( isnan(humidity) || humidity > 99. || humidity < 1. );
}

void EnvironmentSensor::updateTemperature() {
  float temp;
  if ( ! isReachable() ) {
    m_lastTemperature = NAN;
    return;
  }
  temp = device.readTemperature();
  if ( validTemperature(temp) ) {
    m_lastTemperature = temp + m_offset;
    return;
  }
  connect();
  temp = device.readTemperature();
  if ( validTemperature(temp) ) {
    m_lastTemperature = temp + m_offset;
    return;
  }
  m_lastTemperature = NAN;
}

void EnvironmentSensor::updatePressure() {
  float pressure;
  if ( ! isReachable() ) {
    m_lastPressure = NAN;
    return;
  }
  pressure = device.readPressure() / 100.;
  if ( validPressure(pressure) ) {
    m_lastPressure = pressure;
    return;
  }
  connect();
  pressure = device.readPressure() / 100.;
  if ( validPressure(pressure) ) {
    m_lastPressure = pressure;
    return;
  }
  m_lastPressure = NAN;
}

void EnvironmentSensor::updateHumidity() {
  float humidity;
  if ( ! isReachable() ) {
    m_lastHumidity = NAN;
    return;
  }
  humidity = device.readHumidity();
  if ( validHumidty(humidity) ) {
    m_lastHumidity = humidity;
    return;
  }
  connect();
  humidity = device.readHumidity();
  if ( validHumidty(humidity) ) {
    m_lastHumidity = humidity;
    return;
  }
  m_lastHumidity = NAN;
}

void EnvironmentSensor::update(bool force) {
  if ( (unsigned long)(millis() - m_lastUpdate) >= m_updateInterval || force ) {
    updateTemperature();
    updatePressure();
    updateHumidity();
    m_lastUpdate = millis();
  }
}

float EnvironmentSensor::currentTemperature() {
  update();
  return m_lastTemperature;
}

float EnvironmentSensor::currentPressure() {
  update();
  return m_lastPressure;
}

float EnvironmentSensor::currentHumidity() {
  update();
  return m_lastHumidity;
}

float EnvironmentSensor::currentDewpoint() {
  // According to http://irtfweb.ifa.hawaii.edu/~tcs3/tcs3/Misc/Dewpoint_Calculation_Humidity_Sensor_E.pdf
  float RH = currentHumidity();
  float T = currentTemperature();
  if ( isnan(RH) || isnan(T) ) {
    m_lastDewpoint = NAN;
    return NAN;
  }
  float H = (log10(RH)-2)/0.4343 + (17.62*T)/(243.12+T);
  return 243.12*H/(17.62-H);
}

float EnvironmentSensor::getOffset() {
  return m_offset;
}

void EnvironmentSensor::setOffset(float offset) {
  m_offset = offset;
}
