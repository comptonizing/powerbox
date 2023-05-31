#include "Devices.h"

EnvironmentSensor::EnvironmentSensor(unsigned char address) : m_address(address) {
  connect();
}

bool EnvironmentSensor::isReachable() {
  Wire.beginTransmission(m_address);
  return Wire.endTransmission() == 0 ? true : false;
}

void EnvironmentSensor::connect() {
  if ( ! device.begin(m_address) ) {
    return;
  }
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
    m_lastTemperature = temp;
    return;
  }
  connect();
  temp = device.readTemperature();
  if ( validTemperature(temp) ) {
    m_lastTemperature = temp;
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
  pressure = device.readPressure();
  if ( validPressure(pressure) ) {
    m_lastPressure = pressure;
    return;
  }
  connect();
  pressure = device.readPressure();
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
  humidity = device.readPressure();
  if ( validHumidty(humidity) ) {
    m_lastHumidity = humidity;
    return;
  }
  m_lastHumidity = NAN;
}

void EnvironmentSensor::update() {
  if ( (millis() - m_lastUpdate) >= m_updateInterval ) {
    updateTemperature();
    updatePressure();
    updateHumidity();
    updateDewpoint();
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

void EnvironmentSensor::updateDewpoint() {
  // According to http://irtfweb.ifa.hawaii.edu/~tcs3/tcs3/Misc/Dewpoint_Calculation_Humidity_Sensor_E.pdf
  float RH = currentHumidity();
  float T = currentTemperature();
  if ( isnan(RH) || isnan(T) ) {
    m_lastDewpoint = NAN;
    return;
  }
  float H = (log10(RH)-2)/0.4343 + (17.62*T)/(243.12+T);
  float m_lastDewpoint = 243.12*H/(17.62-H);
}
