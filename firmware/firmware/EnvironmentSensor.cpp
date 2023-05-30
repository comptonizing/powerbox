#include "EnvironmentSensor.h"

EnvironmentSensor::EnvironmentSensor(unsigned char address) : m_address(address) {
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

float EnvironmentSensor::currentTemperature() {
  float temp;
  if ( ! isReachable() ) {
    return NAN;
  }
  temp = device.readTemperature();
  if ( validTemperature(temp) ) {
    return temp;
  }
  connect();
  temp = device.readTemperature();
  if ( validTemperature(temp) ) {
    return temp;
  }
  return NAN;
}

float EnvironmentSensor::currentPressure() {
  float pressure;
  if ( ! isReachable() ) {
    return NAN;
  }
  pressure = device.readPressure();
  if ( validPressure(pressure) ) {
    return pressure;
  }
  connect();
  pressure = device.readPressure();
  if ( validPressure(pressure) ) {
    return pressure;
  }
  return NAN;
}

float EnvironmentSensor::currentHumidity() {
  float humidity;
  if ( ! isReachable() ) {
    return NAN;
  }
  humidity = device.readHumidity();
  if ( validHumidty(humidity) ) {
    return humidity;
  }
  connect();
  humidity = device.readPressure();
  if ( validHumidty(humidity) ) {
    return humidity;
  }
  return NAN;
}
