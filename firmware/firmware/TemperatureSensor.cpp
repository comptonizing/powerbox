#include "Devices.h"

TemperatureSensor::TemperatureSensor(unsigned char pin, float offset) :
  m_pin(pin), m_offset(offset), bus(pin), sensor(&bus) {
    sensor.begin();
    sensor.setResolution(12);
    sensor.setWaitForConversion(false);
    update(true);
}

void TemperatureSensor::update(bool block) {
  if ( block ) {
    while ( ! sensor.isConversionComplete() ) {
      ;
    }
  } else {
    if ( ! sensor.isConversionComplete() ) {
      return;
    }
  }
  m_lastTemperature = sensor.getTempCByIndex(0) + m_offset;
  if ( m_lastTemperature == DEVICE_DISCONNECTED_C ) {
    m_lastTemperature = NAN;
  }
  sensor.requestTemperatures();
}

float TemperatureSensor::currentTemperature() {
  update();
  return m_lastTemperature;
}

void TemperatureSensor::setOffset(float offset) {
  m_offset = offset;
}

float TemperatureSensor::getOffset() {
  return m_offset;
}
