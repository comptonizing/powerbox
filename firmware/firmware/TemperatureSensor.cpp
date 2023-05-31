#include "Devices.h"

TemperatureSensor::TemperatureSensor(unsigned char pin) :
  m_pin(pin), bus(pin), sensor(&bus) {
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
  m_lastTemperature = sensor.getTempCByIndex(0);
  if ( m_lastTemperature == DEVICE_DISCONNECTED_C ) {
    m_lastTemperature = NAN;
  }
  sensor.requestTemperatures();
}

float TemperatureSensor::currentTemperature() {
  update();
  return m_lastTemperature;
}
