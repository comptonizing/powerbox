#pragma once

#include <stdint.h>
#include <math.h>
#include <Wire.h>

#include <Adafruit_BME280.h>

class EnvironmentSensor {
  public:
    EnvironmentSensor(unsigned char address = 0x77);
    float currentTemperature();
    float currentPressure();
    float currentHumidity();
    void connect();
  private:
    bool isReachable();
    unsigned char m_address;
    Adafruit_BME280 device;
    bool validTemperature(float temp);
    bool validPressure(float pressure);
    bool validHumidty(float humidity);
};
