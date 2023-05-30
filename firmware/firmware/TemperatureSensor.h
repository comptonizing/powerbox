#include <OneWire.h>
#include <DallasTemperature.h>

class TemperatureSensor {
  public:
    TemperatureSensor(unsigned char pin);
    void update(bool block = false);
    float currentTemperature();
  private:
    unsigned char m_pin;
    unsigned char m_address;
    OneWire bus;
    DallasTemperature sensor;
    float m_lastTemperature = 20.;
};
