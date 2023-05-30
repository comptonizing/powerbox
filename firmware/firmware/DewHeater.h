#include <Arduino.h>

class DewHeater {
  public:
    DewHeater(unsigned char pin);
    void setDutyCycle(unsigned char dc);
    void setDutyCyclePercent(float dc);
    float currentDutyCyclePercent();
    unsigned char currentDutyCycle();
  private:
    unsigned char m_pin = -1;
    unsigned char m_currentDutyCycle = 0;
};
