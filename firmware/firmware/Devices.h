#pragma once

#include <stdint.h>
#include <math.h>
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_MCP4725.h>
#include <PID_v1.h>
#include <ArduinoJson.h>

// Temperature probe 1
#define TEMP1DATA 2
// Temperature probe 2
#define TEMP2DATA 3

// 4x12V Output
#define RAIL12V 6

// Buck converter chip enable
#define ADJ 8

// Dew heater 1 PWM
#define DH1PWM 9
// Dew heater 2 PWM
#define DH2PWM 10

// MCP4725 I2C address
#define DACADDR 0x60

// BME280 I2C address
#define BME280ADDR 0x76

// External Voltage
#define EXT_VOLTAGE 0


class EnvironmentSensor {
  public:
    EnvironmentSensor(unsigned char address = 0x76, float offset = 0.0);
    float currentTemperature();
    float currentPressure();
    float currentHumidity();
    float currentDewpoint();
    void connect();
    void update(bool force = false);
    float getOffset();
    void setOffset(float offset);
  private:
    unsigned long m_updateInterval = 1000; // ms
    float m_lastTemperature = NAN;
    float m_lastPressure = NAN;
    float m_lastHumidity = NAN;
    float m_lastDewpoint = NAN;
    float m_offset = 0.0;
    unsigned long m_lastUpdate = 0;
    void updateTemperature();
    void updatePressure();
    void updateHumidity();
    bool isReachable();
    unsigned char m_address;
    Adafruit_BME280 device;
    bool validTemperature(float temp);
    bool validPressure(float pressure);
    bool validHumidty(float humidity);
};

class VoltageSensor {
  public:
    VoltageSensor(uint8_t pin);
    void setAref(uint8_t mode);
    float voltage();
  private:
    uint8_t m_pin = -1;
    float m_vref = 5.0;
};

class Rail12V {
  public:
    Rail12V(uint8_t pin);
    void on();
    void off();
    uint8_t state();
    bool isOn();
  private:
    uint8_t m_pin = -1;
    uint8_t m_state = LOW;
};

class TemperatureSensor {
  public:
    TemperatureSensor(unsigned char pin, float offset = 0.0);
    void update(bool block = false);
    float currentTemperature();
    void setOffset(float offset);
    float getOffset();
  private:
    unsigned char m_pin;
    unsigned char m_address;
    OneWire bus;
    DallasTemperature sensor;
    float m_lastTemperature = 20.;
    float m_offset = 0.0;
};

class Adj {
  public:
    Adj(uint8_t address, unsigned char pin);
    void on();
    void off();
    bool isOn();
    void setVoltage(float voltage);
    float voltage();
  private:
    unsigned int output2dac(float voltage);
    uint8_t m_address = -1;
    unsigned char m_pin = -1;
    bool m_isOn = false;
    float m_targetVoltage = 0.0;
    Adafruit_MCP4725 dac;
};

class DewHeater {
  public:
    DewHeater(unsigned char pin, unsigned char temperaturePin);
    void update();
    float currentDutyCyclePercent();
    unsigned char currentDutyCycle();
    float currentTemperature();
    float currentTargetTemperature();
    float getOffset();
    void setOffset(float offset);

    typedef enum {
      FIXED = 0,
      DEWPOINT = 1,
      AMBIENT = 2,
      MIDPOINT = 3,
      SLAVE = 4
    } Mode;

    Mode currentMode();
    void setFixed(float dutyCycle);
    void setFixed(unsigned char dutyCycle);
    void setDewpoint(float offset);
    void setAmbient(float offset);
    void setMidpoint(float offset);
    void setSlave(DewHeater *other);

    void setFixedValue(float value);
    void setDewpointOffset(float offset);
    void setAmbientOffset(float offset);
    void setMidpointOffset(float offset);

    float fixedValue();
    float dewpointOffset();
    float ambientOffset();
    float midpointOffset();

  private:
    unsigned char m_pin = -1;
    unsigned char m_temperaturePin = -1;
    TemperatureSensor m_sensor;
    unsigned char m_currentDutyCycle = 0;
    Mode m_mode;

    unsigned char m_fixedValue = 0;
    float m_offsetDewpoint = 2.0;
    float m_offsetAmbient = 5.0;
    float m_offsetMidpoint = 5.0;
    DewHeater *m_master = nullptr;

    unsigned char p2c(float percent);
    float c2p(unsigned char dc);

    void setDutyCycle(unsigned char dc);
    void setDutyCyclePercent(float dc);

    void updateDewpoint();
    void updateAmbient();
    void updateMidpoint();
    void updateSlave();

    double m_targetTemperature = 0.0;
    double m_actualTemperature = 0.0;
    double m_lastTemperature = 0.0;
    double m_pidDutyCycle = m_currentDutyCycle;
    PID m_pid;
    void updateWithPID(float targetTemperature);
};

class Devices {
  public:
    static Devices &i();
    void update();
    VoltageSensor voltageSensor = VoltageSensor(EXT_VOLTAGE);
    EnvironmentSensor environmentSensor = EnvironmentSensor(BME280ADDR);
    Rail12V rail12V = Rail12V(RAIL12V);
    Adj adj = Adj(DACADDR, ADJ);
    DewHeater dewHeater1 = DewHeater(DH1PWM, TEMP1DATA);
    DewHeater dewHeater2 = DewHeater(DH2PWM, TEMP2DATA);
    void state(char *buff, size_t buffSize);
    void print();
  private:
    ~Devices();
    Devices();
    Devices(const Devices&);
    Devices & operator=(const Devices&);
};
