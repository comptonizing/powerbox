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
    EnvironmentSensor(unsigned char address = 0x76);
    float currentTemperature();
    float currentPressure();
    float currentHumidity();
    float currentDewpoint();
    void connect();
    void update();
  private:
    unsigned long m_updateInterval = 1000; // ms
    float m_lastTemperature = NAN;
    float m_lastPressure = NAN;
    float m_lastHumidity = NAN;
    float m_lastDewpoint = NAN;
    unsigned long m_lastUpdate = 0;
    void updateTemperature();
    void updatePressure();
    void updateHumidity();
    void updateDewpoint();
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
  private:
    uint8_t m_pin = -1;
    uint8_t m_state = LOW;
};

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

    typedef enum {
      FIXED,
      DEWPOINT,
      AMBIENT,
      MIDPOINT,
      SLAVE
    } Mode;

    Mode currentMode();
    void setFixed(float dutyCycle);
    void setFixed(unsigned char dutyCycle);
    void setDewpoint(float offset);
    void setAmbient(float offset);
    void setMidpoint(float offset);
    void setSlave(unsigned char (*callback)());

  private:
    unsigned char m_pin = -1;
    unsigned char m_temperaturePin = -1;
    TemperatureSensor m_sensor;
    unsigned char m_currentDutyCycle = 0;
    Mode m_mode;

    float m_offsetDewpoint;
    float m_offsetAmbient;
    float m_offsetMidpoint;
    unsigned char (*m_slaveCallback)() = nullptr;

    unsigned char p2c(float percent);
    float c2p(unsigned char dc);

    void setDutyCycle(unsigned char dc);
    void setDutyCyclePercent(float dc);

    void updateDewpoint();
    void updateAmbient();
    void updateMidpoint();
    void updateSlave();

    float m_targetTemperature = 0.0;
    float m_actualTemperature = 0.0;
};

class Devices {
  public:
    static Devices &instance();
    VoltageSensor voltageSensor = VoltageSensor(EXT_VOLTAGE);
    EnvironmentSensor environmentSensor = EnvironmentSensor(BME280ADDR);
    Rail12V rail12V = Rail12V(RAIL12V);
    Adj adj = Adj(DACADDR, ADJ);
    DewHeater dewHeater1 = DewHeater(DH1PWM, TEMP1DATA);
    DewHeater dewHeater2 = DewHeater(DH2PWM, TEMP2DATA);
  private:
    ~Devices();
    Devices();
    Devices(const Devices&);
    Devices & operator=(const Devices&);
};
