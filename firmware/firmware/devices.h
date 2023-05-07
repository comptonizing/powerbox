#pragma once

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

#include "EnvironmentSensor.h"
