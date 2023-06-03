#include "Devices.h"

Devices &Devices::i() {
  static Devices currentInstance;
  return currentInstance;
}

Devices::Devices() {
}

Devices::~Devices() {
}

void Devices::update() {
  environmentSensor.update();
  dewHeater1.update();
  dewHeater2.update();
}

void Devices::jsonAddDH(const char *prefix, DewHeater *dh, DynamicJsonDocument &json) {
  json[prefix]["M"] = dh->currentMode();
  json[prefix]["DC"] = dh->currentDutyCyclePercent();
  json[prefix]["T"] = dh->currentTemperature();
  json[prefix]["dT"] = dh->getOffset();
  json[prefix]["OD"] = dh->dewpointOffset();
  json[prefix]["OA"] = dh->ambientOffset();
  json[prefix]["OM"] = dh->midpointOffset();
  json[prefix]["F"] = dh->fixedValue();
}

void Devices::state(char *buff, size_t buffSize) {
  DynamicJsonDocument json(256);
  json["V"] = voltageSensor.voltage();
  json["E"]["T"] = environmentSensor.currentTemperature();
  json["E"]["dT"] = environmentSensor.getOffset();
  json["E"]["P"] = environmentSensor.currentPressure();
  json["E"]["H"] = environmentSensor.currentHumidity();
  json["E"]["D"] = environmentSensor.currentDewpoint();
  json["R"] = rail12V.isOn();
  json["A"]["ON"] = adj.isOn();
  json["A"]["V"] = adj.voltage();
  jsonAddDH("D1", &Devices::i().dewHeater1, json);
  jsonAddDH("D2", &Devices::i().dewHeater2, json);
  serializeJson(json, buff, buffSize);
}
