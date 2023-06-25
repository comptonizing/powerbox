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

#ifdef __arm__
// should use uinstd.h to define sbrk but Due causes a conflict
extern "C" char* sbrk(int incr);
#else  // __ARM__
extern char *__brkval;
#endif  // __arm__

int freeMemory() {
  char top;
#ifdef __arm__
  return &top - reinterpret_cast<char*>(sbrk(0));
#elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
  return &top - __brkval;
#else  // __arm__
  return __brkval ? &top - __brkval : &top - __malloc_heap_start;
#endif  // __arm__
}

void Devices::state(char *buff, size_t buffSize) {
  StaticJsonDocument<384> json;
  json[F("V")] = voltageSensor.voltage();
  json[F("E")][F("T")] = environmentSensor.currentTemperature();
  json[F("E")][F("dT")] = environmentSensor.getOffset();
  json[F("E")][F("P")] = environmentSensor.currentPressure();
  json[F("E")][F("H")] = environmentSensor.currentHumidity();
  json[F("E")][F("D")] = environmentSensor.currentDewpoint();
  json[F("R")] = rail12V.isOn();
  json[F("A")][F("ON")] = adj.isOn();
  json[F("A")][F("V")] = adj.voltage();
  json[F("M")] = freeMemory();

  auto dh = &Devices::i().dewHeater1;
  json[F("DH1")][F("M")] = dh->currentMode();
  json[F("DH1")][F("DC")] = dh->currentDutyCyclePercent();
  json[F("DH1")][F("T")] = dh->currentTemperature();
  json[F("DH1")][F("dT")] = dh->getOffset();
  json[F("DH1")][F("OD")] = dh->dewpointOffset();
  json[F("DH1")][F("OA")] = dh->ambientOffset();
  json[F("DH1")][F("OM")] = dh->midpointOffset();
  json[F("DH1")][F("F")] = dh->fixedValue();

  dh = &Devices::i().dewHeater2;
  json[F("DH2")][F("M")] = dh->currentMode();
  json[F("DH2")][F("DC")] = dh->currentDutyCyclePercent();
  json[F("DH2")][F("T")] = dh->currentTemperature();
  json[F("DH2")][F("dT")] = dh->getOffset();
  json[F("DH2")][F("OD")] = dh->dewpointOffset();
  json[F("DH2")][F("OA")] = dh->ambientOffset();
  json[F("DH2")][F("OM")] = dh->midpointOffset();
  json[F("DH2")][F("F")] = dh->fixedValue();

  serializeJson(json, buff, buffSize);
}
