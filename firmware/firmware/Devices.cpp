#include "Devices.h"

Devices &Devices::instance() {
  static Devices currentInstance;
  return currentInstance;
}

Devices::Devices() {
}

Devices::~Devices() {
}
