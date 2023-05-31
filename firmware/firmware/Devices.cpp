#include "Devices.h"

Devices &Devices::i() {
  static Devices currentInstance;
  return currentInstance;
}

Devices::Devices() {
}

Devices::~Devices() {
}
