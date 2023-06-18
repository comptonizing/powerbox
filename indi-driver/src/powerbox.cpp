#include "powerbox.h"


static std::unique_ptr<Powerbox> shelyakDriver(new Powerbox());

Powerbox::Powerbox() {
}

Powerbox::~Powerbox() {
}

const char *Powerbox::getDefaultName() {
  return "Powerbox";
}
