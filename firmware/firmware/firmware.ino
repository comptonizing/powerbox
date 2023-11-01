#include "Devices.h"
#include "Settings.h"
#include "CommandBuffer.h"

void setup(void)
{
  Serial.begin(115200);
  Settings::i().setup();
}

void loop(void)
{
  Settings::i().loop();

  // delay(100);
}
