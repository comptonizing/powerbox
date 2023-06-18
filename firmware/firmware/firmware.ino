#include "Devices.h"
#include "Settings.h"
#include "CommandBuffer.h"

#define BUFFSIZE 256

void setup(void)
{
  Serial.begin(9600);
  Settings::i().setup();
}

void loop(void)
{
  Settings::i().loop();

  delay(1000);
}
