#include "settings.h"
#include "devices.h"

void setup(void)
{
  Serial.begin(9600);

  pinMode(RAIL12V, OUTPUT);
  pinMode(ADJ, OUTPUT);
  pinMode(DH1PWM, OUTPUT);
  pinMode(DH2PWM, OUTPUT);

  digitalWrite(RAIL12V, LOW); // 4x12V off
  digitalWrite(ADJ, LOW); // var off
  digitalWrite(DH1PWM, LOW); // DH1 off
  digitalWrite(DH2PWM, LOW); // DH2 off
  delay(1000);
}

void loop(void)
{
  delay(1000);
}
