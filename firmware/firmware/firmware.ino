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


void setup(void)
{
  Serial.begin(9600);

  pinMode(6, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);

  digitalWrite(6, LOW); // 4x12V off
  digitalWrite(8, LOW); // var off
  digitalWrite(9, LOW); // DH1 off
  digitalWrite(10, LOW); // DH2 off
}

void loop(void)
{
}
