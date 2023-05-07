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
