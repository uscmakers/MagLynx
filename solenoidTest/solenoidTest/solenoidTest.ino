const int flSolenoid = 1,
          frSolenoid = 2,
          blSolenoid = 3,
          brSolenoid = 4;
void setup() {
  // put your setup code here, to run once:
  // just want to set up solenoids as outputs, then turn them on and off again
  pinMode(flSolenoid, OUTPUT);
  pinMode(frSolenoid, OUTPUT);
  pinMode(blSolenoid, OUTPUT);
  pinMode(brSolenoid, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(flSolenoid, HIGH);
  digitalWrite(frSolenoid, HIGH);
  digitalWrite(blSolenoid, HIGH);
  digitalWrite(brSolenoid, HIGH);

  delay(1500);

  digitalWrite(flSolenoid, LOW);
  digitalWrite(frSolenoid, LOW);
  digitalWrite(blSolenoid, LOW);
  digitalWrite(brSolenoid, LOW);
  delay(1500);

}
