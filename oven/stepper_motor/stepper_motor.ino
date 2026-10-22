const int stepPin = 3;
const int dirPin = 4;

void setup() {
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

  digitalWrite(dirPin, HIGH);
}

void loop() {
  // Rotate one direction
  digitalWrite(dirPin, HIGH);

  for (int i = 0; i < 200; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(2000);

    digitalWrite(stepPin, LOW);
    delayMicroseconds(2000);
  }

  delay(1000);

  // Rotate the other direction
  digitalWrite(dirPin, LOW);

  for (int i = 0; i < 200; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(2000);

    digitalWrite(stepPin, LOW);
    delayMicroseconds(2000);
  }

  delay(1000);
}