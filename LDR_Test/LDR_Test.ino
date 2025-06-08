const int ldrPin = A0;     // LDR connected to A0
const int ledPin = 9;      // LED controlled via D9

void setup() {
  pinMode(ledPin, OUTPUT);
  Serial.begin(9600);  // Optional: view readings
}

void loop() {
  int ldrValue = analogRead(ldrPin);
  Serial.println(ldrValue);  // For debugging

  if (ldrValue < 1020) {
    // Any value below 1020 (even 1022) = not full brightness → LED ON
    digitalWrite(ledPin, HIGH);
  } else {
    // Full brightness (1023) → LED OFF
    digitalWrite(ledPin, LOW);
  }

  delay(50); // Smooth readings
}
