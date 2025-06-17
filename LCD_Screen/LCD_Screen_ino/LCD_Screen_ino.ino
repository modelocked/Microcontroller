#include <LiquidCrystal.h>

// LCD pins: RS, E, D4, D5, D6, D7
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

const int Pin_Input_PIR_1 = A0;
const float input_voltage = 5.0;

void setup() {
  pinMode(Pin_Input_PIR_1, INPUT);
  Serial.begin(9600);

  lcd.begin(16, 2);
  lcd.print("PIR_1 Voltage:");
}

void loop() {
  float input_value_1 = analogRead(Pin_Input_PIR_1) * (input_voltage / 1023.0);
  Serial.println(input_value_1);

  lcd.setCursor(0, 1); // Second row
  lcd.print("V: ");
  lcd.print(input_value_1, 2); // 2 decimal places
  lcd.print("     "); // Clear trailing digits

  delay(200);
}
