#include <Arduino_HTS221.h>
#include <Arduino_LPS22HB.h>
#include <Wire.h>

void scanI2CBus() {
  Serial.println("🔍 Scanning I2C bus for connected devices...");
  byte count = 0;

  for (byte address = 8; address < 120; address++) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("✅ I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println();
      count++;
    }
  }

  if (count == 0) {
    Serial.println("❌ No I2C devices found.");
  } else {
    Serial.print("🔁 I2C scan complete. ");
    Serial.print(count);
    Serial.println(" device(s) detected.\n");
  }
}

void setup() {
  Wire.begin();
  Serial.begin(9600);
  while (!Serial); // Wait for Serial Monitor to open

  scanI2CBus(); // Run I2C device detection first

  Serial.println("Checking for onboard sensors...");

  // Check HTS221 (Temperature & Humidity)
  if (!HTS.begin()) {
    Serial.println("❌ HTS221 sensor not found. This may be a Nano 33 BLE (non-Sense) or sensor is faulty.");
    while (1); // Stop here if sensor not found
  } else {
    Serial.println("✅ HTS221 sensor detected.");
  }

  // Check LPS22HB (Pressure)
  if (!BARO.begin()) {
    Serial.println("❌ LPS22HB sensor not found.");
    while (1); // Stop here if sensor not found
  } else {
    Serial.println("✅ LPS22HB sensor detected.");
  }

  Serial.println("All sensors initialised successfully.\n");
}

void loop() {
  float temperature = HTS.readTemperature();
  float humidity = HTS.readHumidity();
  float pressure = BARO.readPressure();

  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.print(" °C, ");

  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.print(" %, ");

  Serial.print("Pressure: ");
  Serial.print(pressure);
  Serial.println(" hPa");

  delay(1000); // Update every second
}
