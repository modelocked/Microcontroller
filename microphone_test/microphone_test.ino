// === Microphone-Based Sound Alarm System with Latching and Periodic Calibration ===
/*
  Description:
  This sketch monitors ambient sound using an analog microphone connected to A0.
  It calibrates the ambient sound level over 60 seconds at startup, and again every 5 minutes.
  If a sudden loud noise (30% above ambient) is detected, it triggers a latched digital output on pin 2.

  Features:
  - Initial and periodic microphone calibration
  - Threshold-based sound detection
  - Latching alarm output
  - Serial feedback for monitoring

  Hardware:
  - Analog microphone module connected to A0
  - Alarm output device (e.g., LED, buzzer) connected to digital pin 2
*/

const int Pin_Microphone = A0;           // Analog input pin connected to microphone output
const int Pin_Alarm_Output = 2;          // Digital output pin for alarm trigger (e.g., LED or buzzer)
const float input_voltage = 5.0;         // Reference voltage for analogRead (used if voltage scaling needed)

const unsigned long calibrationTime = 15UL * 1000UL;          // Duration of each calibration cycle (15,000 ms = 15 s)
const unsigned long calibrationInterval = 5UL * 60UL * 1000UL; // Interval between recalibrations (300,000 ms = 5 min)
const unsigned long samplingInterval = 50;                    // Delay between sound samples (50 ms)
const float threshold_percent = 4;                         // Trigger threshold (% above ambient sound level)

int ambient_mic_level = 0;            // Stores the average microphone level from calibration
bool alarm_triggered = false;         // Latches true after threshold is exceeded
unsigned long lastCalibrationTime = 0; // Tracks time of last calibration

void setup() {
  pinMode(Pin_Microphone, INPUT);           // Set microphone pin as input
  pinMode(Pin_Alarm_Output, OUTPUT);        // Set alarm output pin as output
  Serial.begin(9600);                       // Start serial communication at 9600 baud
  delay(500);                               // Brief delay to allow serial monitor connection
  calibrate_microphone();                   // Perform initial microphone calibration
  lastCalibrationTime = millis();           // Store timestamp of initial calibration
}

void loop() {
  // Check if it's time for periodic recalibration
  if (millis() - lastCalibrationTime >= calibrationInterval) {
    calibrate_microphone();                 // Recalibrate ambient microphone level
    lastCalibrationTime = millis();         // Update calibration timestamp
  }

  int mic_raw = analogRead(Pin_Microphone); // Read raw analog value from microphone
  float threshold = ambient_mic_level * (1.0 + threshold_percent / 100.0); // Compute threshold above ambient

  // Print current mic reading and dynamic threshold for monitoring
  Serial.print("Mic: ");
  Serial.print(mic_raw);
  Serial.print(" | Threshold: ");
  Serial.println(threshold);

  // Check if sound exceeds threshold and alarm hasn't been triggered yet
if (!alarm_triggered && mic_raw > threshold) {
  alarm_triggered = true;                 // Latch the alarm state
  digitalWrite(Pin_Alarm_Output, HIGH);   // Activate alarm output
  Serial.println("Alarm TRIGGERED and LATCHED");
  while (true);                           // Halt execution permanently
}


  delay(samplingInterval);                  // Wait before next sound sample
}

void calibrate_microphone() {
  unsigned long startTime = millis();       // Record start time of calibration
  unsigned long sum = 0;                    // Accumulate mic readings
  int count = 0;                             // Number of samples taken

  Serial.println("Calibrating ambient microphone level...");

  // Take readings over calibrationTime duration
  while (millis() - startTime < calibrationTime) {
    int mic = analogRead(Pin_Microphone);   // Read current mic value
    sum += mic;                             // Add to running sum
    count++;                                // Increment sample count
    delay(1);                               // Small delay to prevent excessive speed
  }

  // Compute average ambient mic level if samples were taken
  if (count > 0) {
    ambient_mic_level = sum / count;
    Serial.print("Calibrated ambient mic level: ");
    Serial.println(ambient_mic_level);
  } else {
    Serial.println("Mic calibration failed."); // In case no samples were collected
  }
}
