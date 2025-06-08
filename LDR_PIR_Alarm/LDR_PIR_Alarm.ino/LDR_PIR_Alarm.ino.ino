const int LDRPin = A0;         // A0 reads from voltage divider (LDR + 1kΩ)
const int PIRPin1 = 2;          // using digital pin 2
const int PIRPin2 = A1;          // using digital pin 3
const float PIR2threshold = 1.8
const int PIRBatt = 9;         // 9V battery on the PIR. for scaling to 5v
const int LEDPin = 9;          // LED connected to D9

const unsigned long armingDelay = 1*30000; // Milliseconds to arm where 60000 is a minute

const unsigned long calibrationTime = 2000; // Milliseconds used to monitor LDR measurements and calibrate
unsigned long lastCalibrationTime = 0; //Want that the LDR is calibrated to ambient light every calibrationInterval 
const unsigned long calibrationInterval = 60000; // 10 minutes in milliseconds



bool latchedAlarm = false;     // Stores the persistent state of the alarm
bool latchEnabled = true;      // Set to false for real-time behavior

int LDR_threshold = 20;        // LDR: <40 = bright → alarm ON
float LDR_threshold_set = 10.0;        // Set the LDR threshold at LDR_threshold_set percentage below ambient
int LDR_ambient_light = 20;        // LDR: <40 = bright → alarm ON

String alarmCause = "None";    // Tracks which sensor triggered the alarm
bool firstLoop = true;         // used to include a delay before the first loop

// Function declarations
bool LDR_monitor();
bool PIR_monitor();
bool sensor_monitor();
void alarm_monitor(bool alarm);
void arming_delay();           // Declare arming_delay()

void calibrate_LDR_threshold() {
  unsigned long startTime = millis();
  unsigned long sum = 0;
  int count = 0;

  Serial.println("Calibrating LDR threshold...");

  while (millis() - startTime < calibrationTime) {
    int LDRValue = analogRead(LDRPin);
    sum += LDRValue;
    count++;
    delay(50);  // Sample every 50ms (~20 samples per second)
  }

  if (count > 0) {
    LDR_ambient_light = sum / count;
    Serial.print("Ambient light measured at an average value of: ");
    Serial.println(LDR_ambient_light);

    LDR_threshold = round(LDR_ambient_light * (1.0 - (LDR_threshold_set / 100.0)));
    Serial.print("LDR threshold set at: ");
    Serial.println(LDR_threshold);
  } else {
    Serial.println("Calibration failed: no samples taken.");
  }
}

const int PIRPin2 = A1;                // Analog pin connected to 3-pin PIR output
float PIR2threshold = 1.8;             // Will be calibrated

const unsigned long armingDelay = 10000;  // Delay duration in milliseconds

void calibrate3pinPIR() {
  unsigned long startTime = millis();
  unsigned long sum = 0;
  int count = 0;

  Serial.println("Calibrating 3-pin PIR sensor...");

  while (millis() - startTime < armingDelay) {
    int rawValue = analogRead(PIRPin2);
    sum += rawValue;
    count++;
    delay(50);  // ~20 samples per second
  }

  if (count > 0) {
    float averageADC = (float)sum / count;
    PIR2threshold = averageADC * (5.0 / 1023.0);  // Convert ADC average to volts
    Serial.print("PIR2 threshold calibrated to: ");
    Serial.print(PIR2threshold);
    Serial.println(" V");
  } else {
    Serial.println("PIR calibration failed: no samples taken.");
  }
}


void setup() {
  pinMode(LEDPin, OUTPUT);     // Set LED pin as output
  pinMode(PIRPin1, INPUT);      // Set PIR as digital input
  //pinMode(PIRPin2, INPUT);      // Set PIR as digital input
  Serial.begin(9600);          // Serial for debug
  calibrate_LDR_threshold();  // Uses armingDelay to collect LDR data
  calibrate3pinPIR(); // Uses armingDelay to collect PIR data
}

void arming_delay() {
  if (firstLoop) {
    Serial.print("Arming commenced. Delay: ");
    Serial.print(armingDelay);
    Serial.println(" ms");

    delay(armingDelay);  // Use global constant
    firstLoop = false;
    Serial.print("Armed");
  }
}


void loop() {
  
  

  arming_delay();              // Arm

  // Recalibrate LDR every 10 minutes
  if (millis() - lastCalibrationTime >= calibrationInterval) {
    calibrate_LDR_threshold();
    lastCalibrationTime = millis();
  }

  // If alarm latched, stop all further execution
  if (latchedAlarm) {
    Serial.println("Loop halted due to latched alarm.");
    while (true);              // Infinite pause — loop stops here
  }

  bool alarm = sensor_monitor();   // Check sensors
  alarm_monitor(alarm);            // React based on sensor status
  delay(1000);                     // Loop delay
}

// Function: reads from A0 and returns LDR alarm state
bool LDR_monitor() {
  int LDRValue = analogRead(LDRPin);
  Serial.print("LDR Value: ");
  Serial.println(LDRValue);
  return (LDRValue < LDR_threshold); // true = alarm (bright)
}

// Function: reads from D2 and A1 and returns PIR alarm state
bool PIR_monitor() {
  int PIRState1 = digitalRead(PIRPin1);           // Digital read from PIR #1
  int PIRRaw2 = analogRead(PIRPin2);              // Analog read from 3-pin PIR
  float PIRVoltage2 = PIRRaw2 * (5.0 / 1023.0);   // Convert to voltage

  Serial.print("PIR1 (no 8): ");
  Serial.print(PIRState1);
  Serial.print(" | PIR2 (3 Pin) voltage: ");
  Serial.println(PIRVoltage2);

  // Motion is detected if digital PIR is HIGH or analog PIR exceeds calibrated threshold
  return (PIRState1 == HIGH || PIRVoltage2 > PIR2threshold);
}



// Function: aggregates all sensor alarms with early break + cause reporting
bool sensor_monitor() {
  bool LDRAlarm = LDR_monitor();
  bool PIRAlarm = PIR_monitor();

  if (LDRAlarm) {
    alarmCause = "LDR";
    Serial.println("Alarm triggered by LDR");
    return true;
  } else if (PIRAlarm) {
    alarmCause = "PIR";
    Serial.println("Alarm triggered by PIR");
    return true;
  } else {
    alarmCause = "None";
    Serial.println("No alarm");
    return false;
  }
}

// Function: reacts to alarm condition with optional latching
void alarm_monitor(bool alarm) {
  if (latchEnabled) {
    if (alarm && !latchedAlarm) {
      latchedAlarm = true;
      Serial.println("ALARM LATCHED");
    }
    digitalWrite(LEDPin, latchedAlarm ? HIGH : LOW);
  } else {
    digitalWrite(LEDPin, alarm ? HIGH : LOW);
  }
}
