const int LDRPin = A0;           // A0 reads from voltage divider (LDR + 1kΩ)
const int PIRPin1 = 2;           // Digital pin for PIR module #1
const int PIRPin2 = A1;          // Analog pin for 3-pin PIR module
const int PIRBatt = 9;           // 9V battery on the PIR. (for comment/info only)
const int LEDPin = 9;            // LED connected to D9

const unsigned long armingDelay = 20 * 1000;        // Milliseconds to arm (10 sec)
const unsigned long calibrationTime = 2 * 1000;     // LDR calibration time (2 SEC)
const unsigned long calibrationInterval = 3150000;  // Recalibrate every 2.5 min

bool latchedAlarm = false;       // Stores the persistent state of the alarm
bool latchEnabled = true;        // Set to false for real-time behavior

float LDR_tolerance_percent = 10.0;       // % deviation allowed from threshold
int LDR_ambient_light = 20;               // Will be measured

float PIR2baseline= 1.8;                // The untriggered voltage output (should be 0V).
float PIR_tolerance_percent = 20.0;       // % deviation allowed from baseline to trigger PIR

unsigned long lastCalibrationTime = 0;    // For periodic LDR calibration
bool firstLoop = true;                    // Controls arming delay

String alarmCause = "None";               // Tracks which sensor triggered the alarm

// Function declarations
bool LDR_monitor();
bool PIR_monitor();
bool sensor_monitor();
void alarm_monitor(bool alarm);
int calibrate_LDR_threshold();

void calibrate3pinPIR();
void arming_delay();
void calibrate_LDR_ambient();

void setup() {
  pinMode(LEDPin, OUTPUT);
  pinMode(PIRPin1, INPUT);
  Serial.begin(9600);
  delay(500); // Give the serial connection time to open

}

void loop() {
  // One-time arming delay
  arming_delay();

  // Periodic re-calibration of LDR
  if (millis() - lastCalibrationTime >= calibrationInterval) {
    calibrate_LDR_ambient();  // Recalibrate ambient light
  }

  if (latchedAlarm) {
    Serial.println("Loop halted due to latched alarm.");
    while (true);  // Infinite halt
  }

  bool alarm = sensor_monitor();
  alarm_monitor(alarm);
}

void arming_delay() {
  if (firstLoop) {
    Serial.print("Arming commenced. Delay: ");
    Serial.print(armingDelay);
    Serial.println(" ms");
    delay(armingDelay);
    firstLoop = false;
    Serial.println("System Armed");

  calibrate_LDR_ambient();    // Properly calibrate ambient light
  calibrate3pinPIR();         // Measure idle voltage of analog PIR

  }
}

void calibrate_LDR_ambient() {
  unsigned long startTime = millis();
  unsigned long sum = 0;
  int count = 0;

  Serial.println("Measuring LDR ambient light...");

  while (millis() - startTime < calibrationTime) {
    int LDRValue = analogRead(LDRPin);
    sum += LDRValue;
    count++;
    delay(50);
  }

  if (count > 0) {
    LDR_ambient_light = sum / count;
    Serial.print("LDR ambient calibrated to: ");
    Serial.println(LDR_ambient_light);
  } else {
    Serial.println("LDR calibration failed.");
  }

  lastCalibrationTime = millis();
}

void calibrate3pinPIR() {
  unsigned long startTime = millis();
  unsigned long sum = 0;
  int count = 0;

  Serial.println("Calibrating 3-pin PIR sensor...");

  while (millis() - startTime < calibrationTime) {
    int val = analogRead(PIRPin2);
    sum += val;
    count++;
    delay(50);
  }

  if (count > 0) {
    float avgADC = (float)sum / count;
    PIR2baseline = avgADC * (5.0 / 1023.0);
    Serial.print("PIR2 average voltage: ");
    Serial.print(PIR2baseline);
    Serial.println(" V");
  } else {
    Serial.println("PIR2 calibration failed.");
  }
}

bool LDR_monitor() {
  int LDRValue = analogRead(LDRPin);

  float lowerBound = LDR_ambient_light * (1.0 - LDR_tolerance_percent / 100.0);
  float upperBound = LDR_ambient_light * (1.0 + LDR_tolerance_percent / 100.0);

  Serial.print("LDR reading: ");
  Serial.println(LDRValue);

  if (LDRValue < lowerBound || LDRValue > upperBound) {
    Serial.print("LDR out of range (");
    Serial.print(lowerBound);
    Serial.print(" - ");
    Serial.print(upperBound);
    Serial.println(")");
    return true;
  }

  return false;
}

bool PIR_monitor() {
  int PIRState1 = digitalRead(PIRPin1);             // 2-pin digital PIR
  int raw2 = analogRead(PIRPin2);                   // 3-pin analog PIR
  float voltage2 = raw2 * (5.0 / 1023.0);           

  Serial.print("PIR No8 (digital): ");
  Serial.print(PIRState1);
  Serial.print(" | 3 Pin PIR (analog) voltage: ");
  Serial.println(voltage2, 2);  // Show 2 decimal places
  float PIR2_trigger = PIR2baseline * (1.0 + PIR_tolerance_percent / 100.0);

  if (PIRState1 == HIGH) {
    alarmCause = "PIR No 8 (2-pin digital)";
    Serial.println("Alarm triggered by PIR No 8 (digital)");
    return true;

  } else if (voltage2 > PIR2_trigger) {
    alarmCause = "3 pin PIR (analogue)";
    Serial.println("Alarm triggered by 3 pin PIR (analogue)");
    return true;
  }

  return false;
}

bool sensor_monitor() {
  bool LDRAlarm = LDR_monitor();
  if (LDRAlarm) {
    alarmCause = "LDR";
    Serial.println("Alarm triggered by LDR");
    return true;
  }

  bool PIRAlarm = PIR_monitor();
  return PIRAlarm;
}

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
