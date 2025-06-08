/*
  === Motion + Light Alarm System ===

  Description:
  This sketch is designed for a simple intruder or environmental alarm system
  using an LDR (Light Dependent Resistor) and two types of PIR (Passive Infrared)
  motion detectors:

    - LDR (analog): Detects ambient light changes.
    - Pin_PIR_No8 (digital): 2-pin digital PIR sensor (labelled "No8").
    - Pin_PIR_1_Digital (digital): 3-pin PIR module with digital output.

  Key Features:
    - Delayed arming to allow setup time.
    - Calibration of ambient light level on startup and periodically.
    - Alarm latching to prevent repeated triggers.
    - LED output to indicate alarm status.
    - Serial feedback to track system behavior.

  Wiring:
    - LDR connected to A0 through voltage divider.
    - Pin_PIR_No8 (2-pin PIR) connected to D2.
    - Pin_PIR_1_Digital (3-pin PIR, digital output) connected to D3.
    - LED alarm indicator on D9.
*/

//Assign analogue pins
const int Pin_LDR= A0;           // A0 reads from voltage divider (500Ω + 300Ω)


//Assign digital pins
const int Pin_PIR_No8 = 2;           // Digital pin for PIR no8
const int Pin_PIR_1_Digital = 3;          // Digital pin for 3-pin PIR module
const int Pin_PIR_2_Digital = 6;          // Digital pin for 3-pin PIR module
const int Pin_Arming = 4;          // Arming toggle switch
const int PIN_Break_Screen = 5;
const int Pin_AIR_AbusLS1020 = 7;          // AIR sensor Abus LS1020
const int Pin_Output = 9;            // LED connected to D9


//Assign other constants
const int Battery_PIR_No8 = 9;           // 9V battery on the PIR. (for comment/info only)
const unsigned long armingDelay = 10 * 1000;        // Milliseconds to arm (10 sec)
const unsigned long disarmDelay = 10 * 1000;        // Milliseconds to disarm (10 sec)

bool Output_State = false;       // Stores the persistent state of the alarm
bool latchEnabled = true;        // Set to false for real-time behavior

float LDR_tolerance_percent = 20.0;       // % deviation allowed from threshold
int LDR_ambient_light = 20;               // Will be measured

unsigned long lastCalibrationTime = 0;
const unsigned long calibrationInterval = 5UL * 60UL * 1000UL; // 10 minutes
const unsigned long calibrationTime = 2 * 1000;     // LDR calibration time (2 SEC)

bool firstLoop = true;                    // Controls arming delay

String alarmCause = "None";               // Tracks which sensor triggered the alarm

// Declare functions
bool LDR_monitor();
bool PIR_monitor();
bool break_screen_monitor();
bool sensor_monitor();
void alarm_monitor(bool alarm);
void arming_delay();
void calibrate_LDR_ambient();

void setup()
{
  //Setup the digital pins
  pinMode(Pin_Output, OUTPUT);
  pinMode(Pin_PIR_No8, INPUT_PULLUP);
  pinMode(Pin_PIR_1_Digital, INPUT);
  pinMode(Pin_PIR_2_Digital, INPUT);
  pinMode(Pin_AIR_AbusLS1020, INPUT_PULLUP);
  pinMode(PIN_Break_Screen, INPUT);
  pinMode(Pin_Arming, INPUT_PULLUP);
  
  //Other setup requirement
  Serial.begin(9600);  // Initialises serial communication at 9600 bits per second (baud) for debugging or data output via USB
  delay(500); // Give the serial connection time to open
}

void loop() {
  if (digitalRead(Pin_Arming) == HIGH)
  {
    arming_delay();  // One-time arming delay

    // Periodic re-calibration of LDR
    if (millis() - lastCalibrationTime >= calibrationInterval)
    {
      calibrate_LDR_ambient();
      lastCalibrationTime = millis();
    }

    if (Output_State)
    {
      Serial.println("Loop halted due to latched alarm.");
      while (true);  // Infinite halt
    }

    // Check sensor-based inputs. Alarm activates if any trigger
    bool alarm = sensor_monitor();

    alarm_monitor(alarm);
  }
  // Else: do nothing
}


void arming_delay()
{
  if (firstLoop)
  {
    Serial.print("Arming commenced. Delay: ");
    Serial.print(armingDelay);
    Serial.println(" ms");
    delay(armingDelay);
    firstLoop = false;
    Serial.println("System Armed");
  calibrate_LDR_ambient();    // Properly calibrate ambient light
  }
}

void calibrate_LDR_ambient()
{
 // Serial.println("=== Starting LDR Calibration ===");

  unsigned long startTime = millis();
  unsigned long sum = 0;
  int count = 0;

  Serial.println("Measuring LDR ambient light...");

  while (millis() - startTime < calibrationTime)
  {
    int LDRValue = analogRead(Pin_LDR);
    sum += LDRValue;
    count++;
    delay(1);
  }

  if (count > 0)
  {
    LDR_ambient_light = sum / count;
    Serial.print("LDR ambient calibrated to: ");
    Serial.println(LDR_ambient_light);
  } else
  {
    Serial.println("LDR calibration failed.");
  }

}

//include an armed indicator
//include a Pressure plate switch

bool AIR_monitor()
{
  int AIRState = digitalRead(Pin_AIR_AbusLS1020);
  return (AIRState == LOW);
}



bool LDR_monitor()
{
  int LDRValue = analogRead(Pin_LDR);

  float lowerBound = LDR_ambient_light * (1.0 - LDR_tolerance_percent / 100.0);
  float upperBound = LDR_ambient_light * (1.0 + LDR_tolerance_percent / 100.0);

  //Serial.print("LDR reading: ");
  //Serial.println(LDRValue);

  if (LDRValue < lowerBound || LDRValue > upperBound)
  {
    Serial.print("LDR out of range (");
    Serial.print(lowerBound);
    Serial.print(" - ");
    Serial.print(upperBound);
    Serial.println(")");
    return true;
  }

  return false;
}

bool PIR_monitor()
{
  int PIRState1 = digitalRead(Pin_PIR_No8);             // 2-pin digital PIR no8
  int PIRState2 = digitalRead(Pin_PIR_1_Digital);             // 3-pin digital PIR 1 
  int PIRState3 = digitalRead(Pin_PIR_2_Digital);             // 3-pin digital PIR 2

  //Serial.print("PIR No8 (digital): ");
  //Serial.print(PIRState1);
  //Serial.print(" | 3 Pin PIR (digital) voltage: ");
  //Serial.print(PIRState2);
  
  if (PIRState1 == HIGH)
  {
    alarmCause = "PIR No 8 (2-pin digital)";
    Serial.println("Alarm triggered by PIR No 8 (digital)");
    return true;

  }
  
  else if (PIRState2 == HIGH)
  {
    alarmCause = "3 pin PIR 1 (digital)";
    Serial.println("Alarm triggered by 3 pin PIR 1 (digital)");
    return true;
  }

  else if (PIRState3 == HIGH)
  {
    alarmCause = "3 pin PIR 2 (digital)";
    Serial.println("Alarm triggered by 3 pin PIR 2 (digital)");
    return true;
  }

  return false;

}

bool break_screen_monitor()
{
  int break_screen_state = digitalRead(PIN_Break_Screen);
  return (break_screen_state == LOW);
}

bool sensor_monitor()
{

  bool LDRAlarm = LDR_monitor();
  if (LDRAlarm) {
    alarmCause = "LDR";
    Serial.println("Alarm triggered by LDR");
    return true;
  }

  bool AIRAlarm = AIR_monitor();
  if (AIRAlarm) {
    alarmCause = "Active IR beam (AIR)";
    Serial.println("Alarm triggered by Active IR beam");
    return true;
  }

  bool break_screen_alarm = break_screen_monitor();
  if (break_screen_alarm) {
    alarmCause = "Break screen)";
    Serial.println("Alarm triggered by break screen");
    return true;
  }

  bool PIRAlarm = PIR_monitor();
  return PIRAlarm;
}

//Give time to be disarmed by owner before alarming
void disarm_prompt()
{
  Serial.println("Disarm");
  delay(disarmDelay);
}

bool disarm_check() {
  return digitalRead(Pin_Arming) == HIGH;
}

void alarm_monitor(bool alarm)
{
  if (latchEnabled)
  {

    // Trigger alarm only if alarm condition is true and output is not already latched
    if (alarm && !Output_State)
    {
      disarm_prompt(); //prompt the owner that the alarm is triggered and must be disarmed

      if (disarm_check())
      {
        Output_State = true;
        Serial.println("Alarm latched and firing");
      }
      else
      {
        Serial.println("Alarm disarmed");
        return; // Exit if disarmed (Pin_Arming is LOW)
      }
    }

    digitalWrite(Pin_Output, Output_State ? HIGH : LOW);
  }

  else
  {
    digitalWrite(Pin_Output, alarm ? HIGH : LOW);
  }

}
