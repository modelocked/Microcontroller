const int Pin_Input_PIR_1 = A0; 
//const int Pin_Output_PIR_1 = A1;
//const int Pin_Input_PIR_2 = A2; 
//const int Pin_Output_PIR_2 = A3;
const float input_voltage = 5.;
const unsigned long measurementInterval = 5UL * 60UL * 1000UL; // 5 minutes

void setup() {
  // put your setup code here, to run once:
pinMode(Pin_Input_PIR_1, INPUT);
//pinMode(Pin_Output_PIR_1, INPUT);
//pinMode(Pin_Input_PIR_2, INPUT);
//pinMode(Pin_Output_PIR_2, INPUT);
Serial.begin(9600); 
}

void loop() {
  // put your main code here, to run repeatedly:
  float input_value_1 = analogRead(Pin_Input_PIR_1)* (input_voltage / 1023.0);
  //float output_value_1 = analogRead(Pin_Output_PIR_1)* (input_voltage / 1023.0);
  //float input_value_2 = analogRead(Pin_Input_PIR_2)* (input_voltage / 1023.0);
  //float output_value_2 = analogRead(Pin_Output_PIR_2)* (input_voltage / 1023.0);

  Serial.println(input_value_1,3);
  //Serial.print(" ");
  //Serial.println(output_value_1);
  //Serial.print(input_value_2);
  //Serial.print(" ");
  //Serial.println(output_value_2);

  delay(measurementInterval);
}
