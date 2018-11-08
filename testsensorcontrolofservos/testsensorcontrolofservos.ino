#include <Arduino.h>
#include <Servo.h>
#include <PID_v1.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Wire.h>
#include <utility/imumaths.h>

// start at 90 degrees
const int initialServoValue = 90;

// init servo objects
Servo rightServo;
Servo leftServo;

// init values for PID algorithm
double desired, input, output;
double kp = 1.25;
double ki = 0.04;
double kd = 0.1;
PID pid(&input, &output, &desired, kp, ki, kd, DIRECT); 

// init  bno objects
Adafruit_BNO055 bno(55);
unsigned long serialTime = 0;

/* get the array from processing and convert it to a float value. 
 * this converts the float values into bytes and then sends those bytes to the arduino the nconverts it back to floats when reciveing the data from teh arduino 
 */

// converts info from board from bytes to float
union {
  byte asBytes[24]; //float the D_paramater 
  float asFloat[6]; // float the input
  
}
values;

// receives info from board to computer
// into human readable format
void SerialReceive()
{
  int index =0;
  byte Auto_man = -1;
  byte Direct_Reverse = -1;
  while(Serial.available() && index < 26)
  {
    if (index ==0) {
      Auto_man = Serial.read();
    }
    else if (index == 1) {
      Direct_Reverse = Serial.read();
    }
    else{
      values.asBytes[index - 2] = Serial.read();
    }
    index++;
  }
  if(index == 26 && (Auto_man == 0 || Auto_man == 1) && (Direct_Reverse == 0 || Direct_Reverse == 1))
  {
    desired = double(values.asFloat[0]);

    if(Auto_man == 0)
    {
      output = double(values.asFloat[2]);
      
    }
    double p, i, d;
    p = double(values.asFloat[3]);
    i = double(values.asFloat[4]);
    d = double(values.asFloat[5]);
    pid.SetTunings(p, i, d);

    if (Auto_man ==0){
      pid.SetControllerDirection(DIRECT);
    }
    else{
      pid.SetControllerDirection(REVERSE);
    }
  }
  // removes excess data
  Serial.flush();
}

// prints updated values post-algorithm
void SerialSend()
{
  Serial.print("PID ");
  Serial.print(desired);
  Serial.print(" ");
  Serial.print(input);
  Serial.print(" ");
  Serial.print(output);
  Serial.print(" ");
  Serial.print(pid.GetKp());
  Serial.print(" ");
  Serial.print(pid.GetKi());
  Serial.print(" ");
  Serial.print(pid.GetKd());
  Serial.print(" ");

  if (pid.GetMode() == AUTOMATIC) {
    Serial.print("Automatic");
  } 
  else {
    Serial.print("Manual");
  }
  Serial.print(" ");

  if (pid.GetDirection() == DIRECT) {
    Serial.println("Direct");
  } 
  else {
     Serial.println("Reverse");
  }
}

// init setup of boards
void setup(){
  Serial.begin(9600);

  if(!bno.begin()){
    Serial.println("Mothafucker it ain't working");
    while(1);
  }
  delay(200);
  bno.setExtCrystalUse(true);
  desired = 0;
  rightServo.attach(A0);
  leftServo.attach(A1); //ports

  rightServo.write(initialServoValue);
  leftServo.write(initialServoValue);
  
  pid.SetMode(AUTOMATIC);
}

void loop() {
  // SENSOR DATA
  sensors_event_t event;
  bno.getEvent(&event);

  int degreeZ = event.orientation.z;
  input = abs(degreeZ);               
  pid.Compute();                      
 
  int newZ;
  if (degreeZ < 0) {
    newZ = degreeZ + output;          
  } else {
    newZ = degreeZ - output;      
  }
  if (newZ < -90) {
    newZ = -90;
  } else if (newZ > 90) {
    newZ = 90;
  }
  rightServo.write(newZ + initialServoValue); 

  if (millis() > serialTime) {
    SerialReceive();
    SerialSend();
    serialTime += 500;  
  }
}
