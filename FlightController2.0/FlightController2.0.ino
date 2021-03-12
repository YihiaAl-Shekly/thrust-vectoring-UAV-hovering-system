/*
   VTOL by Thrust-Vectoring flight controller
   2021/02/27
   Yihia Al-Shekly
*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>
// including my PID class
#include "PID_Controls.h"
PID_Controls PID;

Adafruit_BNO055 bno = Adafruit_BNO055(55);

// RX input vars
double thrINPUT;
double pitchINPUT;
double rollINPUT;
double yawINPUT;
double stateINPUT;
bool radioInUse = false;
// output vals
Servo motor1;
Servo motor2;
Servo myservo1;
Servo myservo2;
Servo myservo3;

//Variables for time
float elapsedTime, time, timePrev;
/////////////////// Yaw PID constants ///////////////////////
float yaw_kp = 0.073; // 0.07
float yaw_ki = 0.002;// 0.001
float yaw_kd = 0.065; // 0.05
float yaw_desired_angle , yawAdd, yaw, yawPID, yawOUT1, yawOUT2;
/////////////////////////////////////////////////////////////

void setup(void)
{

  Serial.begin(115200);
  // BNO055
  Serial.println("Orientation Sensor Test"); Serial.println("");
  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  bno.setExtCrystalUse(true);

  // def input
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  // def output
  motor1.attach(11);
  motor2.attach(10);
  myservo1.attach(9);
  myservo2.attach(3);
  myservo3.attach(4);

  motor1.writeMicroseconds(1000);
  motor2.writeMicroseconds(1000);

  // servo sequins
  for (int x = 1000; x < 2000; x++) {
    myservo1.writeMicroseconds(x);
    myservo2.writeMicroseconds(x);
    myservo3.writeMicroseconds(x);
    delay(1);
  }
  for (int x = 2000; x > 1000; x--) {
    myservo1.writeMicroseconds(x);
    myservo2.writeMicroseconds(x);
    myservo3.writeMicroseconds(x);
    delay(1);
  }
  for (int x = 1000; x < 1500; x++) {
    myservo1.writeMicroseconds(x);
    myservo2.writeMicroseconds(x);
    myservo3.writeMicroseconds(x);
    delay(1);
  }
  myservo1.writeMicroseconds(1500);// only using 1 & 2 so far
  myservo2.writeMicroseconds(1500);
  myservo3.writeMicroseconds(1500);
  // end sequins

  time = millis();

  delay(1000);


}









void loop(void)
{
  // time
  timePrev = time;  // the previous time is stored before the actual time read
  time = millis();  // actual time read
  elapsedTime = (time - timePrev) / 1000;

  // Get a new sensor event
  sensors_event_t event;
  bno.getEvent(&event);

  // read RX
  if (radioInUse) {
    if (pulseIn(A5, HIGH) > 0) {
      thrINPUT = pulseIn(A1, HIGH);
      pitchINPUT = pulseIn(A3, HIGH);
      rollINPUT = pulseIn(A2, HIGH);
      yawINPUT = pulseIn(A4, HIGH);
      stateINPUT = pulseIn(A5, HIGH);
    } else {
      Serial.println("chanels are not connected to analog pins");
    }
  } else {
    thrINPUT = 1000;
    pitchINPUT = 1500;
    rollINPUT = 1500;
    yawINPUT = 1500;
    stateINPUT = 2000;
  }

  // creating the  yaw_desired_angle
  yawAdd = map (yawINPUT, 1000, 2000 , -10, 10);
  if (abs(yawAdd) > 1) {
    yaw_desired_angle  = yaw_desired_angle + yawAdd;
  }
  //Serial.println(yaw_desired_angle);


  // creating the modified yaw true angel (heading)
  yaw = map (event.orientation.x, 0, 360, -360, 360);
  if (yaw > 1) {
    yaw = yaw - 360;
  } else {
    yaw = yaw + 360;
  }
  //Serial.println(yaw);

  ///////////////////pid call////////////////

  //Serial.println(PID.val(yaw_kp, yaw_ki, yaw_kd, yaw_desired_angle, yaw, period));
  yawPID = PID.val(yaw_kp, yaw_ki, yaw_kd, yaw_desired_angle, yaw, elapsedTime);


  /////////////////// thrust and yaw output ///////////////////////////

  thrINPUT = map(thrINPUT, 1000, 2000, 1000, 1300); // thruttel cap

  yawOUT1 = thrINPUT + yawPID;

  //yawOUT1 = map(yawOUT1, 500, 2500, 1000, 2000);

  if (yawOUT1 < 1000) {
    yawOUT1 = 1000;
  }
  if (yawOUT1 > 2000) {
    yawOUT1 = 2000;
  }


  yawOUT2 = thrINPUT - yawPID;

  //yawOUT2 = map(yawOUT2, 500, 2500, 1000, 2000);

  if (yawOUT2 < 1000) {
    yawOUT2 = 1000;
  }
  if (yawOUT2 > 2000) {
    yawOUT2 = 2000;
  }

  if (stateINPUT > 1500) {
    motor1.writeMicroseconds(yawOUT1);
    motor2.writeMicroseconds(yawOUT2);
  } else {
    motor1.writeMicroseconds(1000);
    motor2.writeMicroseconds(1000);
  }

  Serial.print(yawOUT1);
  Serial.print("  ");
  Serial.print(yawOUT2);
  Serial.print("  ");
  Serial.println(yawPID);

  /*
    // imu debug
     //Display the floating point data*
      Serial.print("X: ");
      Serial.print(event.orientation.x, 4);
      Serial.print("\tY: ");
      Serial.print(event.orientation.y, 4);
      Serial.print("\tZ: ");
      Serial.print(event.orientation.z, 4);
      Serial.println("");
    /*

      // RX debug
      Serial.print(thrINPUT);
      Serial.print(" ");
      Serial.print(pitchINPUT);
      Serial.print(" ");
      Serial.print(rollINPUT);
      Serial.print("  ");
      Serial.print(yawINPUT);
      Serial.print("  ");
      Serial.println(stateINPUT);

  */

}
