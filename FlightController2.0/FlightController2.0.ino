/*
 * VTOL by Thrust-Vectoring flight controller 
 * 2021/02/27
 * Yihia Al-Shekly
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

// output vals
Servo motor1;
Servo motor2;
Servo myservo3;
Servo myservo4;
Servo myservo5;

//Variables for time
float elapsedTime, time, timePrev;
int period = 50;  //Refresh rate period of the loop is 50ms
/////////////////// Yaw PID constants ///////////////////////
float yaw_kp = 2;
float yaw_ki = 0.05;
float yaw_kd = 10;
float yaw_desired_angle , yawAdd, yaw, yawPID; 
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
  myservo5.attach(9);
  myservo3.attach(4);
  myservo4.attach(3);

  motor1.writeMicroseconds(1000);
  motor2.writeMicroseconds(1000);

  time = millis();

  delay(5000);


}









void loop(void)
{
  time = millis();
  // Get a new sensor event
  sensors_event_t event;
  bno.getEvent(&event);

  // read RX
  if (pulseIn(A5, HIGH) > 0) {
    thrINPUT = pulseIn(A1, HIGH);
    pitchINPUT = pulseIn(A3, HIGH);
    rollINPUT = pulseIn(A2, HIGH);
    yawINPUT = pulseIn(A4, HIGH);
    stateINPUT = pulseIn(A5, HIGH);
  } else {
    Serial.println("chanels are not connected to analog pins");
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
  if (millis() > time + period) {

    Serial.println(PID.val(yaw_kp, yaw_ki, yaw_kd, yaw_desired_angle, yaw, period));
    yawPID = PID.val(yaw_kp, yaw_ki, yaw_kd, yaw_desired_angle, yaw, period);


  }





  // thrust and yaw output
  motor1.writeMicroseconds(thrINPUT - (yawPID - 1500));
  motor2.writeMicroseconds(thrINPUT + (yawPID - 1500));

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
