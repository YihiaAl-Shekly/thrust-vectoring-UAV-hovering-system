#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);

double pitchINPUT;
double rollINPUT;
double yawINPUT;
double stateINPUT;

Servo myservo1;
Servo myservo2;
Servo myservo3;
Servo myservo4;
Servo myservo5;

void setup(void)
{
  Serial.begin(115200);
  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);

  bno.setExtCrystalUse(true);

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);

  myservo5.attach(11);
  myservo1.attach(10);
  myservo2.attach(9);
  myservo3.attach(4);
  myservo4.attach(3);
}

void loop(void)
{
  /* Get a new sensor event */
  sensors_event_t event;
  bno.getEvent(&event);
   
  pitchINPUT = pulseIn(A0, HIGH);
  rollINPUT = pulseIn(A1, HIGH);
  yawINPUT = pulseIn(A2, HIGH);
  stateINPUT = pulseIn(A3, HIGH);
  
  myservo1.write(pitchINPUT); 
  myservo2.write(pitchINPUT); 
  myservo3.write(pitchINPUT); 
  myservo4.write(pitchINPUT); 
  myservo5.write(pitchINPUT); 
  
  Serial.print(pid(1,1));
  // imu debug
  /* Display the floating point data 
  Serial.print("X: ");
  Serial.print(event.orientation.x, 4);
  Serial.print("\tY: ");
  Serial.print(event.orientation.y, 4);
  Serial.print("\tZ: ");
  Serial.print(event.orientation.z, 4);
  Serial.println("");
  // RX debug
  /*
    Serial.print(pitchINPUT);
    Serial.print(" ");
    Serial.print(rollINPUT);
    Serial.print("  ");
    Serial.print(yawINPUT);
    Serial.print("  ");
    Serial.println(stateINPUT);
  */

  
}
