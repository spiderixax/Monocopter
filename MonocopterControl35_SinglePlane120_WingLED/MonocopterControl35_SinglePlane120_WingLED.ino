#define rcPin1 A3   // Pin 8 Connected to CH1 of Transmitter;
#define rcPin2 A2   // Pin 9 Connected to CH2
#define rcPin3 A1   // Wing
#define buzzerPin 5
//#define flapPin 2
#define wingPin 2
#define blueLedMCCU 9
#define redLedMCCU 10
#define blueLedWing 3
#define redLedWing 4
#include <Servo.h>
#include <QMC5883L.h>
#include <Wire.h>

#include "PinChangeInterrupt.h"

QMC5883L compass;
//Servo ServoFlap;
Servo ServoWing;
int x_mid = 1520;
int y_mid = 1500;
int z_mid = 1520;
volatile int x = x_mid;  // Receiver Channel 2 PPM value
volatile int y = y_mid;  // Receiver Channel 1 PPM value
volatile int z = z_mid;
float flight_bearing;
float current_bearing;
float flight_magnitude = 0;
float x_vec = 0;
float y_vec = 0;
float z_vec = 0;

volatile int prev_time_x = 0;
volatile int prev_time_y = 0;
volatile int prev_time_z = 0;

//float default_angle_flap = 95;
//float servo_angle_flap = default_angle_flap;
//int currentFlapAngle = default_angle_flap;
//int flapUpdateDifference;

//float default_angle_wing = 90;
float default_angle_wing = 65;
float servo_angle_wing = default_angle_wing;
int currentWingAngle = default_angle_wing;
int wingUpdateDifference;

float flap_constant = 0.0006;
float voltageReading = 0;
int cycleCounter = 0;
#define cycleDelay 60


void setup() {
  pinMode(rcPin1, INPUT);
  pinMode(rcPin2, INPUT);
  pinMode(rcPin3, INPUT);
  //pinMode(13, INPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(blueLedMCCU, OUTPUT);
  pinMode(redLedMCCU, OUTPUT);
  pinMode(blueLedWing, OUTPUT);
  pinMode(redLedWing, OUTPUT);

  //ServoFlap.attach(flapPin);
  //ServoFlap.write(default_angle_flap);
  ServoWing.attach(wingPin);
  ServoWing.write(default_angle_wing);
  Wire.begin();
  compass.init();
  compass.setSamplingRate(200);

  //Serial.begin(9600);


  attachPCINT(digitalPinToPCINT(rcPin1), rising_x, RISING);
  attachPCINT(digitalPinToPCINT(rcPin2), rising_y, RISING);
  attachPCINT(digitalPinToPCINT(rcPin3), rising_z, RISING);
  delay(500);
  

}

void loop() {

  if (cycleCounter == cycleDelay) {
    voltageReading = analogRead(A7)*0.00983;
    //Serial.println(voltageReading);
    if (voltageReading > 5.00 and voltageReading < 7.45) {
      //PORTC = PORTC | B00100000;  //not working for some reason
      digitalWrite(buzzerPin, HIGH);
    }
    else {
      //PORTC = PORTC & B11011111;
      digitalWrite(buzzerPin, LOW);
    }
    cycleCounter = 0;
  }

  else {
    cycleCounter++;
  }


  x_vec = x_mid - x;
  y_vec = y_mid - y;
  z_vec = z_mid - z;

  int heading = compass.readHeading();
  
  if(heading==0) {
    /* Still calibrating, so measure but don't print */
  }
  else {
    current_bearing = heading;
  }

  //Serial.println(current_bearing);
  //Serial.println(x_vec);
  //delay(200);

  current_bearing = int(int(current_bearing * 0.2) * 5);

  if (current_bearing > 80 and current_bearing < 160) {
    //digitalWrite(blueLedMCCU, HIGH);
    PORTB = PORTB | B00000010;
  }

  else {
    //digitalWrite(blueLedMCCU, LOW);
    PORTB = PORTB & B11111101;
  }

  if (current_bearing > 260 and current_bearing < 340) {
    //digitalWrite(redLedMCCU, HIGH);
    PORTB = PORTB | B00000100;
  }

  else {
    //digitalWrite(redLedMCCU, LOW);
    PORTB = PORTB & B11111011;
  }


  if ((current_bearing > 50 and current_bearing < 130) or (current_bearing > 230 and current_bearing < 310)) {
    digitalWrite(blueLedWing, HIGH);
    digitalWrite(redLedWing, HIGH);
  }





  else if ((current_bearing >= 0 and current_bearing < 40) or (current_bearing > 320 and current_bearing <= 360)) {
    digitalWrite(blueLedWing, HIGH);
    digitalWrite(redLedWing, LOW);
  }

  //else {
    //digitalWrite(blueLedWing, LOW);
  //}

  else if (current_bearing > 140 and current_bearing < 220) {
    digitalWrite(redLedWing, HIGH);
    digitalWrite(blueLedWing, LOW);
  }

  //else {
    //digitalWrite(redLedWing, LOW);
  //}

  else {
    digitalWrite(blueLedWing, LOW);
    digitalWrite(redLedWing, LOW);
  }


  
  //servo_angle_wing = default_angle_wing + (z_vec * 0.08);

  //wingUpdateDifference = currentWingAngle - servo_angle_wing;

  //if (wingUpdateDifference > 3 or wingUpdateDifference < -3) {
    //ServoWing.write(servo_angle_wing);
    //currentWingAngle = servo_angle_wing;
  //}

  //else {}


  flight_magnitude = sqrt(sq(x_vec) + sq(y_vec));


  if ((x_vec >= 0) && (y_vec >= 0)){
    flight_bearing = 90 - (57.29578 * atan(y_vec / x_vec));

  }

  else if ((x_vec >= 0) && (y_vec < 0)){
    flight_bearing = 90 + (57.29578 * atan(y_vec / - x_vec));

  }

  else if ((x_vec < 0) && (y_vec >= 0)){
    flight_bearing = 270 + (57.29578 * atan(y_vec / - x_vec));

  }

  else if ((x_vec < 0) && (y_vec < 0)){
    flight_bearing = 270 - (57.29578 * atan(y_vec / x_vec));

  }

  //Serial.println(flight_bearing);
  //Serial.println(flight_magnitude);



  if (flight_magnitude > 40) {
    //servo_angle_wing = default_angle_wing - (z_vec * 0.1) - (flap_constant * flight_magnitude * 57.29578 * sin(0.01745329 * (current_bearing - flight_bearing) + 4.712389 - 1.57080));
    servo_angle_wing = default_angle_wing - (flap_constant * flight_magnitude * 57.29578 * sin(0.01745329 * (current_bearing - flight_bearing) + 4.712389 - 1.57080));
  }

  else if (flight_magnitude <= 40) {
    //servo_angle_wing = default_angle_wing - (z_vec * 0.1);
    servo_angle_wing = default_angle_wing;
  }

  //servo_angle_flap = int(int(servo_angle_flap * 0.2) * 5);

  //int flapUpdateDifference = ((int(currentFlapAngle - servo_angle_flap)) ^ 2);
  wingUpdateDifference = currentWingAngle - servo_angle_wing;

  if (wingUpdateDifference > 0 or wingUpdateDifference < -0) {
    ServoWing.write(servo_angle_wing);
    currentWingAngle = servo_angle_wing;
  }

  else {}
  
  //Serial.println(flapUpdateDifference);
  
}




void rising_x() {
  attachPCINT(digitalPinToPCINT(rcPin1), falling_x, FALLING);
  prev_time_x = micros();
}
 
void falling_x() {
  attachPCINT(digitalPinToPCINT(rcPin1), rising_x, RISING);
  x = micros()-prev_time_x;
}

void rising_y() {
  attachPCINT(digitalPinToPCINT(rcPin2), falling_y, FALLING);
  prev_time_y = micros();
}
 
void falling_y() {
  attachPCINT(digitalPinToPCINT(rcPin2), rising_y, RISING);
  y = micros()-prev_time_y;
}

void rising_z() {
  attachPCINT(digitalPinToPCINT(rcPin3), falling_z, FALLING);
  prev_time_z = micros();
}
 
void falling_z() {
  attachPCINT(digitalPinToPCINT(rcPin3), rising_z, RISING);
  z = micros()-prev_time_z;
}
