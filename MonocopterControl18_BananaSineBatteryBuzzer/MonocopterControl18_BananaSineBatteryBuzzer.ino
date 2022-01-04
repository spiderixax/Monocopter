#define rcPin1 8   // Pin 8 Connected to CH1 of Transmitter;
#define rcPin2 9   // Pin 9 Connected to CH2
#define rcPin3 10   // Flap
#define buzzerPin 7
#include <Servo.h>
#include <QMC5883L.h>
#include <Wire.h>
QMC5883L compass;
Servo Servo1;
int flappingWing = 2;
int x_mid = 1520;
int y_mid = 1500;
int z_mid = 1520;
int x = x_mid;  // Receiver Channel 2 PPM value
int y = y_mid;  // Receiver Channel 1 PPM value
int z = z_mid;
float flight_bearing;
float current_bearing;
float flight_magnitude = 0;
float x_vec = 0;
float y_vec = 0;
float z_vec = 0;
float default_angle_1 = 90;
float neutral_angle_1 = default_angle_1;
float servo_angle_1 = default_angle_1;
float flap_constant = 0.002;
float voltageReading = 0;
int cycleCounter = 0;
#define cycleDelay 30


void setup() {
  pinMode(rcPin1, INPUT);
  pinMode(rcPin2, INPUT);
  pinMode(rcPin3, INPUT);
  pinMode(buzzerPin, OUTPUT);

  Servo1.attach(flappingWing);
  Wire.begin();
  compass.init();
  compass.setSamplingRate(200);

  //Serial.begin(9600);
}

void loop() {

  

  if (cycleCounter == cycleDelay) {
    x = pulseIn(rcPin1, HIGH, 50000);  // (Pin, State, Timeout)
    y = pulseIn(rcPin2, HIGH, 50000);
    z = pulseIn(rcPin3, HIGH, 50000);
    voltageReading = analogRead(A7)*0.00909;
    //Serial.println(voltageReading);
    //voltageReading > 4.00 and
    //if ( voltageReading < 7.45) {
      //digitalWrite(buzzerPin, HIGH);
    //}
    //else {
      //digitalWrite(buzzerPin, LOW);
    //}
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

  current_bearing = int(int(current_bearing * 0.2) * 5);

  
  if ((z_vec > 15) or (z_vec < -15)){
    neutral_angle_1 = default_angle_1 - (z_vec * 0.08);

  }

  else {
    neutral_angle_1 = default_angle_1;
  }


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



  if (flight_magnitude > 25) {
    servo_angle_1 = neutral_angle_1 - (flap_constant * flight_magnitude * 57.29578 * sin(0.01745329 * (current_bearing - flight_bearing) + 4.712389));
  }

  else if (flight_magnitude <= 25) {
    servo_angle_1 = neutral_angle_1;
  }

  servo_angle_1 = int(int(servo_angle_1 * 0.2) * 5);


  Servo1.write(servo_angle_1);
  //Serial.println(servo_angle_1);
  
  
}
