#define rcPin1 8   // Pin 8 Connected to CH1 of Transmitter;
#define rcPin2 9   // Pin 9 Connected to CH2
#define rcPin3 10   // Flap
#include <Servo.h>
#include <QMC5883L.h>
#include <Wire.h>
QMC5883L compass;
Servo Servo1;
int flappingWing = 2;
int x = 0;  // Receiver Channel 1 PPM value
int y = 0;  // Receiver Channel 2 PPM value
int z = 0;
int x_mid = 1520;
int y_mid = 1500;
int z_mid = 1520;
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


void setup() {
  pinMode(rcPin1, INPUT);
  pinMode(rcPin2, INPUT);
  pinMode(rcPin3, INPUT);

  Servo1.attach(flappingWing);
  Wire.begin();
  compass.init();
  compass.setSamplingRate(200);

  //Serial.begin(9600);
}

void loop() {

  x = pulseIn(rcPin1, HIGH, 50000);  // (Pin, State, Timeout)
  y = pulseIn(rcPin2, HIGH, 50000);
  z = pulseIn(rcPin3, HIGH, 50000);

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

  
  if ((z_vec > 15) or (z_vec < -15)){
    neutral_angle_1 = default_angle_1 - (z_vec * 0.05);

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


  Servo1.write(servo_angle_1);
  
  
}
