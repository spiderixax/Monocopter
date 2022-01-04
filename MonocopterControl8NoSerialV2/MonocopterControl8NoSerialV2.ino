#define rcPin1 8   // Pin 8 Connected to CH1 of Transmitter;
#define rcPin2 9   // Pin 9 Connected to CH2
#define rcPin3 10   // Flap
#include <Servo.h>
#include <QMC5883L.h>
#include <Wire.h>
QMC5883L compass;
Servo Servo1;
int servoPin = 3;
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
float default_angle = 95;
float neutral_angle = default_angle;
float servo_angle = 90;
float flap_constant = 0.0005;

void setup() {
  pinMode(rcPin1, INPUT);
  pinMode(rcPin2, INPUT);
  pinMode(rcPin3, INPUT);

  Servo1.attach(servoPin); 

  Wire.begin();

  compass.init();
  compass.setSamplingRate(200);

}

void loop() {

// Read in the length of the signal in microseconds
  x = pulseIn(rcPin1, HIGH, 50000);  // (Pin, State, Timeout)
  y = pulseIn(rcPin2, HIGH, 50000);
  z = pulseIn(rcPin3, HIGH, 50000);

  x_vec = x_mid - x;
  y_vec = y_mid - y;
  z_vec = z_mid - z;

  
  
  if ((z_vec > 15) or (z_vec < -15)){
    neutral_angle = default_angle + (z_vec * 0.05);

  }

  else {
    neutral_angle = default_angle;
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

  
  int heading = compass.readHeading();
  if(heading==0) {
    /* Still calibrating, so measure but don't print */
  } else {

    current_bearing = heading;
  }

  if (flight_magnitude > 25) {
    servo_angle = neutral_angle + (flap_constant * flight_magnitude * 57.29578 * sin(0.01745329 * (current_bearing - flight_bearing) + 4.712389));

  }

  else if (flight_magnitude <= 25) {
    servo_angle = neutral_angle;
  }

  Servo1.write(servo_angle);
  
}
