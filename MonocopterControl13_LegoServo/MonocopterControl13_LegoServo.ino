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
float default_angle = 30;
float neutral_angle = default_angle;
float servo_angle = 90;
float flap_constant = 0.08;

bool flyDirection = false;

void setup() {
  pinMode(rcPin1, INPUT);
  pinMode(rcPin2, INPUT);
  pinMode(rcPin3, INPUT);

  Servo1.attach(servoPin); 
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

  //Serial.println("1");

  int heading = compass.readHeading();
  //Serial.println("1.5");
  if(heading==0) {
    /* Still calibrating, so measure but don't print */
    
  }
  else {
    current_bearing = heading;
  }

  //Serial.println("2");

  
  if ((z_vec > 15) or (z_vec < -15)){
    neutral_angle = default_angle + (z_vec * 0.08);

  }

  else {
    neutral_angle = default_angle;
  }

  //Serial.println("3");


  flight_magnitude = sqrt(sq(x_vec) + sq(y_vec));


  if ((x_vec >= 0) && (y_vec >= 0)){
    flight_bearing = 90 - (57.29578 * atan(y_vec / x_vec));

    
    if (0 <= current_bearing and current_bearing <= flight_bearing + 90) {
      flyDirection = true;
    }

    else if (270 + flight_bearing <= current_bearing and current_bearing <= 360) {
      flyDirection = true;
    }

    else {
      flyDirection = false;
    }

  }
  

  else if ((x_vec >= 0) && (y_vec < 0)){
    flight_bearing = 90 + (57.29578 * atan(y_vec / - x_vec));

    if (flight_bearing - 45 <= current_bearing and current_bearing <= flight_bearing + 45) {
      flyDirection = true;
    }

    else {
      flyDirection = false;
    }

  }

  else if ((x_vec < 0) && (y_vec < 0)){
    flight_bearing = 270 - (57.29578 * atan(y_vec / x_vec));

    if (flight_bearing - 45 <= current_bearing and current_bearing <= flight_bearing + 45) {
      flyDirection = true;
    }
    
    else {
      flyDirection = false;
    }

  }

  else if ((x_vec < 0) && (y_vec >= 0)){
    flight_bearing = 270 + (57.29578 * atan(y_vec / - x_vec));

    if (flight_bearing - 45 <= current_bearing and current_bearing <= 360) {
      flyDirection = true;
    }

    else if (0 <= current_bearing and current_bearing <= 360 - flight_bearing) {
      flyDirection = true;
    }

    else {
      flyDirection = false;
    }


  }

  

  if (flight_magnitude > 25) {
    //servo_angle = neutral_angle + (flap_constant * flight_magnitude * 57.29578 * sin(0.01745329 * (current_bearing - flight_bearing) + 4.712389));


    if (flyDirection == true) {
      servo_angle = neutral_angle + flap_constant * flight_magnitude; // changed sign
    }

    else if (flyDirection == false) {
      servo_angle = neutral_angle - flap_constant * flight_magnitude; // changed sign
    }


    
  }

  else if (flight_magnitude <= 25) {
    servo_angle = neutral_angle;
  }

  Servo1.write(180 - servo_angle);
  //Serial.println(180 - servo_angle);
  //Serial.println(current_bearing);
  //Serial.println(flight_bearing);
  //Serial.println("");
  
}
