#define rcPin1 8   // Pin 8 Connected to CH1 of Transmitter;
#define rcPin2 9   // Pin 9 Connected to CH2
#define rcPin3 10   // Flap
#define buzzerPin 7
#define flapPin 2
//#define wingPin 3
#include <Servo.h>
#include <QMC5883L.h>
#include <Wire.h>
QMC5883L compass;
Servo ServoFlap;
int x_mid = 1520;
int y_mid = 1500;
//int z_mid = 1520;
int x = x_mid;  // Receiver Channel 2 PPM value
int y = y_mid;  // Receiver Channel 1 PPM value
//int z = z_mid;
float flight_bearing;
float current_bearing;
float flight_magnitude = 0;
float x_vec = 0;
float y_vec = 0;
//float z_vec = 0;
float default_angle_flap = 105;
float servo_angle_flap = default_angle_flap;
int currentFlapAngle = default_angle_flap;
int flapUpdateDifference;

float flap_constant = 0.002;
float voltageReading = 0;
int cycleCounter = 0;
#define cycleDelay 50


void setup() {
  pinMode(rcPin1, INPUT);
  pinMode(rcPin2, INPUT);
  pinMode(rcPin3, INPUT);
  pinMode(13, INPUT);
  pinMode(buzzerPin, OUTPUT);

  ServoFlap.attach(flapPin);
  ServoFlap.write(default_angle_flap);
  Wire.begin();
  compass.init();
  compass.setSamplingRate(200);

  Serial.begin(9600);
}

void loop() {

  if (cycleCounter == cycleDelay) {
    x = pulseIn(rcPin1, HIGH, 50000);  // (Pin, State, Timeout)
    y = pulseIn(rcPin2, HIGH, 50000);
    //z = pulseIn(rcPin3, HIGH, 50000);
    voltageReading = analogRead(A7)*0.00756;
    if (voltageReading > 4.00 and voltageReading < 7.45) {
      digitalWrite(buzzerPin, HIGH);
    }
    else {
      digitalWrite(buzzerPin, LOW);
    }
    cycleCounter = 0;
  }

  else {
    cycleCounter++;
  }

  x_vec = x_mid - x;
  y_vec = y_mid - y;
  //z_vec = z_mid - z;

  int heading = compass.readHeading();
  
  if(heading==0) {
    /* Still calibrating, so measure but don't print */
  }
  else {
    current_bearing = heading;
  }

  //Serial.println(current_bearing);

  current_bearing = int(int(current_bearing * 0.2) * 5);


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
    servo_angle_flap = default_angle_flap - (flap_constant * flight_magnitude * 57.29578 * sin(0.01745329 * (current_bearing - flight_bearing) + 4.712389 + 3.141593));
  }

  else if (flight_magnitude <= 40) {
    servo_angle_flap = default_angle_flap;
  }

  //servo_angle_flap = int(int(servo_angle_flap * 0.2) * 5);

  //int flapUpdateDifference = ((int(currentFlapAngle - servo_angle_flap)) ^ 2);
  flapUpdateDifference = currentFlapAngle - servo_angle_flap;

  if (flapUpdateDifference > 2 or flapUpdateDifference < -2) {
    ServoFlap.write(servo_angle_flap);
    currentFlapAngle = servo_angle_flap;
  }

  else {}
  
  //Serial.println(flapUpdateDifference);
  
  
}
