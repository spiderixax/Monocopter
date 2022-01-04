#define rcPin1 A3   // Pin 8 Connected to CH1 of Transmitter;
#define rcPin2 A2   // Pin 9 Connected to CH2
#define rcPin3 A1   // Wing
#define buzzerPin 5
#define flapPin 2
#define wingPin 3
#include <Servo.h>
#include <QMC5883L.h>
#include <Wire.h>
QMC5883L compass;
Servo ServoFlap;
Servo ServoWing;
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

float default_angle_flap = 95;
float servo_angle_flap = default_angle_flap;
int currentFlapAngle = default_angle_flap;
int flapUpdateDifference;

float default_angle_wing = 70;
float servo_angle_wing = default_angle_wing;
int currentWingAngle = default_angle_wing;
int wingUpdateDifference;

float flap_constant = 0.002;
float voltageReading = 0;
int cycleCounter = 0;
#define cycleDelay 40


void setup() {
  pinMode(rcPin1, INPUT);
  pinMode(rcPin2, INPUT);
  pinMode(rcPin3, INPUT);
  pinMode(13, INPUT);
  pinMode(buzzerPin, OUTPUT);

  ServoFlap.attach(flapPin);
  ServoFlap.write(default_angle_flap);
  ServoWing.attach(wingPin);
  ServoWing.write(default_angle_wing);
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
    voltageReading = analogRead(A7)*0.00983;
    //Serial.println(voltageReading);
    if (voltageReading > 5.00 and voltageReading < 7.45) {
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

  
  servo_angle_wing = default_angle_wing + (z_vec * 0.08);


  wingUpdateDifference = currentWingAngle - servo_angle_wing;

  if (wingUpdateDifference > 3 or wingUpdateDifference < -3) {
    ServoWing.write(servo_angle_wing);
    currentWingAngle = servo_angle_wing;
  }

  else {}


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
    servo_angle_flap = default_angle_flap - (flap_constant * flight_magnitude * 57.29578 * sin(0.01745329 * (current_bearing - flight_bearing) + 4.712389));
  }

  else if (flight_magnitude <= 40) {
    servo_angle_flap = default_angle_flap;
  }

  //servo_angle_flap = int(int(servo_angle_flap * 0.2) * 5);

  //int flapUpdateDifference = ((int(currentFlapAngle - servo_angle_flap)) ^ 2);
  flapUpdateDifference = currentFlapAngle - servo_angle_flap;

  if (flapUpdateDifference > 3 or flapUpdateDifference < -3) {
    ServoFlap.write(servo_angle_flap);
    currentFlapAngle = servo_angle_flap;
  }

  else {}
  
  //Serial.println(flapUpdateDifference);
  
}
