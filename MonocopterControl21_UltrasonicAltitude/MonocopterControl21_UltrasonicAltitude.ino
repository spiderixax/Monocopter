#define rcPinX 8   // Pin 8 Connected to CH1 of Transmitter;
#define rcPinY 9   // Pin 9 Connected to CH2
#define rcPinZ 10   // Flap
#define rcPinThrottle 13  // Throttle
#define buzzerPin 7
#include <Servo.h>
#include <QMC5883L.h>
#include <Wire.h>
#include <PID_v1.h>
QMC5883L compass;
Servo Servo1;
Servo Motor;
#define flappingWing 2
#define motor 6
int x_mid = 1520;
int y_mid = 1500;
int z_mid = 1520;
int throttle_min = 1060;
int x = x_mid;  // Receiver Channel 2 PPM value
int y = y_mid;  // Receiver Channel 1 PPM value
int z = z_mid;
int throttle = 0;
float flight_bearing;
float current_bearing;
float flight_magnitude = 0;
float x_vec = 0;
float y_vec = 0;
float z_vec = 0;
float default_angle_1 = 100;
float neutral_angle_1 = default_angle_1;
float servo_angle_1 = default_angle_1;
int current_servo_angle = default_angle_1;
float flap_constant = 0.002;
float voltage_reading = 0;
int cycle_counter = 0;
#define cycle_delay 1

#define trigPin 11
#define echoPin 12
double duration, altitude_cm;
double desired_altitude = 0;
double motor_power = 0;

double Kp = 0;
double Ki = 0;
double Kd = 0;

PID myPID(&altitude_cm, &motor_power, &desired_altitude, 1, 1, 1, DIRECT);

void setup() {
  pinMode(rcPinX, INPUT);
  pinMode(rcPinY, INPUT);
  pinMode(rcPinZ, INPUT);
  pinMode(rcPinThrottle, INPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  Servo1.attach(flappingWing);
  Motor.attach(motor);
  Wire.begin();
  compass.init();
  compass.setSamplingRate(200);
  
  Kp = (analogRead(A0) - 20) * 0.0125;
  Ki = (analogRead(A1) - 8) * 0.0125;
  Kd = (analogRead(A2) - 10) * 0.0125;
  myPID.SetTunings(Kp, Ki, Kd);
  myPID.SetMode(AUTOMATIC);

  //Serial.begin(9600);
  
  //Serial.println(Kp);
  //Serial.println(Ki);
  //Serial.println(Kd);
  //delay(2000);
}



void loop() {
  //Serial.println(analogRead(A0));
  //Serial.println(analogRead(A1));
  //Serial.println(analogRead(A2));
  //Serial.println("");
  //delay(1000);

  if (cycle_counter == cycle_delay) {
    x = pulseIn(rcPinX, HIGH, 50000);  // (Pin, State, Timeout)
    y = pulseIn(rcPinY, HIGH, 50000);
    z = pulseIn(rcPinZ, HIGH, 50000);
    throttle = pulseIn(rcPinThrottle, HIGH, 50000);

    desired_altitude = (throttle - throttle_min) * 0.4348;
    if (desired_altitude < 0) {
      desired_altitude = 0;
    }

    myPID.Compute();
    if (motor_power < 40) {
      motor_power = 40;
    }
    if (motor_power > 160) {
      motor_power = 160;
    }
    if (desired_altitude < 10) {
      motor_power = 40;
    }
      
    //Serial.println(motor_power);
    Motor.write(motor_power);
    //Motor.write(desired_altitude * 0.6);
    //Serial.println(desired_altitude * 0.6);
    //delay(1000);

    
    //Serial.println(desired_altitude);
    voltage_reading = analogRead(A7)*0.00756;
    if (voltage_reading > 4.00 and voltage_reading < 7.45) {
      digitalWrite(buzzerPin, HIGH);
    }
    else {
      digitalWrite(buzzerPin, LOW);
    }


    digitalWrite(trigPin, LOW);
    delayMicroseconds(5);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH, 11000);
    altitude_cm = (duration/2) / 29.1;
    //Serial.println(altitude_cm);




    
    cycle_counter = 0;
  }

  else {
    cycle_counter++;
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

  //Serial.println(flight_bearing);
  //Serial.println(flight_magnitude);



  if (flight_magnitude > 40) {
    servo_angle_1 = neutral_angle_1 - (flap_constant * flight_magnitude * 57.29578 * sin(0.01745329 * (current_bearing - flight_bearing) + 4.712389));
  }

  else if (flight_magnitude <= 40) {
    servo_angle_1 = neutral_angle_1;
  }

  //servo_angle_1 = int(int(servo_angle_1 * 0.2) * 5);

  //int servo_update_difference = ((int(current_servo_angle - servo_angle_1)) ^ 2);
  int servo_update_difference = current_servo_angle - servo_angle_1;

  if (servo_update_difference > 5 or servo_update_difference < -5) {
    Servo1.write(servo_angle_1);
    current_servo_angle = servo_angle_1;
  }

  else {}
  
  //Serial.println(servo_update_difference);
  
  
}
