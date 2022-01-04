#define rcPin1 8   // Pin 8 Connected to CH1 of Transmitter;
#define rcPin2 9   // Pin 9 Connected to CH2
#include <Servo.h>
#include <QMC5883L.h>
#include <Wire.h>


#include <IRremote.h>
#include <EEPROM.h>
#define irPin 10
IRrecv irrecv(irPin);
decode_results results;


QMC5883L compass;
Servo Servo1;
int servoPin = 3;
int x = 0;  // Receiver Channel 1 PPM value
int y = 0;  // Receiver Channel 2 PPM value

int x_mid = 1520;
int y_mid = 1500;
float flight_bearing;
float current_bearing;
float flight_magnitude = 0;
float x_vec = 0;
float y_vec = 0;
float neutral_angle = 100;
float servo_angle = 90;
float flap_constant = 7;

void setup() {
  pinMode(rcPin1, INPUT);
  pinMode(rcPin2, INPUT);
  Serial.begin(9600);

  Servo1.attach(servoPin); 


  irrecv.enableIRIn();

  flap_constant = EEPROM.read(0);
  neutral_angle = EEPROM.read(1);
  flap_constant = EEPROM.read(0);
  neutral_angle = EEPROM.read(1);

  

  Wire.begin();

  compass.init();
  compass.setSamplingRate(50);

  Serial.println("QMC5883L Compass Demo");
  Serial.println("Turn compass in all directions to calibrate....");
}

void loop() {

// Read in the length of the signal in microseconds
  x = pulseIn(rcPin1, HIGH, 50000);  // (Pin, State, Timeout)
  y = pulseIn(rcPin2, HIGH, 50000);

  x_vec = x_mid - x;
  y_vec = y_mid - y;


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

  if (flight_magnitude > 15) {
    servo_angle = neutral_angle + (flap_constant * 0.0001 * flight_magnitude * 57.29578 * sin(0.01745329 * (current_bearing - flight_bearing) + 4.712389));

  }

  else if (flight_magnitude <= 15) {
    servo_angle = neutral_angle;
  }

  Servo1.write(servo_angle); 
  
  Serial.print("X #1: ");
  Serial.println(x);
  Serial.print("Y #2: ");
  Serial.println(y); 
  Serial.println("Flight Bearing:");
  Serial.println(flight_bearing);
  Serial.println("Flight Magnitude:");
  Serial.println(flight_magnitude); 

  Serial.println("Current Bearing:");
  Serial.println(current_bearing); 

  Serial.println("Servo Angle");
  Serial.println(servo_angle); 

  Serial.println("------------");
  delay(500);
  
}
