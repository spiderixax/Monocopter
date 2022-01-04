#define rcPinX A3   // Connected to CH1 of Transmitter
#define rcPinY A2   // Connected to CH2 of transmitter
#define rcPinZ A1   // Connected to CH3 of transmitter
#define rcPinT A0
#define potentialDividerPin A7
#define buzzerPin 3
#define wingPin 7
#define motorPin 8
#define blueLedMCCU 2
#define redLedMCCU 4
#define blueLedWing 5
#define pinkLedWing 6
#include <Servo.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

#include "PinChangeInterrupt.h"

/* Assign a unique ID to this sensor at the same time */
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

Servo ServoWing;
Servo Motor;
int x_mid = 1520; // Check the middle value on your transmitter
int y_mid = 1500;
int z_min = 1100;
int t_min = 1100;
int heading;
volatile int x = x_mid;
volatile int y = y_mid;
volatile int z = z_min;
volatile int t = t_min;
float flight_bearing;
float current_bearing = 0;
float flight_magnitude = 0;
float x_vec = 0;
float y_vec = 0;
float z_vec = 0;

int safety;

volatile int prev_time_x = 0;
volatile int prev_time_y = 0;
volatile int prev_time_z = 0;
volatile int prev_time_t = 0;

int default_angle_wing = 90;
int servo_angle_wing = default_angle_wing;

float flap_constant = 0.045;
float voltageReading = 0;
int cycleCounter = 0;
#define cycleDelay 60

bool rotation_reset = false;
unsigned long rotation_start_time = 1;
float previous_last_rotation_time = 1;
float last_rotation_time = 1;
float predicted_rotation_time = 1;
float current_rotation_progress = 1;

void setup() {
  TCCR0B = TCCR0B & B11111000 | B00000001;    // set timer 0 divisor to     1 for PWM frequency of 62500.00 Hz
  pinMode(rcPinX, INPUT);
  pinMode(rcPinY, INPUT);
  pinMode(rcPinZ, INPUT);
  pinMode(rcPinT, INPUT);
  
  pinMode(buzzerPin, OUTPUT);
  pinMode(blueLedMCCU, OUTPUT);
  pinMode(redLedMCCU, OUTPUT);
  pinMode(blueLedWing, OUTPUT);
  pinMode(pinkLedWing, OUTPUT);

  ServoWing.attach(wingPin);
  ServoWing.write(default_angle_wing);
  Motor.attach(motorPin);
  Motor.writeMicroseconds(z);

  if(!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    //Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
  }

  //Serial.begin(9600);

  attachPCINT(digitalPinToPCINT(rcPinX), rising_x, RISING); //  PWM values from the receiver are read using pin change interrupts
  attachPCINT(digitalPinToPCINT(rcPinY), rising_y, RISING);
  attachPCINT(digitalPinToPCINT(rcPinZ), rising_z, RISING);
  attachPCINT(digitalPinToPCINT(rcPinT), rising_t, RISING);
  digitalWrite(blueLedMCCU, HIGH);
  digitalWrite(redLedMCCU, HIGH);
  delay(500);

}

void loop() {

  if (cycleCounter == cycleDelay) {                                           //  A section of code executed every (cycleDelay)th cycle, 
    voltageReading = analogRead(A7)*0.01488;                                  //  here it checks if the battery voltage (measured by a
    //Serial.println(voltageReading);                                         //  potential divider) is within the safe range, if I
    if (voltageReading > 5.00 and voltageReading < 7.45 and t < 1200) {     //  want it to check (I use Channel T for this)
      digitalWrite(buzzerPin, HIGH);
    }
    else if (voltageReading > 10 and voltageReading < 11.25 and t < 1200) {
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

  heading = getHeading();

  if (heading >= 270 and heading <=360 and rotation_reset == true) {
    previous_last_rotation_time = last_rotation_time;
    last_rotation_time = millisCorrected() - rotation_start_time;
    rotation_start_time = millisCorrected();
    rotation_reset = false;
    //Serial.println("At 360");
  }

  else if (heading >= 45 and heading <= 255 and rotation_reset == false) {
    rotation_reset = true;
    //Serial.println("At 180");
  }

  else {}

  //predicted_rotation_time = float(float(last_rotation_time) * float(float(last_rotation_time) / float(previous_last_rotation_time)));
  //predicted_rotation_time = long(sq(last_rotation_time)) / previous_last_rotation_time; //long needed for sq
  predicted_rotation_time = last_rotation_time * (last_rotation_time / previous_last_rotation_time);

  current_rotation_progress = (float(millisCorrected() - rotation_start_time) / float(predicted_rotation_time));

  current_bearing = 360 - (360 * current_rotation_progress);

  current_bearing = constrain(current_bearing, 0, 360);


   // North-South indicator LEDs on the MCCU control board:
  if ((current_bearing >= 0 and current_bearing < 20) or (current_bearing >= 340 and current_bearing <= 360)) {
    digitalWrite(redLedMCCU, HIGH);
  }

  else {
    digitalWrite(redLedMCCU, LOW);
  }

  if (current_bearing > 160 and current_bearing < 200) {
    digitalWrite(blueLedMCCU, HIGH);
  }

  else {
    digitalWrite(blueLedMCCU, LOW);
  }

  safety = map(t, 1450, 1570, 0, 3);
  safety = map(safety, 0, 3, 0, 255);
  safety = constrain(safety, 0, 255);


  //  North-South indicator LEDs on the tip of the wing:
  if (flight_magnitude > 50 and sin(0.01745329 * (current_bearing - flight_bearing) + 4.712389) < -0.9) {
    analogWrite(blueLedWing, safety);
    analogWrite(pinkLedWing, safety);
  }

  else if (flight_magnitude > 50 and sin(0.01745329 * (current_bearing - flight_bearing) + 4.712389) > 0.9) {
    analogWrite(blueLedWing, 0);
    analogWrite(pinkLedWing, 0);
  }
  
  else if ((current_bearing >= 0 and current_bearing < 90) or (current_bearing >= 270 and current_bearing <= 360)) {
    analogWrite(blueLedWing, safety);
    analogWrite(pinkLedWing, 0);
  }

  else if (current_bearing >= 90 and current_bearing < 270) {
    analogWrite(pinkLedWing, safety);
    analogWrite(blueLedWing, 0);
  }

  else {
    analogWrite(blueLedWing, 0);
    analogWrite(pinkLedWing, 0);
  }


  flight_magnitude = sqrt(sq(x_vec) + sq(y_vec));
  flight_magnitude = constrain(flight_magnitude, 0, 360);
  //  To make sure that moving the stick on the transmitter diagonally does not result in a greater flight_magnitude


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

  //Serial.println(z);
  

  if (flight_magnitude > 40) {
    servo_angle_wing = default_angle_wing - (flap_constant * flight_magnitude * sin(0.01745329 * (current_bearing - flight_bearing) + 3.14159));
  }

  else if (flight_magnitude <= 40) {
    servo_angle_wing = default_angle_wing;
  }
  //Serial.println(heading);
  ServoWing.write(servo_angle_wing);
  Motor.writeMicroseconds(z);

}

int getHeading() {
  sensors_event_t event; 
  mag.getEvent(&event);

  //float heading = atan2(event.magnetic.y, event.magnetic.x);
  int localHeading = atan2(event.magnetic.y, event.magnetic.x) * 180/M_PI; 
  
  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  int declinationAngle = 6;
  localHeading += declinationAngle;
  
  // Correct for when signs are reversed.
  if(localHeading < 0)
    localHeading += 360;
    
  // Check for wrap due to addition of declination.
  if(localHeading > 360)
    localHeading -= 360;
  
  return localHeading;
}

unsigned long millisCorrected() {
  return millis() / 64;
}

unsigned long microsCorrected() {
  return micros() / 64;
}

// The pin change interrupt service routines
void rising_x() {
  attachPCINT(digitalPinToPCINT(rcPinX), falling_x, FALLING);
  prev_time_x = microsCorrected();
}
void falling_x() {
  attachPCINT(digitalPinToPCINT(rcPinX), rising_x, RISING);
  x = microsCorrected()-prev_time_x;
}
void rising_y() {
  attachPCINT(digitalPinToPCINT(rcPinY), falling_y, FALLING);
  prev_time_y = microsCorrected();
}
void falling_y() {
  attachPCINT(digitalPinToPCINT(rcPinY), rising_y, RISING);
  y = microsCorrected()-prev_time_y;
}
void rising_z() {
  attachPCINT(digitalPinToPCINT(rcPinZ), falling_z, FALLING);
  prev_time_z = microsCorrected();
}
void falling_z() {
  attachPCINT(digitalPinToPCINT(rcPinZ), rising_z, RISING);
  z = microsCorrected()-prev_time_z;
}
void rising_t() {
  attachPCINT(digitalPinToPCINT(rcPinT), falling_t, FALLING);
  prev_time_t = microsCorrected();
}
void falling_t() {
  attachPCINT(digitalPinToPCINT(rcPinT), rising_t, RISING);
  t = microsCorrected()-prev_time_t;
}
