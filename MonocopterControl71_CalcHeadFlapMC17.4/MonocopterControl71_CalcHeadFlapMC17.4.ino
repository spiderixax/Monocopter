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
#define blueLedWing 6
#define redkLedWing 5
#include <Servo.h>

#include <QMC5883L.h>
#include <Wire.h>

#include "PinChangeInterrupt.h"

QMC5883L compass;

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
#define safety_max 200

volatile int prev_time_x = 0;
volatile int prev_time_y = 0;
volatile int prev_time_z = 0;
volatile int prev_time_t = 0;

int default_angle_wing = 150; //90-somewhat works?, 80-too low, 110-too high, 100-unstable
int servo_angle_wing = default_angle_wing;

float flap_constant = 0.1;
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
  pinMode(redkLedWing, OUTPUT);
  ServoWing.attach(wingPin);
  ServoWing.write(default_angle_wing);
  Motor.attach(motorPin);
  Motor.writeMicroseconds(z);
  Wire.begin();
  compass.init();
  compass.setSamplingRate(200);
  //Serial.begin(9600);
  attachPCINT(digitalPinToPCINT(rcPinX), rising_x, RISING); //  PWM values from the receiver are read using pin change interrupts
  attachPCINT(digitalPinToPCINT(rcPinY), rising_y, RISING);
  attachPCINT(digitalPinToPCINT(rcPinZ), rising_z, RISING);
  attachPCINT(digitalPinToPCINT(rcPinT), rising_t, RISING);
  //digitalWrite(blueLedMCCU, HIGH);
  //digitalWrite(redLedMCCU, HIGH);
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

  //voltageReading = analogRead(A7)*0.01488;
  //Serial.println(voltageReading); 

  x_vec = x_mid - x;
  y_vec = y_mid - y;

  heading = compass.readHeading();
  //Serial.println(heading);

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

  safety = map(t, 1400, 1530, 0, 2);
  safety = map(safety, 0, 2, 0, safety_max);
  safety = constrain(safety, 0, safety_max);

  //  North-South indicator LEDs on the tip of the wing:
  if (current_bearing >= 90 and current_bearing < 270) {
    analogWrite(redkLedWing, 0);
    analogWrite(blueLedWing, safety);
  }
  else {
    analogWrite(redkLedWing, safety);
    analogWrite(blueLedWing, 0);
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
  //Serial.println(current_bearing);
  //Serial.println(t);
  if (flight_magnitude > 40) {
    servo_angle_wing = default_angle_wing - (flap_constant * flight_magnitude * sin(0.01745 * (current_bearing - flight_bearing) + 4.7124));
  }

  else if (flight_magnitude <= 40) {
    servo_angle_wing = default_angle_wing;
  }
  //z = map(z, 1000, 1700, 1000, 2000);
  //Serial.println(z);
  //delay(800);
  ServoWing.write(servo_angle_wing);
  Motor.writeMicroseconds(map(z, 1000, 1700, 950, 2000));

}

/*int correctedHeading(int toCorrect) {
  //toCorrect -= 0;
  if(toCorrect < 0)
    toCorrect += 360;
    
  // Check for wrap due to addition of declination.
  if(toCorrect > 360)
    toCorrect -= 360;

  return toCorrect;
}*/

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
  z = microsCorrected()-prev_time_z-40;
}
void rising_t() {
  attachPCINT(digitalPinToPCINT(rcPinT), falling_t, FALLING);
  prev_time_t = microsCorrected();
}
void falling_t() {
  attachPCINT(digitalPinToPCINT(rcPinT), rising_t, RISING);
  t = microsCorrected()-prev_time_t;
}
