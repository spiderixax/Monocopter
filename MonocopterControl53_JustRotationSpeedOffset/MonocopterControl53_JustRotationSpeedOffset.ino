#define rcPinX A3   // Connected to CH1 of Transmitter
#define rcPinY A2   // Connected to CH2 of transmitter
#define rcPinZ A1   // Connected to CH3 of transmitter
#define rcPinT A0
// The motor is controlled directly from the receiver, independently of the Arduino
#define buzzerPin 5
#define motorPin 4
#define greenLedWing 9
#define redLedWing 10
//#include <Servo.h>
#include "ServoTimer2.h"
#include <QMC5883L.h>
#include <Wire.h>

#include "PinChangeInterrupt.h"

QMC5883L compass;
//Servo motor;
ServoTimer2 motor;
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
float current_bearing;
float flight_magnitude = 0;
float x_vec = 0;
float y_vec = 0;

int safety;


volatile int prev_time_x = 0;
volatile int prev_time_y = 0;
volatile int prev_time_z = 0;
volatile int prev_time_t = 0;

//int default_angle_wing = 70;
//int servo_angle_wing = default_angle_wing;
int motor_power;

float flap_constant = 0.6;
float voltageReading = 0;
int cycleCounter = 0;
#define cycleDelay 60


//float rotation_speed_offset = 1200;

//float rotation_speed_offset = 600;
//float flight_magnitude_offset = 0.02;

//float rotation_speed_offset = 1100;
//float flight_magnitude_offset = 0.005;

float rotation_speed_offset = 0;
float flight_magnitude_offset = 0;


bool rotation_reset = false;
unsigned long rotation_start_time;
float previous_last_rotation_time;
float last_rotation_time;
float predicted_rotation_time;
float current_rotation_progress;


void setup() {

  TCCR1B = TCCR1B & B11111000 | B00000001;    // set timer 1 divisor to     1 for PWM frequency of 31372.55 Hz
  
  pinMode(rcPinX, INPUT);
  pinMode(rcPinY, INPUT);
  pinMode(rcPinZ, INPUT);
  pinMode(buzzerPin, OUTPUT);
  //pinMode(greenLedWing, OUTPUT);
  //pinMode(redLedWing, OUTPUT);

  motor.attach(motorPin);
  motor.write(z_min);
  Wire.begin();
  compass.init();
  compass.setSamplingRate(200);

  //Serial.begin(9600);

  attachPCINT(digitalPinToPCINT(rcPinX), rising_x, RISING); //  PWM values from the receiver are read using pin change interrupts
  attachPCINT(digitalPinToPCINT(rcPinY), rising_y, RISING);
  attachPCINT(digitalPinToPCINT(rcPinZ), rising_z, RISING);
  attachPCINT(digitalPinToPCINT(rcPinT), rising_t, RISING);
  delay(500);

}

void loop() {

  if (cycleCounter == cycleDelay) {                                           //  A section of code executed every (cycleDelay)th cycle, 
    voltageReading = analogRead(A7)*0.01548;                                  //  here it checks if the battery voltage (measured by a
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

  heading = compass.readHeading();

  if (heading == 0) {
    
  }

  else if (heading >= 270 and heading <=360 and rotation_reset == true) {
    previous_last_rotation_time = last_rotation_time;
    last_rotation_time = millis() - rotation_start_time;
    rotation_start_time = millis();
    rotation_reset = false;
    //Serial.println("At 360");
  }

  else if (heading >= 90 and heading <= 180 and rotation_reset == false) {
    rotation_reset = true;
    //Serial.println("At 180");
  }

  else {}


  //predicted_rotation_time = float(float(last_rotation_time) * float(float(last_rotation_time) / float(previous_last_rotation_time)));
  //predicted_rotation_time = long(sq(last_rotation_time)) / previous_last_rotation_time; //long needed for sq
  predicted_rotation_time = last_rotation_time * (last_rotation_time / previous_last_rotation_time);

  current_rotation_progress = (float(millis() - rotation_start_time) / float(predicted_rotation_time));

  current_bearing = 360 - (360 * current_rotation_progress);

  current_bearing = constrain(current_bearing, 0, 360);
  

  //else {
    //current_bearing = heading;
  //}

  //Serial.println(current_bearing);

  safety = map(t, 1450, 1570, 0, 4);
  safety = map(safety, 0, 4, 0, 255);
  safety = constrain(safety, 0, 255);

  //  North-South indicator LEDs on the tip of the wing:
  if (current_bearing >= 0 and current_bearing < 180) {
    analogWrite(greenLedWing, safety);
    analogWrite(redLedWing, 0);
  }

  else if (current_bearing >= 180 and current_bearing <= 360) {
    analogWrite(redLedWing, safety);
    analogWrite(greenLedWing, 0);
  }

  else {
    analogWrite(greenLedWing, 0);
    analogWrite(redLedWing, 0);
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

  //Serial.println(flight_bearing);
  //Serial.println(flight_magnitude);

  if (flight_magnitude > 40) {
    //servo_angle_wing = default_angle_wing - (flap_constant * flight_magnitude * sin(0.01745329 * (current_bearing - flight_bearing) + 3.14159));
    motor_power = z - (flap_constant * flight_magnitude * sin(0.01745329 * (current_bearing - flight_bearing) + 4.71239 + (rotation_speed_offset / predicted_rotation_time) + (flight_magnitude * flight_magnitude_offset)));
  }

  else if (flight_magnitude <= 40) {
    //servo_angle_wing = default_angle_wing;
    motor_power = z;
  }

  //motor.writeMicroseconds(motor_power);
  motor.write(motor_power);
  //Serial.println(flight_magnitude);
}

// The pin change interrupt service routines
void rising_x() {
  attachPCINT(digitalPinToPCINT(rcPinX), falling_x, FALLING);
  prev_time_x = micros();
}
void falling_x() {
  attachPCINT(digitalPinToPCINT(rcPinX), rising_x, RISING);
  x = micros()-prev_time_x;
}
void rising_y() {
  attachPCINT(digitalPinToPCINT(rcPinY), falling_y, FALLING);
  prev_time_y = micros();
}
void falling_y() {
  attachPCINT(digitalPinToPCINT(rcPinY), rising_y, RISING);
  y = micros()-prev_time_y;
}
void rising_z() {
  attachPCINT(digitalPinToPCINT(rcPinZ), falling_z, FALLING);
  prev_time_z = micros();
}
void falling_z() {
  attachPCINT(digitalPinToPCINT(rcPinZ), rising_z, RISING);
  z = micros()-prev_time_z;
}
void rising_t() {
  attachPCINT(digitalPinToPCINT(rcPinT), falling_t, FALLING);
  prev_time_t = micros();
}
void falling_t() {
  attachPCINT(digitalPinToPCINT(rcPinT), rising_t, RISING);
  t = micros()-prev_time_t;
}
