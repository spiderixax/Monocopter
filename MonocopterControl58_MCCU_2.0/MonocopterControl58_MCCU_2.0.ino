#define rcPinX A3   // Connected to CH1 of Transmitter
#define rcPinY A2   // Connected to CH2 of transmitter
#define rcPinZ A1   // Connected to CH3 of transmitter
#define rcPinT A0
#define potentialDividerPin A7
#define buzzerPin 3
#define wingPin 7
#define motorPin 8
#define blueLedMCCU 4
#define redLedMCCU 10
#define blueLedWing 5
#define pinkLedWing 6
#include <Servo.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

#include "PinChangeInterrupt.h"

int sensedHeadings[] = {0,10,20,30,40,50,60,70,80,90,100,110,120,130,140,150,160,170,180,190,200,210,220,230,240,250,260,270,280,290,300,310,320,330,340,350,360};
int realHeadings[] =  {350,2,13,25,36,50,61,73,83,95,106,116,125,134,144,152,160,169,177,184,192,198,206,215,222,232,238,250,258,269,279,290,300,311,324,336,348};

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
float current_bearing;
float flight_magnitude = 0;
float x_vec = 0;
float y_vec = 0;
float z_vec = 0;

volatile int prev_time_x = 0;
volatile int prev_time_y = 0;
volatile int prev_time_z = 0;
volatile int prev_time_t = 0;

int default_angle_wing = 110;
int servo_angle_wing = default_angle_wing;

float flap_constant = 0.045;
float voltageReading = 0;
int cycleCounter = 0;
#define cycleDelay 60

bool rotation_reset = false;
unsigned long rotation_start_time;
float previous_last_rotation_time;
float last_rotation_time;
float predicted_rotation_time;
float current_rotation_progress;

void setup() {
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
  Motor.attach(wingPin);
  Motor.writeMicroseconds(z);

  if(!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    //Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
  }

  Serial.begin(9600);

  attachPCINT(digitalPinToPCINT(rcPinX), rising_x, RISING); //  PWM values from the receiver are read using pin change interrupts
  attachPCINT(digitalPinToPCINT(rcPinY), rising_y, RISING);
  attachPCINT(digitalPinToPCINT(rcPinZ), rising_z, RISING);
  attachPCINT(digitalPinToPCINT(rcPinT), rising_t, RISING);
  delay(500);

}

void loop() {

  if (cycleCounter == cycleDelay) {                                           //  A section of code executed every (cycleDelay)th cycle, 
    voltageReading = analogRead(potentialDividerPin)*0.01548;                                  //  here it checks if the battery voltage (measured by a
    //Serial.println(voltageReading);                                         //  potential divider) is within the safe range, if I
    if (voltageReading > 5.00 and voltageReading < 7.45 and z_vec > 60) {     //  want it to check (I use Channel T for this)
      digitalWrite(buzzerPin, HIGH);
    }
    else if (voltageReading > 10 and voltageReading < 11.25 and z_vec > 60) {
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

   // North-South indicator LEDs on the MCCU control board:
  /*if (current_bearing > 80 and current_bearing < 160) {
    //digitalWrite(blueLedMCCU, HIGH);
    PORTB = PORTB | B00000010;  //  Using direct port manipulation here to increase speed
  }

  else {
    //digitalWrite(blueLedMCCU, LOW);
    PORTB = PORTB & B11111101;
  }

  if (current_bearing > 260 and current_bearing < 340) {
    //digitalWrite(redLedMCCU, HIGH);
    PORTB = PORTB | B00000100;
  }

  else {
    //digitalWrite(redLedMCCU, LOW);
    PORTB = PORTB & B11111011;
  }*/


  //  North-South indicator LEDs on the tip of the wing:
  if (flight_magnitude > 50 and sin(0.01745329 * (current_bearing - flight_bearing) + 4.712389) < -0.9) {
    digitalWrite(blueLedWing, HIGH);
    digitalWrite(pinkLedWing, HIGH);
  }

  else if (flight_magnitude > 50 and sin(0.01745329 * (current_bearing - flight_bearing) + 4.712389) > 0.9) {
    digitalWrite(blueLedWing, LOW);
    digitalWrite(pinkLedWing, LOW);
  }
  
  else if ((current_bearing >= 0 and current_bearing < 90) or (current_bearing >= 270 and current_bearing <= 360)) {
    digitalWrite(blueLedWing, HIGH);
    digitalWrite(pinkLedWing, LOW);
  }

  else if (current_bearing >= 90 and current_bearing < 270) {
    digitalWrite(pinkLedWing, HIGH);
    digitalWrite(blueLedWing, LOW);
  }

  else {
    digitalWrite(blueLedWing, LOW);
    digitalWrite(pinkLedWing, LOW);
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

  Serial.println(flight_bearing);
  //Serial.println(flight_magnitude);

  if (flight_magnitude > 40) {
    servo_angle_wing = default_angle_wing - (flap_constant * flight_magnitude * sin(0.01745329 * (current_bearing - flight_bearing) + 3.14159));
  }

  else if (flight_magnitude <= 40) {
    servo_angle_wing = default_angle_wing;
  }

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
   
  int roundedHeading = localHeading / 10;
  roundedHeading = roundedHeading * 10;

  int indexOfRoundedHeading;

  for (int i=0; i<37; i++) {
   if (roundedHeading == sensedHeadings[i]) {
     indexOfRoundedHeading = i;
     break;
   }
  }

  int correctedHeading = realHeadings[indexOfRoundedHeading];
  
  return correctedHeading;
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
