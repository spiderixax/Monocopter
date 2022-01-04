// RC PulseIn Serial Read out By: Nick Poole

//extended By Jason Mclaughlin

//2015

#include <Servo.h> // for servo in example

Servo myservo; //servo entity

int pos = 0; //position for servo

int ch1; // Here's where we'll keep our channel values

int ch2;

int ch3;

void setup() { myservo.attach(9);

pinMode(5, INPUT); // Set our input pins as such

pinMode(6, INPUT); pinMode(7, INPUT);

Serial.begin(9600); // Pour a bowl of Serial

}

void loop() {

ch1 = pulseIn(5, HIGH, 25000); // Read the pulse width of

ch2 = pulseIn(6, HIGH, 25000); // each channel

ch3 = pulseIn(7, HIGH, 25000);

Serial.print("Channel 1:"); // Print the value of

Serial.println(ch1); // each channel

Serial.print("Channel 2:");

Serial.println(ch2);

Serial.print("Channel 3:");

Serial.println(ch3);

if ((ch3 >= 1500) && (ch3 <= 1600)){// the center postion for controller

Serial.println("between");

pos = 90;//set servo to center

myservo.write(pos); // set to pos which is 90

}

else{

Serial.println("not between");

for(pos = 0; pos < 180; pos += 1) // goes from 0 degrees to 180 degrees

{ // in steps of 1 degree

myservo.write(pos); // tell servo to go to position in variable 'pos'

delay(1); // waits 1ms for the servo to reach the position

}

for(pos = 180; pos>=1; pos-=1) // goes from 180 degrees to 0 degrees

{

myservo.write(pos); // tell servo to go to position in variable 'pos'

delay(1); // waits 1ms for the servo to reach the position

}

}

delay(1500); // I put this here just to make the terminal

// window happier

}
