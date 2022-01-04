#define rcPin1 8   // Pin 8 Connected to CH1 of Transmitter;
#define rcPin2 9   // Pin 9 Connected to CH2

int x = 0;  // Receiver Channel 1 PPM value
int y = 0;  // Receiver Channel 2 PPM value


int x_mid = 1520;
int y_mid = 1500;

int dir_vec = 0;

float flight_bearing;

float flight_magnitude = 0;

float x_vec = 0;
float y_vec = 0;


void setup() {
  pinMode(rcPin1, INPUT);
  pinMode(rcPin2, INPUT);
  Serial.begin(9600);
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

  Serial.print("X #1: ");
  Serial.println(x);
  Serial.print("Y #2: ");
  Serial.println(y); 
  Serial.println("Flight Bearing:");
  Serial.println(flight_bearing);
  Serial.println("Flight Magnitude:");
  Serial.println(flight_magnitude); 

  Serial.println("------------");
  delay(500);
  
}
