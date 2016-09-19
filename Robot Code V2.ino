/***************************************************
  This sketch is what drives the simple object 
  detection robot found at the following url: 
  http://www.instructables.com/id/Simple-3D-Printed-Arduino-Robot/
  It is based off of an example sketch detailed in
  the block comment below.

  Andrew Bevelhymer
  9-19-16
*****************************************************/

/*************************************************** 
  This is an example for our Adafruit 16-channel PWM & Servo driver
  Servo test - this will drive 16 servos, one after the other

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815

  These displays use I2C to communicate, 2 pins are required to  
  interface. For Arduino UNOs, thats SCL -> Analog 5, SDA -> Analog 4

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define RIGHTMIN  590 // Minimum pulse length amount for right servo
#define RIGHTMAX  150 // Maximum pulse length amount for right servo
#define RIGHTSTOP 370 // "Middle" pulse length (stops servo from spinning) for right servo
#define LEFTMIN 150	// Minimum pulse length amount for left servo
#define LEFTMAX 590 // Maximum pulse length amount for left servo
#define LEFTSTOP 370 // "Middle" pulse length (stops servo from spinning) for left servo

// our servo # counter
uint8_t RIGHT = 0; // Right servo
uint8_t LEFT = 1; // Left servo

int trigpin = 13; // Trigger pin for ultrasonic sensor
int echopin = 12; // Echo pin for ultrasonic sensor

void setup() {
  Serial.begin(9600);
  Serial.println("16 channel Servo test!");

  pwm.begin();
  
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  pinMode(trigpin, OUTPUT);
  pinMode(echopin, INPUT);
}

// you can use this function if you'd like to set the pulse length in seconds
// e.g. setServoPulse(0, 0.001) is a ~1 millisecond pulse width. its not precise!
void setServoPulse(uint8_t n, double pulse) {
  double pulselength;
  
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= 60;   // 60 Hz
  Serial.print(pulselength); Serial.println(" us per period"); 
  pulselength /= 4096;  // 12 bits of resolution
  Serial.print(pulselength); Serial.println(" us per bit"); 
  pulse *= 1000;
  pulse /= pulselength;
  Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}


/*
	Below, I have pre-defined several functions that when called will make the robot do common tasks
	such as going forward, left, right, and reverse.
*/
int goForward () {
	pwm.setPWM(RIGHT, 0, RIGHTMAX);
	pwm.setPWM(LEFT, 0, LEFTMAX);
}

int goReverse () {
	pwm.setPWM(RIGHT, 0, RIGHTMIN);
	pwm.setPWM(LEFT, 0, LEFTMIN);
}

int turnRight () {
	pwm.setPWM(RIGHT, 0, RIGHTMIN);
	pwm.setPWM(LEFT, 0, LEFTMAX);
}

int turnLeft () {
	pwm.setPWM(RIGHT, 0, RIGHTMAX);
	pwm.setPWM(LEFT, 0, LEFTMIN);
}

int turnLeftOneWheel () {
	pwm.setPWM(RIGHT, 0, RIGHTMAX);
	pwm.setPWM(LEFT, 0, LEFTSTOP);
}

int turnRightOneWheel () {
	pwm.setPWM(RIGHT, 0, RIGHTSTOP);
	pwm.setPWM(LEFT, 0, LEFTMAX);
}


/*
	In the loop function, just uncomment the function you would like the 
	robot to do. The functions and their descriptions are listed below the
	loop function.
*/
void loop() {
  //dance();
  objAvoid();
  //lineFollow();
}

/*
	This "dance" function simply makes the robot turn left, right, 
	reverse, go forward, reverse, and go forward again. This function
	does not make use of the sensors and is mostly just a test to make
	sure the servos work. i.e. the robot can move in all directions.
*/
void dance () {
	turnLeft();
	delay(1500);
	turnRight();
	delay(2000);
	goReverse();
	delay(500);
	goForward();
	delay(500);
	goReverse();
	delay(500);
	goForward();
	delay(500);
}

/*
	This "objAvoid" function, when called, turns the robot in to an 
	object avoidance robot by making use of the ultrasonic sensor
	as well as the custom infrared optical reflective sensors.
	(detail on sensors in instructable)
*/
void tableSense() {
	
	// Setting up ultrasonic sensor and getting distance
  	long duration, distance;
  	digitalWrite(trigpin, LOW); 
  	delayMicroseconds(2); 
  	digitalWrite(trigpin, HIGH);
  	delayMicroseconds(10);
  	digitalWrite(trigpin, LOW);
  	duration = pulseIn(echopin, HIGH);
  	distance = (duration/2) / 29.1;
	
	// Getting Analog values from IR sensors
	int leftRead = analogRead(A1);
	int rightRead = analogRead(A2);
	
	// Nothing under IR sensors
	if (leftRead > 800 || rightRead > 800) {
		turnLeft();
		delay(300);
	}
	// object within 4 cm of ultrasonic sensor
	if (distance < 4) {
		turnLeft();
		delay(300);
	}
	else {
		goForward();
	}
}


/*
	This function turns the robot in to a line following robot using only
	the 2 IR sensors.
*/
void lineFollow () {
	int leftRead = analogRead(A1);
	int rightRead = analogRead(A2);
	if (leftRead > rightRead) {
		turnRightOneWheel();
	}
	else {
		turnLeftOneWheel();
	}
}

