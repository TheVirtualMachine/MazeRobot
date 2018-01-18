/*******************************************************************************
* Copyright (C) Vincent Macri, David White, Kevin Du                           *
* Teacher: Mr. Wong                                                            *
* Due Date: 2018-01-17                                                         *
*                                                                              *
* This is an Arduino program that controls a robot which follows a black line. *
* It does this using LDR sensors to know which way to move.                    *
* The robot's motor is a Tamiya gearbox connected to an L293 motor chip.       *
* The robot is using tank treads to drive around.                              *
*                                                                              *
* Maze Robot is free software: you can redistribute it and/or modify           *
* it under the terms of the GNU General Public License as published by         *
* the Free Software Foundation, either version 3 of the License, or            *
* (at your option) any later version.                                          *
*                                                                              *
* Maze Robot is distributed in the hope that it will be useful,                *
* but WITHOUT ANY WARRANTY; without even the implied warranty of               *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the                 *
* GNU General Public License for more details.                                 *
*                                                                              *
* You should have received a copy of the GNU General Public License            *
* along with Maze Robot. If not, see <http://www.gnu.org/licenses/>.           *
*                                                                              *
* If you have any inquiries, contact us at wongtech@vincemacri.ca              *
********************************************************************************/

// Define LDR pins.
#define LEFT_LDR_PIN A2 // The left LDR pin.
#define RIGHT_LDR_PIN A1 // The right LDR pin.

// Define motor pin numbers.
#define LEFT_FORWARD 13 // The pin to move the left motor forwards.
#define LEFT_BACKWARD 12 // The pin to move the left motor backwards.
#define RIGHT_FORWARD 4 // The pin to move the right motor forwards.
#define RIGHT_BACKWARD 5 // The pin to move the right motor backwards.

// Variables for LDRs.
#define LEFT_LDR 0
#define RIGHT_LDR 1

// The number of LDRs.
#define LDR_COUNT 2

// The calibration threshold.
#define CALIBRATE_THRESHOLD 80

// The values of the LDRs.
int lightReadings[LDR_COUNT];

// The thresholds that marks the difference between black and white.
int midStart;
int midEnd;

// The state of where we are on the map.
int leftState;
int rightState;

// The state we were at on the last timer tick.
int previousState;

// How many ticks we have been on this state for.
int ticksOnState;

// How many ticks we have been on the left state for.
int ticksOnLeftState;

// How often to pulse the motor.
#define PULSE_THRESHOLD 15

// When to do a hard left.
#define HARD_LEFT_THRESHOLD 30

// When to prepare for a hard left.
#define HARD_LEFT_MARK 80

// Keep track of when to make a hard left turn.
bool makeHardLeft;

// The possible values of state.
#define ON_BLACK 0
#define ON_WHITE 1
#define ON_MID 2

// Define motor power amounts.
#define FULL_POWER HIGH // Analog value for full motor power.
#define NO_POWER LOW // Analog value for no motor power.

// Setup the program.
void setup() {
	Serial.begin(9600); // Start the serial communication.
	// Setup the motors.
	pinMode(LEFT_FORWARD, OUTPUT);
	pinMode(LEFT_BACKWARD, OUTPUT);
	pinMode(RIGHT_FORWARD, OUTPUT);
	pinMode(RIGHT_BACKWARD, OUTPUT);
	stop();
	delay(1000); // Give everything time to warm up.
	calibrateLDR();
	stop();
}

// Calibrate the LDR threshold.
void calibrateLDR() {
	int left;
	int right;
	int diff = CALIBRATE_THRESHOLD;
	while (diff <= CALIBRATE_THRESHOLD) {
		for (int i = 0; i < 25; i++) {
			readLDR();
			left += lightReadings[LEFT_LDR];
			right += lightReadings[RIGHT_LDR];
		}

		left /= 25;
		right /= 25;

		if (left > right) {
			diff = left - right;
		}
	}
	int lightThreshold = (left + right) / 2;
	midStart = lightThreshold - (lightThreshold * 0.1);
	midEnd = lightThreshold + (lightThreshold * 0.1);
}

// Read the LDR sensors.
void readLDR() {
	lightReadings[LEFT_LDR] = analogRead(LEFT_LDR_PIN);
	lightReadings[RIGHT_LDR] = analogRead(RIGHT_LDR_PIN);
}

// Compare the LDR readings to the light threshold, and update the LDR booleans accordingly.
void compareLDR() {
	int leftReading = lightReadings[LEFT_LDR];
	int rightReading = lightReadings[RIGHT_LDR];

	if (leftReading < midStart) {
		leftState = ON_WHITE;
	} else if (leftReading > midEnd) {
		leftState = ON_BLACK;
	} else {
		leftState = ON_MID;
	}

	if (rightReading < midStart) {
		rightState = ON_WHITE;
	} else if (rightReading > midEnd) {
		rightState = ON_BLACK;
	} else {
		rightState = ON_MID;
	}
}

// Move forward.
void forward() {
	leftMotorForward();
	rightMotorForward();
}

// Move backwards.
void backward() {
	leftMotorBackward();
	rightMotorBackward();
}

// Move the right motor forward.
void rightMotorForward() {
	digitalWrite(RIGHT_FORWARD, FULL_POWER);
	digitalWrite(RIGHT_BACKWARD, NO_POWER);
}

// Move the right motor backward.
void rightMotorBackward() {
	digitalWrite(RIGHT_FORWARD, NO_POWER);
	digitalWrite(RIGHT_BACKWARD, FULL_POWER);
}

// Move the left motor forward.
void leftMotorForward() {
	digitalWrite(LEFT_FORWARD, FULL_POWER);
	digitalWrite(LEFT_BACKWARD, NO_POWER);
}

// Move the left motor backward.
void leftMotorBackward() {
	digitalWrite(LEFT_FORWARD, NO_POWER);
	digitalWrite(LEFT_BACKWARD, FULL_POWER);
}

// Turn the left motor off.
void leftMotorOff() {
	digitalWrite(LEFT_FORWARD, NO_POWER);
	digitalWrite(LEFT_BACKWARD, NO_POWER);
}

// Turn the right motor off.
void rightMotorOff() {
	digitalWrite(RIGHT_FORWARD, NO_POWER);
	digitalWrite(RIGHT_BACKWARD, NO_POWER);
}

// Turn left.
void turnLeft() {
	leftMotorBackward();
	rightMotorForward();
}

// Turn right.
void turnRight() {
	leftMotorForward();
	rightMotorBackward();
}

// Stop the motors.
void stop() {
	leftMotorOff();
	rightMotorOff();
}

// Read sensors.
void sense() {
	readLDR();
	compareLDR();
}

// Pulse the forward motor.
void pulseForward() {
	if (ticksOnState > PULSE_THRESHOLD) {
		turnRight();
		delay(3);
		ticksOnState = 0;
		stop();
	} else {
		forward();
	}
}

// Veer to the right.
void veerRight() {
	turnRight();
	delay(10);
	forward();
	delay(5);
}

// Veer to the left.
void veerLeft() {
	turnLeft();
	delay(10);
	forward();
	delay(5);
}

// Do the line tracking.
void lineTrack() {
	if (rightState == ON_WHITE && leftState != ON_WHITE && previousState == ON_WHITE && ticksOnLeftState > PULSE_THRESHOLD && ticksOnLeftState < HARD_LEFT_MARK && ticksOnState > PULSE_THRESHOLD && ticksOnState < HARD_LEFT_MARK) {
		makeHardLeft = true;
		stop();
		delay(500);
	}

	if (leftState == previousState) {
		ticksOnState++;
	} else {
		ticksOnState = 0;
		ticksOnLeftState = 0;
	}

	Serial.print(leftState);
	Serial.print(" ");
	Serial.println(ticksOnState);

	if (leftState == ON_BLACK) {
		turnRight();
		ticksOnLeftState = 0;
		if (ticksOnState > PULSE_THRESHOLD) {
			delay(10);
		}
	} else if (leftState == ON_WHITE) {
		veerLeft();
		/*
		if (ticksOnLeftState > HARD_LEFT_THRESHOLD && makeHardLeft) {
			ticksOnState = 0;
			ticksOnLeftState = 0;
			makeHardLeft = false;
			turnLeft();
			delay(1200);
			forward();
			delay(300);
			turnRight();
			delay(1200);
			forward();
		} else
		*/
		if (ticksOnState > PULSE_THRESHOLD) {
			turnLeft();
			ticksOnLeftState++;
			delay(10);
		}
	} else {
		pulseForward();
		ticksOnLeftState = 0;
	}
}

void loop() {
	previousState = leftState;
	sense();
	lineTrack();
}
