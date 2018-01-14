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

#include "sounds.h"

// Define LDR pins.
#define LEFT_LDR_PIN A2 // The left LDR pin.
#define RIGHT_LDR_PIN A1 // The right LDR pin.

// Define motor pin numbers.
#define LEFT_FORWARD 13 // The pin to move the left motor forwards.
#define LEFT_BACKWARD 12 // The pin to move the left motor backwards.
#define RIGHT_FORWARD 4 // The pin to move the right motor forwards.
#define RIGHT_BACKWARD 5 // The pin to move the right motor backwards.

// Define miscellaneous pins.
#define SPEAKER 3 // The pin for the speaker.

// Variables for LDRs.
#define LEFT_LDR 0
#define RIGHT_LDR 1

// The number of LDRs.
#define LDR_COUNT 2

// The calibration threshold.
#define CALIBRATE_THRESHOLD 150

// The values of the LDRs.
int lightReadings[LDR_COUNT];

// If the LDRs are on black.
bool isOnBlack[LDR_COUNT];

// The threshold that marks the difference between black and white.
int lightThreshold;

// Define motor power amounts.
#define FULL_POWER HIGH // Analog value for full motor power.
#define NO_POWER LOW // Analog value for no motor power.

// Keep track of oscillation.
#define OSCILLATION_THRESHOLD 100 // The threshold at which to trigger an oscillation correction.
int leftOscillationCount; // The number of times we have moved left recently.
int rightOscillationCount; // The number of times we have moved right recently.

// Setup the program.
void setup() {
	Serial.begin(9600); // Start the serial communication.
	Serial.println("Starting program.");
	// Setup the motors.
	pinMode(LEFT_FORWARD, OUTPUT);
	pinMode(LEFT_BACKWARD, OUTPUT);
	pinMode(RIGHT_FORWARD, OUTPUT);
	pinMode(RIGHT_BACKWARD, OUTPUT);
	stop();
	delay(100); // Give everything time to warm up.
	stop();
	delay(1000); // Give everything time to warm up.
	calibrateLDR();
}

// Calibrate the LDR threshold.
void calibrateLDR() {
	Serial.println("Starting calibration.");
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
		Serial.print("Calibrating... ");
		Serial.print(left);
		Serial.print(" ");
		Serial.println(right);
	}
	lightThreshold = (left + right) / 2;
	Serial.print("Calibrated to ");
	Serial.println(lightThreshold);
}

// Read the LDR sensors.
void readLDR() {
	lightReadings[LEFT_LDR] = analogRead(LEFT_LDR_PIN);
	lightReadings[RIGHT_LDR] = analogRead(RIGHT_LDR_PIN);
}

// Compare the LDR readings to the light threshold, and update the LDR booleans accordingly.
void compareLDR() {
	if (abs(lightReadings[LEFT_LDR] - lightReadings[RIGHT_LDR]) >= CALIBRATE_THRESHOLD) {
		if (lightReadings[LEFT_LDR] > lightReadings[RIGHT_LDR]) {
			isOnBlack[LEFT_LDR] = true;
			isOnBlack[RIGHT_LDR] = false;
		} else {
			isOnBlack[LEFT_LDR] = false;
			isOnBlack[RIGHT_LDR] = true;
		}
	} else {
		int avgVal = (lightReadings[LEFT_LDR] + lightReadings[RIGHT_LDR]) / 2;
		if (avgVal > lightThreshold) {
			isOnBlack[LEFT_LDR] = true;
			isOnBlack[RIGHT_LDR] = true;
		} else {
			isOnBlack[LEFT_LDR] = false;
			isOnBlack[RIGHT_LDR] = false;
		}

	}
}

// Move forward.
void forward() {
	Serial.print("\tForward");
	digitalWrite(LEFT_FORWARD, FULL_POWER);
	digitalWrite(LEFT_BACKWARD, NO_POWER);
	digitalWrite(RIGHT_FORWARD, FULL_POWER);
	digitalWrite(RIGHT_BACKWARD, NO_POWER);
}

// Turn left.
void turnLeft() {
	Serial.print("\tLeft");
	digitalWrite(LEFT_FORWARD, NO_POWER);
	digitalWrite(LEFT_BACKWARD, FULL_POWER);
	digitalWrite(RIGHT_FORWARD, FULL_POWER);
	digitalWrite(RIGHT_BACKWARD, NO_POWER);
}

// Turn right.
void turnRight() {
	Serial.print("\tRight");
	digitalWrite(LEFT_FORWARD, FULL_POWER);
	digitalWrite(LEFT_BACKWARD, NO_POWER);
	digitalWrite(RIGHT_FORWARD, NO_POWER);
	digitalWrite(RIGHT_BACKWARD, FULL_POWER);
}

// Move backward.
void backward() {
	Serial.print("\tBackward");
	digitalWrite(LEFT_FORWARD, NO_POWER);
	digitalWrite(LEFT_BACKWARD, FULL_POWER);
	digitalWrite(RIGHT_FORWARD, NO_POWER);
	digitalWrite(RIGHT_BACKWARD, FULL_POWER);
	backupNoise();
}

// Stop the motors.
void stop() {
	//Serial.print("Stop\t");
	digitalWrite(LEFT_FORWARD, NO_POWER);
	digitalWrite(LEFT_BACKWARD, NO_POWER);
	digitalWrite(RIGHT_FORWARD, NO_POWER);
	digitalWrite(RIGHT_BACKWARD, NO_POWER);
}

// Read sensors.
void sense() {
	readLDR();
	compareLDR();
	//Serial.print(isOnBlack[LEFT_LDR]);
	//Serial.print(" ");
	//Serial.print(isOnBlack[RIGHT_LDR]);
}

// Play a beeping noise.
void backupNoise() {
	noTone(SPEAKER);
	tone(SPEAKER, BACKUP_PITCH, BACKUP_LENGTH);
}

// Check for oscillation.
bool isOscillating() {
	return (leftOscillationCount > OSCILLATION_THRESHOLD && rightOscillationCount > OSCILLATION_THRESHOLD);
}

// Do the line tracking.
void lineTrack() {
	if (isOscillating()) {
		Serial.println("OSCILLATING");
		stop();
		delay(10);
		delay(200);
		turnRight();
		delay(750);
		forward();
		delay(250);
		rightOscillationCount = 0;
		leftOscillationCount = 0;
	} else {
		if (isOnBlack[LEFT_LDR] && !isOnBlack[RIGHT_LDR]) {
			forward();
			rightOscillationCount = min(OSCILLATION_THRESHOLD, rightOscillationCount);
			leftOscillationCount = min(OSCILLATION_THRESHOLD, leftOscillationCount);
			rightOscillationCount -= 10;
			leftOscillationCount -= 10;
			rightOscillationCount = max(0, rightOscillationCount);
			leftOscillationCount = max(0, leftOscillationCount);
			delay(5);
		} else if (isOnBlack[LEFT_LDR] && !isOnBlack[RIGHT_LDR]) {
			turnRight();
			rightOscillationCount++;
			delay(10);
		} else if (isOnBlack[LEFT_LDR] && isOnBlack[RIGHT_LDR]) {
			backward();
			delay(5);
			turnRight();
			rightOscillationCount++;
			delay(10);
		} else {
			turnLeft();
			leftOscillationCount++;
			delay(10);
		}
	}
	stop();
}

void loop() {
	Serial.println();
	sense();
	lineTrack();
}
