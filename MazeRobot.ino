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
#define LEFT_LDR_PIN A2
//#define MIDDLE_LDR_PIN A0
#define RIGHT_LDR_PIN A1

// Define motor pin numbers.
#define LEFT_FORWARD 13 // The pin to move the left motor forwards.
#define LEFT_BACKWARD 12 // The pin to move the left motor backwards.
#define RIGHT_FORWARD 4 // The pin to move the right motor forwards.
#define RIGHT_BACKWARD 5 // The pin to move the right motor backwards.

// Define miscellaneous pins.
#define SPEAKER 3 // The pin for the speaker.

// Variables for LDRs.
#define LEFT_LDR 0
//#define MIDDLE_LDR 1
#define RIGHT_LDR 2

#define LDR_COUNT 3

#define CALIBRATE_THRESHOLD 200

int lightReadings[LDR_COUNT];
bool isOnBlack[LDR_COUNT];

int lightThreshold;

// Define motor power amounts.
#define FULL_POWER LOW // Analog value for full motor power.
#define NO_POWER LOW // Analog value for no motor power.

// Setup the program.
void setup() {
	stop();
	delay(100); // Give everything time to warm up.
	Serial.begin(9600); // Start the serial communication.
	delay(100); // Give everything time to warm up.
	Serial.println("Starting program.");
	// Setup the motors.
	pinMode(LEFT_FORWARD, OUTPUT);
	pinMode(LEFT_BACKWARD, OUTPUT);
	pinMode(RIGHT_FORWARD, OUTPUT);
	pinMode(RIGHT_BACKWARD, OUTPUT);
	stop();
	delay(1000); // Give everything time to warm up.
	calibrateLDR();
}

// Calibrate the LDR threshold.
void calibrateLDR() {
	Serial.println("Starting calibration.");
	int minReading;
	int maxReading;
	int diff = CALIBRATE_THRESHOLD;
	while (diff <= CALIBRATE_THRESHOLD) {
		int left = 0;
		//int middle = 0;
		int right = 0;
		for (int i = 0; i < 10; i++) {
			readLDR();
			left += lightReadings[LEFT_LDR];
			//middle += lightReadings[MIDDLE_LDR];
			right += lightReadings[RIGHT_LDR];
		}

		left /= 10;
		//middle /= 10;
		right /= 10;

		minReading = min(left, right);
		maxReading = max(left, right);
		diff = maxReading - minReading;
		Serial.print("Calibrating... ");
		Serial.print(lightReadings[LEFT_LDR]);
		Serial.print(" ");
		//Serial.print(lightReadings[MIDDLE_LDR]);
		//Serial.print(" ");
		Serial.println(lightReadings[RIGHT_LDR]);
	}
	lightThreshold = (minReading + maxReading) / 2;
	Serial.print("Calibrated to ");
	Serial.println(lightThreshold);
}

// Read the LDR sensors.
void readLDR() {
	lightReadings[LEFT_LDR] = analogRead(LEFT_LDR_PIN);
	//lightReadings[MIDDLE_LDR] = analogRead(MIDDLE_LDR_PIN);
	lightReadings[RIGHT_LDR] = analogRead(RIGHT_LDR_PIN);
}

// Compare the LDR readings to the light threshold, and update the LDR booleans accordingly.
void compareLDR() {
	isOnBlack[LEFT_LDR] = lightReadings[LEFT_LDR] > lightThreshold;
	//isOnBlack[MIDDLE_LDR] = lightReadings[MIDDLE_LDR] > lightThreshold;
	isOnBlack[RIGHT_LDR] = lightReadings[RIGHT_LDR] > lightThreshold;
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
	Serial.print("Stop\t");
	digitalWrite(LEFT_FORWARD, NO_POWER);
	digitalWrite(LEFT_BACKWARD, NO_POWER);
	digitalWrite(RIGHT_FORWARD, NO_POWER);
	digitalWrite(RIGHT_BACKWARD, NO_POWER);
}

// Read sensors.
void sense() {
	readLDR();
	compareLDR();
	Serial.print(lightReadings[LEFT_LDR]);
	Serial.print(" ");
	//Serial.print(lightReadings[MIDDLE_LDR]);
	//Serial.print(" ");
	Serial.print(lightReadings[RIGHT_LDR]);
}

// Play a beeping noise.
void backupNoise() {
	noTone(SPEAKER);
	tone(SPEAKER, BACKUP_PITCH, BACKUP_LENGTH);
}

// Do the line tracking.
void lineTrack() {
	if (isOnBlack[RIGHT_LDR] && !isOnBlack[LEFT_LDR]) {
		forward();
		delay(10);
		stop();
		delay(25);
	} else if (isOnBlack[LEFT_LDR] && !isOnBlack[RIGHT_LDR]) {
		turnLeft();
	} else if (!isOnBlack[LEFT_LDR] && !isOnBlack[RIGHT_LDR]) {
		turnRight();
	} else {
		//backward();
		delay(10);
		turnLeft();
		delay(30);
	}
}

void loop() {
	Serial.println();
	sense();
	lineTrack();
}
