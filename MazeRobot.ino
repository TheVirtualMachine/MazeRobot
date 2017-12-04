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
#define LEFT_LDR_PIN A0
#define MIDDLE_LDR_PIN A1
#define RIGHT_LDR_PIN A2

// Define motor pin numbers.
#define LEFT_ENABLE 10 // The pin to enable the left motor.
#define RIGHT_ENABLE 11 // The pin to enable the right motor.
#define LEFT_FORWARD 13 // The pin to move the left motor forwards.
#define LEFT_BACKWARD 12 // The pin to move the left motor backwards.
#define RIGHT_FORWARD 6 // The pin to move the right motor forwards.
#define RIGHT_BACKWARD 7 // The pin to move the right motor backwards.

// Define miscellaneous pins.
#define SPEAKER 3 // The pin for the speaker.

// Variables for LDRs.
int leftLDR, middleLDR, rightLDR;

// Define thresholds.
#define LDR_THRESHOLD 550

// Define motor power amounts.
#define FULL_POWER 255 // Analog value for full motor power.
#define HALF_POWER 128 // Analog value for half motor power.
#define NO_POWER 0 // Analog value for no motor power.

// Setup the program.
void setup() {
	// Setup the motors.
	pinMode(LEFT_ENABLE, OUTPUT);
	pinMode(RIGHT_ENABLE, OUTPUT);
	pinMode(LEFT_FORWARD, OUTPUT);
	pinMode(LEFT_BACKWARD, OUTPUT);
	pinMode(RIGHT_FORWARD, OUTPUT);
	pinMode(RIGHT_BACKWARD, OUTPUT);

	Serial.begin(9600); // Start the serial communication.
}

// Read the LDR sensors.
void readLDR() {
	leftLDR = analogRead(LEFT_LDR_PIN);
	middleLDR = analogRead(MIDDLE_LDR_PIN);
	rightLDR = analogRead(RIGHT_LDR_PIN);
}

// Move forward.
void forward() {
	analogWrite(LEFT_ENABLE, FULL_POWER);
	analogWrite(RIGHT_ENABLE, FULL_POWER);

	digitalWrite(LEFT_FORWARD, HIGH);
	digitalWrite(LEFT_BACKWARD, LOW);
	digitalWrite(RIGHT_FORWARD, HIGH);
	digitalWrite(RIGHT_BACKWARD, LOW);
}

// Turn left.
void turnLeft() {
	analogWrite(LEFT_ENABLE, FULL_POWER);
	analogWrite(RIGHT_ENABLE, FULL_POWER);

	digitalWrite(LEFT_FORWARD, LOW);
	digitalWrite(LEFT_BACKWARD, HIGH);
	digitalWrite(RIGHT_FORWARD, HIGH);
	digitalWrite(RIGHT_BACKWARD, LOW);
}

// Veer left.
void veerLeft() {
	analogWrite(LEFT_ENABLE, HALF_POWER);
	analogWrite(RIGHT_ENABLE, FULL_POWER);

	digitalWrite(LEFT_FORWARD, HIGH);
	digitalWrite(LEFT_BACKWARD, LOW);
	digitalWrite(RIGHT_FORWARD, HIGH);
	digitalWrite(RIGHT_BACKWARD, LOW);
}

// Turn right.
void turnRight() {
	analogWrite(LEFT_ENABLE, FULL_POWER);
	analogWrite(RIGHT_ENABLE, FULL_POWER);

	digitalWrite(LEFT_FORWARD, HIGH);
	digitalWrite(LEFT_BACKWARD, LOW);
	digitalWrite(RIGHT_FORWARD, LOW);
	digitalWrite(RIGHT_BACKWARD, HIGH);
}

// Veer right.
void veerRight() {
	analogWrite(LEFT_ENABLE, FULL_POWER);
	analogWrite(RIGHT_ENABLE, HALF_POWER);

	digitalWrite(LEFT_FORWARD, HIGH);
	digitalWrite(LEFT_BACKWARD, LOW);
	digitalWrite(RIGHT_FORWARD, HIGH);
	digitalWrite(RIGHT_BACKWARD, LOW);
}

// Move backward.
void backward() {
	analogWrite(LEFT_ENABLE, FULL_POWER);
	analogWrite(RIGHT_ENABLE, FULL_POWER);

	digitalWrite(LEFT_FORWARD, LOW);
	digitalWrite(LEFT_BACKWARD, HIGH);
	digitalWrite(RIGHT_FORWARD, LOW);
	digitalWrite(RIGHT_BACKWARD, HIGH);
}

// Stop the motors.
void stop() {
	analogWrite(LEFT_ENABLE, NO_POWER);
	analogWrite(RIGHT_ENABLE, NO_POWER);

	digitalWrite(LEFT_FORWARD, LOW);
	digitalWrite(LEFT_BACKWARD, LOW);
	digitalWrite(RIGHT_FORWARD, LOW);
	digitalWrite(RIGHT_BACKWARD, LOW);
}

// Check if a given LDR reading should trigger a reaction.
bool isLDRTriggered(int level) {
	return (level >= LDR_THRESHOLD);
}

// Read sensors.
void sense() {
	readLDR();
}

void backupNoise() {
	noTone(SPEAKER);
	tone(SPEAKER, BACKUP_PITCH, BACKUP_LENGTH);
}

// Do the line tracking.
void lineTrack() {
	if (!isLDRTriggered(middleLDR)) {
		if (isLDRTriggered(leftLDR)) {
			veerLeft();
		} else if (isLDRTriggered(rightLDR)) {
			veerRight();
		} else {
			turnLeft();
		}
	}
}

void loop() {
	sense();
	lineTrack();
}
