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
#define MIDDLE_LDR_PIN A2
#define RIGHT_LDR_PIN A4

// Define motor pin numbers.
#define LEFT_FORWARD 13 // The pin to move the left motor forwards.
#define LEFT_BACKWARD 12 // The pin to move the left motor backwards.
#define RIGHT_FORWARD 7 // The pin to move the right motor forwards.
#define RIGHT_BACKWARD 6 // The pin to move the right motor backwards.

// Define miscellaneous pins.
#define SPEAKER 3 // The pin for the speaker.

// Variables for LDRs.
int leftLDR, middleLDR, rightLDR;

// Define thresholds.
#define LDR_THRESHOLD 500

// Define motor power amounts.
#define FULL_POWER 200 // Analog value for full motor power.
#define HALF_POWER 150 // Analog value for half motor power.
#define NO_POWER 0 // Analog value for no motor power.

// Setup the program.
void setup() {
	delay(100); // Give everything time to warm up.
	// Setup the motors.
	pinMode(LEFT_FORWARD, OUTPUT);
	pinMode(LEFT_BACKWARD, OUTPUT);
	pinMode(RIGHT_FORWARD, OUTPUT);
	pinMode(RIGHT_BACKWARD, OUTPUT);
	stop();

	Serial.begin(9600); // Start the serial communication.
	delay(1000); // Give everything time to warm up.
}

// Read the LDR sensors.
void readLDR() {
	leftLDR = analogRead(LEFT_LDR_PIN);
	middleLDR = analogRead(MIDDLE_LDR_PIN);
	rightLDR = analogRead(RIGHT_LDR_PIN);
}

// Move forward.
void forward() {
	Serial.print("	Forward");
	digitalWrite(LEFT_FORWARD, HIGH);
	digitalWrite(LEFT_BACKWARD, LOW);
	digitalWrite(RIGHT_FORWARD, HIGH);
	digitalWrite(RIGHT_BACKWARD, LOW);
}

// Turn left.
void turnLeft() {
	Serial.print("	Left");
	digitalWrite(LEFT_FORWARD, LOW);
	digitalWrite(LEFT_BACKWARD, HIGH);
	digitalWrite(RIGHT_FORWARD, HIGH);
	digitalWrite(RIGHT_BACKWARD, LOW);
}

// Turn right.
void turnRight() {
	Serial.print("	Right");
	digitalWrite(LEFT_FORWARD, HIGH);
	digitalWrite(LEFT_BACKWARD, LOW);
	digitalWrite(RIGHT_FORWARD, LOW);
	digitalWrite(RIGHT_BACKWARD, HIGH);
}

// Move backward.
void backward() {
	Serial.print("	Backward");
	digitalWrite(LEFT_FORWARD, LOW);
	digitalWrite(LEFT_BACKWARD, HIGH);
	digitalWrite(RIGHT_FORWARD, LOW);
	digitalWrite(RIGHT_BACKWARD, HIGH);
}

// Stop the motors.
void stop() {
	Serial.print("Stop	");
	digitalWrite(LEFT_FORWARD, LOW);
	digitalWrite(LEFT_BACKWARD, LOW);
	digitalWrite(RIGHT_FORWARD, LOW);
	digitalWrite(RIGHT_BACKWARD, LOW);
}

// Check if a given LDR reading should trigger a reaction.
// Returns true on black, and false on white.
bool isLDRTriggered(int level) {
	return (level >= LDR_THRESHOLD);
}

// Read sensors.
void sense() {
	readLDR();
	Serial.print(leftLDR);
	Serial.print(" ");
	Serial.print(middleLDR);
	Serial.print(" ");
	Serial.print(rightLDR);
}

void backupNoise() {
	noTone(SPEAKER);
	tone(SPEAKER, BACKUP_PITCH, BACKUP_LENGTH);
}

// Do the line tracking.
void lineTrack() {
	if (!isLDRTriggered(middleLDR) && !isLDRTriggered(rightLDR)) {
		if (isLDRTriggered(leftLDR)) {
			forward();
		} else {
			turnLeft();
		}
	} else {
		turnRight();
	}
}

void loop() {
	stop();
	delay(50);
	sense();
	lineTrack();
	delay(30);
	Serial.println();
}
