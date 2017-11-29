/********************************************
* Vincent Macri, David White, Kevin Du      *
* Mr. Wong                                  *
* 2018-01-17                                *
* DESCRIPTIVE COMMENT.                      *
*********************************************/
// Define LDR pins.
#define LEFT_LDR A0
#define MIDDLE_LDR A1
#define RIGHT_LDR A2

// Define motor pin numbers.
#define LEFT_FORWARD 13 // The pin to move the left motor forwards.
#define LEFT_BACKWARD 12 // The pin to move the left motor forwards.
#define RIGHT_FORWARD 6 // The pin to move the left motor forwards.
#define RIGHT_BACKWARD 7 // The pin to move the left motor forwards.

// Variables for LDRs.
int frontLeftLight, frontRightLight, backLeftLight, backRightLight;

// Define thresholds.
#define LDR_THRESHOLD 550

// Setup the program.
void setup() {
	// Setup the motors.
	pinMode(LEFT_FORWARD, OUTPUT);
	pinMode(LEFT_BACKWARD, OUTPUT);
	pinMode(RIGHT_FORWARD, OUTPUT);
	pinMode(RIGHT_BACKWARD, OUTPUT);

	Serial.begin(9600); // Start the serial communication.
}

// Read the LDR sensors.
void readLDR() {
	frontLeftLight = analogRead(LDR_FRONT_LEFT);
	frontRightLight = analogRead(LDR_FRONT_RIGHT);
	backLeftLight = analogRead(LDR_BACK_LEFT);
	backRightLight = analogRead(LDR_BACK_RIGHT);
}

// Move forward.
void forward() {
	digitalWrite(LEFT_FORWARD, HIGH);
	digitalWrite(LEFT_BACKWARD, LOW);
	digitalWrite(RIGHT_FORWARD, HIGH);
	digitalWrite(RIGHT_BACKWARD, LOW);
}

// Move left.
void left() {
	digitalWrite(LEFT_FORWARD, LOW);
	digitalWrite(LEFT_BACKWARD, HIGH);
	digitalWrite(RIGHT_FORWARD, HIGH);
	digitalWrite(RIGHT_BACKWARD, LOW);
}

// Move right.
void right() {
	digitalWrite(LEFT_FORWARD, HIGH);
	digitalWrite(LEFT_BACKWARD, LOW);
	digitalWrite(RIGHT_FORWARD, LOW);
	digitalWrite(RIGHT_BACKWARD, HIGH);
}

// Move backward.
void backward() {
	digitalWrite(LEFT_FORWARD, LOW);
	digitalWrite(LEFT_BACKWARD, HIGH);
	digitalWrite(RIGHT_FORWARD, LOW);
	digitalWrite(RIGHT_BACKWARD, HIGH);
}

// Stop the motors.
void stop() {
	digitalWrite(LEFT_FORWARD, LOW);
	digitalWrite(LEFT_BACKWARD, LOW);

	digitalWrite(RIGHT_FORWARD, LOW);
	digitalWrite(RIGHT_BACKWARD, LOW);
}

// Check if a given LDR reading should trigger a reaction.
bool isLDRTriggered(int level) {
	return (level <= LDR_THRESHOLD);
}

void loop() {
	// readLDR();
	forward();
}
