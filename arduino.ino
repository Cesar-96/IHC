#include <AccelStepper.h>
#include <Servo.h>
#include <math.h>

// Enums
enum Command {
  OPEN = 'O',
  CLOSE = 'C',
  UP = 'U',
  DOWN = 'D',
  LEFT = 'L',
  RIGHT = 'R',
  NONE = 'N'
};

// Globals
#define BUFFER_SIZE 10

char buffer[BUFFER_SIZE];
Command command = NONE;

int curr_idx = -1, top_idx = 0;

// Switches
#define limitSwitch1 11
#define limitSwitch2 10
#define limitSwitch3 9
#define limitSwitch4 A3

// Steppers
AccelStepper stepper1(1, 2, 5);
AccelStepper stepper2(1, 3, 6);
AccelStepper stepper3(1, 4, 7);
AccelStepper stepper4(1, 12, 13);

// Gripper
Servo gripper;
int gripper_open = 100, gripper_close = 0; // Angles

void setup() {
  Serial.begin(115200);

  gripper.attach(A0);

  pinMode(limitSwitch1, INPUT_PULLUP);
  pinMode(limitSwitch2, INPUT_PULLUP);
  pinMode(limitSwitch3, INPUT_PULLUP);
  pinMode(limitSwitch4, INPUT_PULLUP);

  // Stepper motors max speed
  stepper1.setMaxSpeed(4000);
  stepper1.setAcceleration(2000);
  stepper2.setMaxSpeed(4000);
  stepper2.setAcceleration(2000);
  stepper3.setMaxSpeed(4000);
  stepper3.setAcceleration(2000);
  stepper4.setMaxSpeed(3700);
  stepper4.setAcceleration(1850);

  calibration();
}

void loop() {
  while (Serial.available() > 0) {
    char receivedChar = Serial.read();
    buffer[top_idx++] = receivedChar;
    if (top_idx == BUFFER_SIZE) top_idx = 0;
  }

  procces_buffer();
}

void procces_buffer() {
  if (curr_idx != top_idx) {
    command = static_cast<Command>(buffer[curr_idx]);

    switch (command) {
      case OPEN:
        move_gripper(gripper_open);
        break;
      case CLOSE:
        move_gripper(gripper_close);
        break;
      case UP:
        move_stepper(&stepper4, 15000);
        break;
      case DOWN:
        move_stepper(&stepper4, 5000);
        break;
      case LEFT:
        move_stepper(&stepper1, -3500);
        break;
      case RIGHT:
        move_stepper(&stepper1, 0);
        break;
    }

    buffer[curr_idx] = NONE;
    if (++curr_idx == BUFFER_SIZE) curr_idx = 0;
  }
}

void move_gripper(int gripper_angle) {
  gripper.write(gripper_angle);
}

void move_stepper(AccelStepper *stepper, int steps) {
  stepper->moveTo(steps);
  stepper->runToPosition();
}

void calibration() {
  // Move to the top position
  while (digitalRead(limitSwitch4) != 1) {
    stepper4.setSpeed(1500);
    stepper4.runSpeed();
    stepper4.setCurrentPosition(17000);
  }

  // Delay for stability
  delay(20);

  // Move to a mid-position
  stepper4.moveTo(10000);
  while (stepper4.currentPosition() != 10000)
    stepper4.run();

  // Delay for stability
  delay(20);

  // Move to the leftmost position
  while (digitalRead(limitSwitch1) != 1) {
    stepper1.setSpeed(-1200);
    stepper1.runSpeed();
    stepper1.setCurrentPosition(-4000);
  }

  // Delay for stability
  delay(20);

  // Move to a mid-position
  stepper1.moveTo(-2000);
  while (stepper1.currentPosition() != -2000)
    stepper1.run();
}
#include <AccelStepper.h>
#include <Servo.h>
#include <math.h>

// Enums
enum Command {
  OPEN = 'O',
  CLOSE = 'C',
  UP = 'U',
  DOWN = 'D',
  LEFT = 'L',
  RIGHT = 'R',
  NONE = 'N'
};

// Globals
#define BUFFER_SIZE 10

char buffer[BUFFER_SIZE];
Command command = NONE;

int curr_idx = -1, top_idx = 0;

// Switches
#define limitSwitch1 11
#define limitSwitch2 10
#define limitSwitch3 9
#define limitSwitch4 A3

// Steppers
AccelStepper stepper1(1, 2, 5);
AccelStepper stepper2(1, 3, 6);
AccelStepper stepper3(1, 4, 7);
AccelStepper stepper4(1, 12, 13);

// Gripper
Servo gripper;
int gripper_open = 100, gripper_close = 0; // Angles

void setup() {
  Serial.begin(115200);

  gripper.attach(A0);

  pinMode(limitSwitch1, INPUT_PULLUP);
  pinMode(limitSwitch2, INPUT_PULLUP);
  pinMode(limitSwitch3, INPUT_PULLUP);
  pinMode(limitSwitch4, INPUT_PULLUP);

  // Stepper motors max speed
  stepper1.setMaxSpeed(4000);
  stepper1.setAcceleration(2000);
  stepper2.setMaxSpeed(4000);
  stepper2.setAcceleration(2000);
  stepper3.setMaxSpeed(4000);
  stepper3.setAcceleration(2000);
  stepper4.setMaxSpeed(3700);
  stepper4.setAcceleration(1850);

  calibration();
}

void loop() {
  while (Serial.available() > 0) {
    char receivedChar = Serial.read();
    buffer[top_idx++] = receivedChar;
    if (top_idx == BUFFER_SIZE) top_idx = 0;
  }

  procces_buffer();
}

void procces_buffer() {
  if (curr_idx != top_idx) {
    command = static_cast<Command>(buffer[curr_idx]);

    switch (command) {
      case OPEN:
        move_gripper(gripper_open);
        break;
      case CLOSE:
        move_gripper(gripper_close);
        break;
      case UP:
        move_stepper(&stepper4, 15000);
        break;
      case DOWN:
        move_stepper(&stepper4, 5000);
        break;
      case LEFT:
        move_stepper(&stepper1, -3500);
        break;
      case RIGHT:
        move_stepper(&stepper1, 0);
        break;
    }

    buffer[curr_idx] = NONE;
    if (++curr_idx == BUFFER_SIZE) curr_idx = 0;
  }
}

void move_gripper(int gripper_angle) {
  gripper.write(gripper_angle);
}

void move_stepper(AccelStepper *stepper, int steps) {
  stepper->moveTo(steps);
  stepper->runToPosition();
}

void calibration() {
  // Move to the top position
  while (digitalRead(limitSwitch4) != 1) {
    stepper4.setSpeed(1500);
    stepper4.runSpeed();
    stepper4.setCurrentPosition(17000);
  }

  // Delay for stability
  delay(20);

  // Move to a mid-position
  stepper4.moveTo(10000);
  while (stepper4.currentPosition() != 10000)
    stepper4.run();

  // Delay for stability
  delay(20);

  // Move to the leftmost position
  while (digitalRead(limitSwitch1) != 1) {
    stepper1.setSpeed(-1200);
    stepper1.runSpeed();
    stepper1.setCurrentPosition(-4000);
  }

  // Delay for stability
  delay(20);

  // Move to a mid-position
  stepper1.moveTo(-2000);
  while (stepper1.currentPosition() != -2000)
    stepper1.run();
}
