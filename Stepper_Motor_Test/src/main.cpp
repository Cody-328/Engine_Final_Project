#include <AccelStepper.h>

#define DRIVER 1
#define STEP_PIN 5
#define DIR_PIN 6
#define STOP_BUTTON_PIN 13

AccelStepper stepper(DRIVER, STEP_PIN, DIR_PIN);

// Normal running speed (constant)
float forwardSpeed = -80.0;

// Fast speed for special 1.5-turn move
float reverseSpeed = 50.0;

// Set this to your motor's steps per revolution
const int STEPS_PER_REV = 200;

// Dynamic 1.5-turn distance
const int SPECIAL_STEPS = STEPS_PER_REV * 1.5;

void setup() {
  pinMode(STOP_BUTTON_PIN, INPUT_PULLUP);  // Button connects to GND
  stepper.setMaxSpeed(100);
  stepper.setSpeed(forwardSpeed);
}

void loop() {

  // Normal spinning until button is pressed
  if (digitalRead(STOP_BUTTON_PIN) == LOW) {

    // Stop immediately
    // (Just don't call runSpeed)
    delay(2000);  // Wait 2 seconds

    // Reset step counter
    stepper.setCurrentPosition(0);

    // Move opposite direction at a faster constant speed
    stepper.setSpeed(reverseSpeed);

    // Run until exactly 1.5 turns have completed
    while (abs(stepper.currentPosition()) < SPECIAL_STEPS) {
      stepper.runSpeed();
    }

    // ---- STOP FOREVER AFTER THE BACK MOVE ----
    while (true) {
      // Do nothing, hold position
    }
  }

  // Normal continuous rotation
  stepper.runSpeed();
}
