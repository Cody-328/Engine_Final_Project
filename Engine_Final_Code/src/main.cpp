#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <AccelStepper.h>

// SCREEN PINS AND SETTINGS 
constexpr int SCREEN_WIDTH = 128;
constexpr int SCREEN_HEIGHT = 64;
constexpr int Screen_Button = 12;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// STEPPER MOTOR, RELATED PINS, AND SPEED SETTINGS
constexpr int DRIVER = 1;
constexpr int STEP_PIN = 5;
constexpr int DIR_PIN = 6;
bool AdjScrewOut = false;
constexpr int Stop_Button   = 13;

AccelStepper stepper(DRIVER, STEP_PIN, DIR_PIN);

// Start up running speeds (WaitToStart state)
// Screw in running speed (constant)
float forwardSpeed = -40.0; //Screw turns in

// Screw out running speed (constant)
float reverseSpeed = 40.0; //Screw turns out

// Motor's steps per revolution
const int STEPS_PER_REV = 200;

// Base-tune offset
const float BASE_TURNS = 1.3f;  // change this as needed
const long SPECIAL_STEPS = (long)(STEPS_PER_REV * BASE_TURNS);

// THERMISTOR PINS AND PARAMETERS
// A0 pin = GPIO18 (confirmed by testing)
const int THERM_PIN = 18;        // analog input pin

// Thermistor parameters
const float R_FIXED = 10000.0;    // 10k resistor to GND
const float R0      = 10000.0;   // 10k at 25°C
const float BETA    = 3435.0;    // Beta value
const float T0      = 298.15;    // 25°C in Kelvin

// O2 sensor pin
const int O2_PIN    = 17;   // Narrowband O2 sensor input

// TUNING STEPPER PARAMETERS
const float O2_SETPOINT = 0.45;   // target lambda ~ stoich (V)
const float KP_O2       = 2.0;   // proportional gain (tune this)
const float MAX_SPEED   = 40.0;   // max stepper speed in steps/s

// How far from seated to fully out
const float TOTAL_TURNS = 7.0f;   // total mechanical range
const long MAX_POS_STEPS = (long)(STEPS_PER_REV * TOTAL_TURNS);

// >>> SPARK SENSE (inductive pickup on A4 = pin 14)
const int SPARK_PIN = 14;           // A4 is 14 on your board
const uint32_t SAMPLE_INTERVAL_US = 100;  // 100 µs = 10 kHz

// Threshold for spark detection (tune this after seeing ADC peaks)
const int SPARK_THRESHOLD = 3500;    // start higher than noise, adjust as needed

hw_timer_t* sparkTimer = nullptr;
portMUX_TYPE sparkMux = portMUX_INITIALIZER_UNLOCKED;

// Spark detection state
volatile uint32_t sampleIndex      = 0;  // increments every ISR
volatile uint32_t lastSparkSample  = 0;
volatile uint32_t periodSamples    = 0;
volatile bool     newSparkPeriod   = false;
volatile int      lastRawSpark     = 0;  // optional debug

float engineRPM  = 0.0f;   // instant RPM from latest spark period
float displayRPM = 0.0f;   // smoothed RPM for display

// ENGINE & SCREEN STATE
enum EngineStates {
  WaitToStart,     // 0
  StartEng,        // 1
  OpenLoop,        // 2
  ClosedLoop,      // 3
};

EngineStates currentState = WaitToStart;

enum ScreenStates {
  FuelRatio, // 0
  EngTemp,   // 1
  EngRPM     // 2
};

ScreenStates currentScreen = FuelRatio;

// for edge detection on the screen button
bool lastScreenButton = HIGH;

// >>> SPARK: 10 kHz timer ISR with analogRead + edge detection
void IRAM_ATTR sparkISR() {
  int raw = analogRead(SPARK_PIN);  // 0–4095
  bool above = (raw > SPARK_THRESHOLD);

  static bool lastAbove = false;

  sampleIndex++;

  // Rising edge detection
  if (above && !lastAbove) {
    uint32_t now = sampleIndex;

    if (lastSparkSample != 0) {
      uint32_t diff = now - lastSparkSample;
      if (diff > 0) {
        portENTER_CRITICAL_ISR(&sparkMux);
        periodSamples  = diff;
        newSparkPeriod = true;
        portEXIT_CRITICAL_ISR(&sparkMux);
      }
    }

    lastSparkSample = now;
  }

  lastAbove = above;
  lastRawSpark = raw;  // optional debug
}

void setup() {
  delay(500);
  Serial.begin(9600);
  delay(500);
  
  Wire.begin(); 
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("SSD1306 init failed!");
    while (1);
  }

  // BUTTONS
  pinMode(Stop_Button,   INPUT_PULLUP);
  pinMode(Screen_Button, INPUT_PULLUP);

  // STEPPER
  stepper.setMaxSpeed(MAX_SPEED);
  stepper.setSpeed(forwardSpeed);

  // ADC SETTINGS
  analogReadResolution(12);       // 0–4095

  analogSetPinAttenuation(THERM_PIN, ADC_11db); // 0–3.3V range Thermistor
  analogSetPinAttenuation(O2_PIN,    ADC_0db);  // ~0–1.1V range, best for 0–1V O2 Sensor
  analogSetPinAttenuation(SPARK_PIN, ADC_0db);  // ~0–1.1V range for spark

  // >>> SPARK: 100 µs hardware timer setup (10 kHz)
  // Timer 0, prescaler 80 → 1 MHz timer clock (1 tick = 1 µs)
  sparkTimer = timerBegin(0, 80, true);
  timerAttachInterrupt(sparkTimer, &sparkISR, true);
  timerAlarmWrite(sparkTimer, SAMPLE_INTERVAL_US, true);  // 100 µs
  timerAlarmEnable(sparkTimer);
}

void loop() {

  // >>> SPARK: compute RPM from periodSamples (in 100 µs units)
  {
    uint32_t periodCopy = 0;
    bool     gotPeriod  = false;

    portENTER_CRITICAL(&sparkMux);
    if (newSparkPeriod) {
      periodCopy     = periodSamples;
      newSparkPeriod = false;
      gotPeriod      = true;
    }
    portEXIT_CRITICAL(&sparkMux);

    if (gotPeriod && periodCopy > 0) {
  // Instant RPM from this spark
  float instantRPM = 600000.0f / (float)periodCopy;  // 100 µs per sample

  // sanity clamp
  if (instantRPM < 0)      instantRPM = 0;
  if (instantRPM > 8000)   instantRPM = 8000;

  engineRPM = instantRPM;  // keep the raw value if you ever need it

  // ---- Exponential smoothing for display ----
  // alpha ~ 0.1–0.3 is usually nice. Smaller = smoother, slower.
  const float alpha = 0.05f;

  if (displayRPM == 0.0f) {
    // First time init: avoid big jump from 0
    displayRPM = instantRPM;
  } else {
    displayRPM = (1.0f - alpha) * displayRPM + alpha * instantRPM;
  }

} else {
  // Optional: slowly decay displayRPM to zero if no sparks
  // e.g. after 500 ms of no new period, set to 0 (you can add timing if you want)
}

  }

  // ---------- THERMISTOR READING ----------
  const int samples = 10;
  long sum = 0;

  for (int i = 0; i < samples; i++) {
    sum += analogRead(THERM_PIN);
  }

  float raw = sum / float(samples);

  // Convert ADC (0–4095) to voltage fraction
  float f = raw / 4095.0;

  // Avoid divide-by-zero / log issues
  if (f < 0.0001) f = 0.0001;
  if (f > 0.9999) f = 0.9999;

  // Divider: 3.3V --[R_FIXED]--+--[Thermistor]--GND (ADC at node)
  float R_therm = R_FIXED * f / (1.0 - f);

  // Beta equation
  float invT = (1.0 / T0) + (1.0 / BETA) * log(R_therm / R0);
  float T_K  = 1.0 / invT;
  float T_C  = T_K - 273.15;

  // C → F
  float T_F = (T_C * 9.0 / 5.0) + 32.0;

  // ---------- O2 SENSOR ----------
  int mv  = analogReadMilliVolts(O2_PIN);   // calibrated reading
  float volts = mv / 1000.0;
  float afr;
  afr = 14.7 + 3.0 * tanh( (0.45 - volts) * 12 );

  // ---------- STATE MACHINE ----------
  switch (currentState) {
    case WaitToStart:
      if (!AdjScrewOut) {
        // ---------------------------
        // NORMAL "WAIT" BEHAVIOR
        // ---------------------------
        stepper.runSpeed();   // normal spinning

        display.clearDisplay();
        display.setTextSize(2);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
        display.println("Wait!");
        display.display();

        // Button pressed → start the reverse move
        if (digitalRead(Stop_Button) == LOW) {
          stepper.setSpeed(0);         // stop
          delay(2000);                 // waits for 2 seconds

          stepper.setCurrentPosition(0);   // zero the steps
          stepper.setSpeed(reverseSpeed);  // set reverse speed
          AdjScrewOut = true;             // now we're in the back-move phase
        }

      } else {
        // ---------------------------
        // BACK-MOVE PHASE (still in WaitToStart state)
        // ---------------------------
        stepper.runSpeed();    // keep the motor moving backwards

        long steps = abs(stepper.currentPosition());
        float turns = (float)steps / (float)STEPS_PER_REV;

        // Keep "Wait!" on screen and show turns
        display.clearDisplay();
        display.setTextSize(2);
        display.setCursor(0, 0);
        display.println("Wait!");

        display.setTextSize(2);
        display.setCursor(0, 30);
        display.print("Turns: ");
        display.print(turns, 1);   // 1 decimal place
        display.display();

        // Screw is adjusted
        if (steps >= SPECIAL_STEPS) {
          stepper.setSpeed(0);       // stop motor
          AdjScrewOut = false;       // clear flag

          // Now move on to the next enum state
          currentState = StartEng;
        }
      }
      break;

    case StartEng:
      display.clearDisplay();
      display.setTextSize(2);
      display.setTextColor(SSD1306_WHITE);

      display.setCursor(0, 0);
      display.println("Start ENG");

      display.setCursor(0, 30);
      display.print("RPM:");
      display.print((int)displayRPM);  // show live RPM here

      display.display();

      // Advance once engine is actually running
      if (displayRPM > 1000) {
        currentState = OpenLoop;
      }
      break;

    case OpenLoop:
      display.clearDisplay();
      display.setTextSize(2);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 0);
      display.println("Open Loop");
      display.setCursor(0, 34);
      display.print("Temp:");
      display.print(T_F, 0);   // 0 decimal places after the dot
      display.print("F");
      display.display();

      if (T_F >= 120){ //Temperature threshold to enter closed loop
        currentState = ClosedLoop;
      }
      break;

    case ClosedLoop: {
      // Read current position (0 = seated IN)
      long pos = stepper.currentPosition();

      float cmdSpeed = 0.0f;

      // -------- ZERO / STOP BUTTON SAFETY --------
      bool stopPressed = (digitalRead(Stop_Button) == LOW);

      if (stopPressed) {
        // If button is pressed, we want to safely return to seat (pos = 0)
        // but NEVER go past it.

        if (pos > 0) {
          // Above zero: walk IN slowly until we hit seat
          float homingSpeed = -50.0f;   // slow, safe IN movement
          cmdSpeed = homingSpeed;

          // Seat protection: if we're at/below zero, don't go further IN
          if (pos <= 0) {
            cmdSpeed = 0;
            stepper.setCurrentPosition(0);  // make sure 0 is exact
          }
        } else {
          // Already at or below seat -> stop moving
          cmdSpeed = 0;
          stepper.setCurrentPosition(0);    // clamp at 0
        }
      } else {
        // -------- NORMAL O2 P-CONTROL --------
        float error = O2_SETPOINT - volts;

        // Optional: clamp error to ignore crazy spikes
        if (error >  0.4) error =  0.4;
        if (error < -0.4) error = -0.4;

        cmdSpeed = KP_O2 * error;
        
        //----- DEBUG OUTPUT -----
        Serial.print("O2 volts: ");
        Serial.print(volts, 3);
        Serial.print("   AFR: ");
        Serial.print(afr, 1);
        Serial.print("   err: ");
        Serial.print(error, 3);
        Serial.print("   cmd: ");
        Serial.print(cmdSpeed, 1);
        Serial.print("   pos: ");
        Serial.println(stepper.currentPosition());

        // Speed clamp (motor safety)
        if (cmdSpeed >  MAX_SPEED) cmdSpeed =  MAX_SPEED;
        if (cmdSpeed < -MAX_SPEED) cmdSpeed = -MAX_SPEED;

        // ----- POSITION LIMITING -----
        // 1) Don't go IN past the seat (pos <= 0)
        if (pos <= 0 && cmdSpeed < 0) {
          cmdSpeed = 0;
        }

        // 2) Don't go OUT past max (~7 turns)
        if (pos >= MAX_POS_STEPS && cmdSpeed > 0) {
          cmdSpeed = 0;
        }
      }

      // Apply final speed command
      stepper.setSpeed(cmdSpeed);
      stepper.runSpeed();

      // --- 1) Handle screen button (with edge detection) ---
      bool screenButton = digitalRead(Screen_Button);
      if (lastScreenButton == HIGH && screenButton == LOW) {
        // button just pressed -> go to next screen
        currentScreen = (ScreenStates)((currentScreen + 1) % 3);
      }
      lastScreenButton = screenButton;

      // --- 2) Draw current closed-loop screen ---
      display.clearDisplay();
      display.setTextSize(2);
      display.setTextColor(SSD1306_WHITE);

      // Title line
      display.setCursor(0, 0);
      display.println("CLSD Loop");

      // Second line for what you’re viewing
      display.setCursor(0, 34);  // start lower so it’s a separate line

      switch (currentScreen) {
        case FuelRatio:
          display.print("AFR:");
          display.print(afr, 1);
          break;
        case EngTemp:
          display.print("Temp:");
          display.print(T_F, 0);   // 0 decimal places after the dot
          display.print("F");
          break;
        case EngRPM:
          display.print("RPM:");
          display.print((int)displayRPM);  // show live RPM
          break;
      }
      display.display();
      break;
    }
  }
}
