#include <Arduino.h>

// Your A0 pin = GPIO18 (confirmed by testing)
const int THERM_PIN = 18;        // analog input pin

// Thermistor parameters
const float R_FIXED = 10000.0;    // 10k resistor to GND
const float R0      = 10000.0;   // 10k at 25°C
const float BETA    = 3435.0;    // Beta value
const float T0      = 298.15;    // 25°C in Kelvin

void setup() {
  Serial.begin(115200);
  delay(500);

  analogReadResolution(12);       // 0–4095
  analogSetAttenuation(ADC_11db); // 0–3.3V range
}

void loop() {
  const int samples = 10;
  long sum = 0;

  for (int i = 0; i < samples; i++) {
    sum += analogRead(THERM_PIN);
    delay(5);
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

  // Print final temperature
  Serial.print("ADC: "); Serial.print(raw);
  Serial.print("  R: "); Serial.print(R_therm, 1);
  Serial.print("  Temp: "); Serial.print(T_F, 2);
  Serial.println(" F");

  delay(500);
}