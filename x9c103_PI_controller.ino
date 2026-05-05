#include <Arduino.h>

// =============================================================
//  X9C103S PI Voltage Controller
//  Calibration: 1V = 6 steps, max 5V = 30 steps
//  User sends target voltage via Serial Monitor
//  PI controller adjusts wiper step to match
// =============================================================

// ── Pin Definitions ──────────────────────────────────────────
const int CS     = 14;
const int INC    = 12;
const int UD     = 13;
const int ADC_PIN = 35;

// ── Potentiometer Config ──────────────────────────────────────
const int   MAX_STEPS        = 99;
const float STEPS_PER_VOLT   = 6.0f;     // calibration: 1V = 6 steps
const float MAX_VOLTAGE      = 5.0f;     // hard upper limit
const float MIN_VOLTAGE      = 0.0f;

// Derived: max usable step from calibration = 5V * 6 = 30
const int   MAX_CAL_STEP     = 30;       // 5V ceiling

// ── ADC Config ───────────────────────────────────────────────
const float ADC_REF          = 3.3f;
const float ADC_MAX          = 4095.0f;
const float DIVIDER_RATIO    = 2.0f;     // VW = V_gpio * 2
const int   OVERSAMPLE       = 64;

// ── Two-Point Calibration ─────────────────────────────────────
const float CAL_RAW_LOW      = 0.3651f;
const float CAL_ACTUAL_LOW   = 0.52f;
const float CAL_RAW_HIGH     = 2.2822f;
const float CAL_ACTUAL_HIGH  = 2.47f;

// ── Timing ───────────────────────────────────────────────────
const int   INC_PULSE_US     = 2;
const int   CS_SETTLE_US     = 1;
const int   STEP_DELAY_MS    = 5;

// ── PI Controller Parameters ─────────────────────────────────
// Tune these for your system's response
float Kp = 3.0f;   // Proportional gain — how aggressively to correct error
float Ki = 1.9f;   // Integral gain    — how aggressively to eliminate steady-state error

const float PI_INTERVAL_MS   = 200.0f;  // controller runs every 200ms
const float PI_DT            = PI_INTERVAL_MS / 1000.0f; // in seconds

// Integral windup limit (in volts) — prevents integrator from accumulating too much
const float INTEGRAL_LIMIT   = 2.0f;

// Deadband — stop correcting if error is smaller than this (avoids jitter)
const float DEADBAND_V       = 0.05f;   // 50mV deadband

// ── State ────────────────────────────────────────────────────
int   currentStep    = 0;
float targetVoltage  = 0.0f;
float integral       = 0.0f;
unsigned long lastPITime = 0;

// =============================================================
//  X9C103S Control Functions
// =============================================================
void sendPulse() {
  digitalWrite(INC, HIGH);
  delayMicroseconds(INC_PULSE_US);
  digitalWrite(INC, LOW);
  delayMicroseconds(INC_PULSE_US);
}

void resetToZero() {
  Serial.println("[INIT] Winding potentiometer to step 0...");
  digitalWrite(UD, LOW);
  delayMicroseconds(CS_SETTLE_US);
  digitalWrite(CS, LOW);
  delayMicroseconds(CS_SETTLE_US);

  for (int i = 0; i <= MAX_STEPS; i++) {
    sendPulse();
    delayMicroseconds(INC_PULSE_US);
  }

  digitalWrite(CS, HIGH);
  currentStep = 0;
  Serial.println("[INIT] Wiper at step 0.");
}

void setStep(int targetStep) {
  // Hard clamp — never exceed 5V calibrated ceiling
  targetStep = constrain(targetStep, 0, MAX_CAL_STEP);

  if (targetStep == currentStep) return;

  int  steps = abs(targetStep - currentStep);
  bool goUp  = (targetStep > currentStep);

  digitalWrite(UD, goUp ? HIGH : LOW);
  delayMicroseconds(CS_SETTLE_US);
  digitalWrite(CS, LOW);
  delayMicroseconds(CS_SETTLE_US);

  for (int i = 0; i < steps; i++) {
    sendPulse();
    delay(STEP_DELAY_MS);
  }

  digitalWrite(CS, HIGH);
  currentStep = targetStep;
}

// =============================================================
//  ADC Reading
// =============================================================
float applyCalibration(float raw_v) {
  float slope  = (CAL_ACTUAL_HIGH - CAL_ACTUAL_LOW) / (CAL_RAW_HIGH - CAL_RAW_LOW);
  float offset = CAL_ACTUAL_LOW - slope * CAL_RAW_LOW;
  return slope * raw_v + offset;
}

float readVW() {
  long sum = 0;
  for (int i = 0; i < OVERSAMPLE; i++) {
    sum += analogRead(ADC_PIN);
    delayMicroseconds(100);
  }
  int   raw    = (int)(sum / OVERSAMPLE);
  float v_gpio = (raw / ADC_MAX) * ADC_REF;
  float v_cal  = applyCalibration(v_gpio);
  return v_cal * DIVIDER_RATIO;
}

// =============================================================
//  PI Controller
//  Converts voltage error → step adjustment
// =============================================================
void runPI() {
  float measured = readVW();
  float error    = targetVoltage - measured;

  // Deadband — ignore tiny errors to prevent wiper hunting
  if (fabs(error) < DEADBAND_V) {
    integral = integral * 0.95f; // gently decay integral in deadband
    printStatus(measured, error, 0);
    return;
  }

  // Accumulate integral with anti-windup clamp
  integral += error * PI_DT;
  integral  = constrain(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

  // PI output in volts
  float output_v = (Kp * error) + (Ki * integral);

  // Convert voltage correction → step correction
  int stepCorrection = (int)roundf(output_v * STEPS_PER_VOLT);

  // Clamp correction to max ±5 steps per cycle to avoid aggressive jumps
  stepCorrection = constrain(stepCorrection, -5, 5);

  int newStep = constrain(currentStep + stepCorrection, 0, MAX_CAL_STEP);
  setStep(newStep);

  printStatus(measured, error, stepCorrection);
}

// =============================================================
//  Serial Output
// =============================================================
void printStatus(float measured, float error, int stepCorr) {
  float expectedV = (float)currentStep / STEPS_PER_VOLT;

  Serial.println("-------------------------------------");
  Serial.printf("  Target        : %.2f V\n",  targetVoltage);
  Serial.printf("  Measured VW   : %.4f V\n",  measured);
  Serial.printf("  Error         : %+.4f V\n", error);
  Serial.printf("  Integral      : %+.4f\n",   integral);
  Serial.printf("  Step Correct  : %+d\n",     stepCorr);
  Serial.printf("  Wiper Step    : %d / %d\n", currentStep, MAX_CAL_STEP);
  Serial.printf("  Expected VW   : %.3f V\n",  expectedV);
  Serial.println("-------------------------------------");
}

void printHelp() {
  Serial.println("\n  Enter target voltage (0.0 to 5.0) and press Enter.");
  Serial.println("  Examples: 1.5   2.0   3.3   5.0");
  Serial.printf( "  Kp=%.2f  Ki=%.2f  Deadband=%.0fmV\n\n", Kp, Ki, DEADBAND_V * 1000);
}

// =============================================================
//  Serial Input Handler
// =============================================================
void handleSerialInput() {
  if (!Serial.available()) return;

  String input = Serial.readStringUntil('\n');
  input.trim();

  if (input.length() == 0) return;

  float requested = input.toFloat();

  // Validate range
  if (requested < MIN_VOLTAGE || requested > MAX_VOLTAGE) {
    Serial.printf("[ERROR] %.2f V is out of range. Enter 0.0 to 5.0 V.\n", requested);
    return;
  }

  targetVoltage = requested;
  integral      = 0.0f;  // reset integrator on new setpoint to avoid windup carry-over

  int expectedStep = (int)roundf(targetVoltage * STEPS_PER_VOLT);
  Serial.printf("\n[SET] Target: %.2f V → initial step estimate: %d\n\n",
                targetVoltage, expectedStep);
}

// =============================================================
//  Setup & Loop
// =============================================================
void setup() {
  Serial.begin(115200);
  delay(3000); // wait for monitor to connect

  Serial.println("\n========================================");
  Serial.println("   X9C103S PI Voltage Controller");
  Serial.println("   Calibration: 1V = 6 steps | Max 5V");
  Serial.println("========================================");

  // ── Pot pins ─────────────────────────────────────────────
  pinMode(CS,  OUTPUT);
  pinMode(INC, OUTPUT);
  pinMode(UD,  OUTPUT);
  digitalWrite(CS,  HIGH);
  digitalWrite(INC, LOW);
  digitalWrite(UD,  HIGH);

  // ── ADC ──────────────────────────────────────────────────
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  pinMode(ADC_PIN, INPUT);

  // ── Init pot at step 0 ───────────────────────────────────
  resetToZero();
  delay(200);

  printHelp();

  lastPITime = millis();
}

void loop() {
  // Handle user input any time it arrives
  handleSerialInput();

  // Run PI controller on fixed interval
  if (millis() - lastPITime >= (unsigned long)PI_INTERVAL_MS) {
    lastPITime = millis();

    if (targetVoltage > 0.0f || currentStep > 0) {
      runPI();
    }
  }
}
