#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

const int CS      = 14;
const int INC     = 12;
const int UD      = 13;
const int ADC_PIN = 35;        
Adafruit_ADS1115 ads;

const float ADS_MV_PER_BIT = 0.000125f;  
const float ZONE_HIGH = -0.850f;   // -850mV  upper bound (less negative)
const float ZONE_LOW  = -1.200f;   // -1200mV lower bound (more negative)

const int   MAX_STEPS      = 99;
const float STEPS_PER_VOLT = 6.0f;
const float MAX_VOLTAGE    = 5.0f;
const float MIN_VOLTAGE    = 0.0f;
const int   MAX_CAL_STEP   = 30;   // 5V * 6 steps/V

const float ADC_REF       = 3.3f;
const float ADC_MAX       = 4095.0f;
const float DIVIDER_RATIO = 2.0f;   // VW = V_gpio35 * 2
const int   OVERSAMPLE    = 64;

const float CAL_RAW_LOW    = 0.3651f;
const float CAL_ACTUAL_LOW = 0.52f;
const float CAL_RAW_HIGH    = 2.2822f;
const float CAL_ACTUAL_HIGH = 2.47f;

const int INC_PULSE_US  = 2;
const int CS_SETTLE_US  = 1;
const int STEP_DELAY_MS = 5;

// ── PI Controller Parameters ─────────────────────────────────
float Kp = 3.0f;
float Ki = 1.9f;

const float PI_INTERVAL_MS = 200.0f;
const float PI_DT          = PI_INTERVAL_MS / 1000.0f;

const float INTEGRAL_LIMIT = 2.0f;
const float DEADBAND_V = 0.020f;
const int MAX_STEP_CORRECTION = 3;
int   currentStep = 0;
float integral    = 0.0f;
unsigned long lastPITime = 0;

enum ZoneState { ZONE_NORMAL, ZONE_TOO_HIGH, ZONE_TOO_LOW };
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


float readADS() {
  int16_t raw   = ads.readADC_Differential_0_1();
  float voltage = (float)raw * 0.125f / 1000.0f;  // GAIN_ONE: 0.125mV/bit
  return voltage;
}

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

ZoneState classifyZone(float signal_v) {
  if (signal_v > ZONE_HIGH)      return ZONE_TOO_HIGH;  // above -850mV (less negative)
  else if (signal_v < ZONE_LOW)  return ZONE_TOO_LOW;   // below -1200mV (more negative)
  else                           return ZONE_NORMAL;
}

void runPI() {
  float signal_v = readADS();
  float vw       = readVW();
  ZoneState zone = classifyZone(signal_v);

  float error        = 0.0f;
  int   stepCorrection = 0;
  const char* zoneLabel = "NORMAL";

  if (zone == ZONE_NORMAL) {
    // Within range — hold position, gently decay integral
    integral  *= 0.95f;
    zoneLabel  = "NORMAL (hold)";
    printStatus(signal_v, vw, error, stepCorrection, zoneLabel);
    return;
  }

  if (zone == ZONE_TOO_HIGH) {
    error     = signal_v - ZONE_HIGH;  // positive error
    zoneLabel = "TOO HIGH - increasing step";
  }
  else {
    error     = signal_v - ZONE_LOW;   // negative error
    zoneLabel = "TOO LOW  - decreasing step";
  }

  // Deadband check
  if (fabs(error) < DEADBAND_V) {
    integral *= 0.95f;
    printStatus(signal_v, vw, 0.0f, 0, "DEADBAND");
    return;
  }

  // Accumulate integral with anti-windup
  integral += error * PI_DT;
  integral  = constrain(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

  // PI output in volts, converted to steps
  float output_v = (Kp * error) + (Ki * integral);
  stepCorrection = (int)roundf(output_v * STEPS_PER_VOLT);
  stepCorrection = constrain(stepCorrection, -MAX_STEP_CORRECTION, MAX_STEP_CORRECTION);

  int newStep = constrain(currentStep + stepCorrection, 0, MAX_CAL_STEP);
  setStep(newStep);

  printStatus(signal_v, vw, error, stepCorrection, zoneLabel);
}

void printStatus(float signal_v, float vw, float error, int stepCorr, const char* zoneLabel) {
  float vw_expected = (float)currentStep / STEPS_PER_VOLT;

  Serial.println("-------------------------------------");
  Serial.printf("  ADS Signal    : %+.4f V\n",  signal_v);
  Serial.printf("  Zone          : %s\n",        zoneLabel);
  Serial.printf("  Zone Bounds   : [%.3f V  to  %.3f V]\n", ZONE_LOW, ZONE_HIGH);
  Serial.printf("  Error         : %+.4f V\n",  error);
  Serial.printf("  Integral      : %+.4f\n",    integral);
  Serial.printf("  Step Correct  : %+d\n",      stepCorr);
  Serial.printf("  Wiper Step    : %d / %d\n",  currentStep, MAX_CAL_STEP);
  Serial.printf("  VW (GPIO35)   : %.4f V\n",   vw);
  Serial.printf("  VW (expected) : %.3f V\n",   vw_expected);
  Serial.println("-------------------------------------");
}


void setup() {
  Serial.begin(115200);
  delay(3000);

  pinMode(CS,  OUTPUT);
  pinMode(INC, OUTPUT);
  pinMode(UD,  OUTPUT);
  digitalWrite(CS,  HIGH);
  digitalWrite(INC, LOW);
  digitalWrite(UD,  HIGH);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  pinMode(ADC_PIN, INPUT);
  Serial.println("[ADC] GPIO35 configured — 12-bit, 11dB");

  Wire.begin();  // SDA=GPIO21, SCL=GPIO22 (ESP32 default)
  ads.begin();
  if (!ads.begin()) {
    Serial.println("[ERROR] ADS1115 not found! Check SDA/SCL wiring.");
    while (1);
  }
// In setup():
ads.setGain(GAIN_ONE);  // ±4.096V — safe for -1.2V


  resetToZero();
  delay(200);

  Serial.println("\n[READY] PI controller running...");
  Serial.printf("  Kp=%.2f  Ki=%.2f  Deadband=%.0fmV\n\n", Kp, Ki, DEADBAND_V * 1000);

  lastPITime = millis();
}

void loop() {
  if (millis() - lastPITime >= (unsigned long)PI_INTERVAL_MS) {
    lastPITime = millis();
    runPI();
  }
}
