#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <INA226.h>

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

const char* ssid = "RUNGKUT";
const char* password = "Don'tBeAnnoyingPlease";
const char* mqtt_server = "edx.petra.ac.id"; 
const char* mqtt_username = "quartz_rungkut"; 
const char* mqtt_password = "CathodicProtection"; 
const String topic = "ICCP/";

// ── Protection Pins ──────────────────────────────────────────
const int ntcPin    = 33;   // NTC thermistor ADC
const int sensorPin = 34;   // OCR current sensor ADC
const int relayPin  = 26;   // Relay output

// ── Digipot Pins ─────────────────────────────────────────────
const int CS      = 14;
const int INC     = 12;
const int UD      = 13;
const int ADC_PIN = 35;     // VW monitor via voltage divider

// ── External Devices ─────────────────────────────────────────
Adafruit_ADS1115 ads;
INA226 ina(0x40);

WiFiClient espClient;
PubSubClient client(espClient);

#define RELAY_OPEN  LOW
#define RELAY_CLOSE HIGH

// =============================================================
//  NTC TEMPERATURE CONFIG
// =============================================================
const float Vref      = 3.3f;
const int   ADC_RES   = 4095;
const float R_fixed   = 10000.0f;
const float Beta      = 3950.0f;
const float T0_K      = 298.2f;
const float R0        = 10000.0f;
const float TEMP_TRIP = 35.0f;
const float TEMP_RST  = 32.5f;
bool tempFault        = false;

// =============================================================
//  OCR CURRENT PROTECTION CONFIG
// =============================================================
float calibration    = 2.12f;
float pickupCurrent  = 2.0f;
const int OCR_SAMPLES = 500;

enum RelayState { NORMAL, TRIPPED, RECLOSE_WAIT, LOCKOUT };
RelayState relayState = NORMAL;
int   attemptCount    = 0;
unsigned long tripTime = 0;
const int maxAttempts  = 3;
const int recloseDelay = 300;

// =============================================================
//  INA226 CONFIG
// =============================================================
const float INA_K = 1.25f;  // current calibration factor

// =============================================================
//  DIGIPOT / PI CONTROLLER CONFIG
// =============================================================
const int   MAX_STEPS      = 99;
const float STEPS_PER_VOLT = 6.0f;
const float MAX_VOLTAGE    = 5.0f;
const float MIN_VOLTAGE    = 0.0f;
const int   MAX_CAL_STEP   = 30;

// GPIO35 ADC
const float ADC_REF       = 3.3f;
const float ADC_MAX       = 4095.0f;
const float DIVIDER_RATIO = 2.0f;
const int   OVERSAMPLE    = 64;

// GPIO35 two-point calibration
const float CAL_RAW_LOW     = 0.3651f;
const float CAL_ACTUAL_LOW  = 0.52f;
const float CAL_RAW_HIGH    = 2.2822f;
const float CAL_ACTUAL_HIGH = 2.47f;

// ADS1115 — differential A0-A1 for auto zone control
// GAIN_ONE: ±4.096V, 0.125mV/bit
const float ADS_GAIN_ONE_FACTOR = 0.000125f;

// Auto zone bounds
const float ZONE_HIGH = -0.850f;
const float ZONE_LOW  = -1.200f;

// Digipot timing
const int INC_PULSE_US  = 2;
const int CS_SETTLE_US  = 1;
const int STEP_DELAY_MS = 5;

// PI gains
float Kp = 3.0f;
float Ki = 1.9f;

const float PI_INTERVAL_MS = 200.0f;
const float PI_DT          = PI_INTERVAL_MS / 1000.0f;
const float INTEGRAL_LIMIT = 2.0f;

const float MANUAL_DEADBAND = 0.05f;
const float AUTO_DEADBAND   = 0.020f;
const int   MANUAL_MAX_CORR = 5;
const int   AUTO_MAX_CORR   = 3;

// =============================================================
//  STATE
// =============================================================
enum ControlMode { MODE_NONE, MODE_MANUAL, MODE_AUTO };
enum ZoneState   { ZONE_NORMAL, ZONE_TOO_HIGH, ZONE_TOO_LOW };

ControlMode controlMode  = MODE_NONE;
int         currentStep  = 0;
float       integral     = 0.0f;
float       targetVoltage = 0.0f;

unsigned long lastPITime      = 0;
unsigned long lastMonitorTime = 0;
const unsigned long MONITOR_INTERVAL = 1000;

// Last readings (shared between PI and display)
float lastSignal   = 0.0f;
float lastVW       = 0.0f;
float lastTemp     = 0.0f;
float lastOCR      = 0.0f;
float lastVInj     = 0.0f;
float lastIInj     = 0.0f;
float lastVRef     = 0.0f;

void reconnect() {
  while (!client.connected()) {
    Serial.println("Attempting MQTT connection");
    if (client.connect("ESP32_Client", mqtt_username, mqtt_password)) {
      Serial.println("MQTT connected");
    } else {
      delay(1000);
    }
  }
}

// =============================================================
//  DIGIPOT FUNCTIONS
// =============================================================
void sendPulse() {
  digitalWrite(INC, HIGH); delayMicroseconds(INC_PULSE_US);
  digitalWrite(INC, LOW);  delayMicroseconds(INC_PULSE_US);
}

void resetToZero() {
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
}

void setStep(int target) {
  target = constrain(target, 0, MAX_CAL_STEP);
  if (target == currentStep) return;
  int  steps = abs(target - currentStep);
  bool goUp  = (target > currentStep);
  digitalWrite(UD, goUp ? HIGH : LOW);
  delayMicroseconds(CS_SETTLE_US);
  digitalWrite(CS, LOW);
  delayMicroseconds(CS_SETTLE_US);
  for (int i = 0; i < steps; i++) { sendPulse(); delay(STEP_DELAY_MS); }
  digitalWrite(CS, HIGH);
  currentStep = target;
}

// =============================================================
//  SENSOR READING FUNCTIONS
// =============================================================
float applyCalibration(float raw_v) {
  float slope  = (CAL_ACTUAL_HIGH - CAL_ACTUAL_LOW) / (CAL_RAW_HIGH - CAL_RAW_LOW);
  float offset = CAL_ACTUAL_LOW - slope * CAL_RAW_LOW;
  return slope * raw_v + offset;
}

float readVW() {
  long sum = 0;
  for (int i = 0; i < OVERSAMPLE; i++) { sum += analogRead(ADC_PIN); delayMicroseconds(100); }
  float v_gpio = ((float)(sum / OVERSAMPLE) / ADC_MAX) * ADC_REF;
  return applyCalibration(v_gpio) * DIVIDER_RATIO;
}

float readADS_Zone() {
  // Differential A0-A1 for auto zone control (GAIN_ONE)
  int16_t raw = ads.readADC_Differential_0_1();
  return (float)raw * 0.125f / 1000.0f;
}

float readADS_Ref() {
  // Single-ended A0 for reference voltage (GAIN_TWO, original calibration)
  int16_t raw = ads.readADC_SingleEnded(0);
  float v = raw * 0.0625f / 1000.0f;
  return 1.03f * v + 0.01f;
}

float readTemperature() {
  float sum = 0;
  for (int i = 0; i < 20; i++) { sum += analogRead(ntcPin); delay(5); }
  float Vout = (sum / 20.0f * Vref) / ADC_RES;
  if (Vout <= 0.01f) return -100.0f;
  float R_ntc = R_fixed * ((Vref / Vout) - 1.0f);
  float tempK = 1.0f / ((1.0f / T0_K) + (1.0f / Beta) * log(R_ntc / R0));
  return tempK - 273.15f;
}

float measureCurrent() {
  float offset = 0;
  for (int i = 0; i < OCR_SAMPLES; i++) { offset += analogRead(sensorPin); delayMicroseconds(200); }
  float offsetV = (offset / OCR_SAMPLES * Vref) / ADC_RES;
  float sum = 0;
  for (int i = 0; i < OCR_SAMPLES; i++) {
    float v = (analogRead(sensorPin) * Vref) / ADC_RES;
    float c = v - offsetV;
    sum += c * c;
  }
  float current = sqrt(sum / OCR_SAMPLES) * calibration;
  return (current < 0.05f) ? 0.0f : current;
}

// =============================================================
//  PROTECTION LOGIC
// =============================================================
void runProtection(float temp, float current) {
  // Temperature protection
  if (!tempFault && temp >= TEMP_TRIP) tempFault = true;
  if  (tempFault && temp <= TEMP_RST)  tempFault = false;

  if (tempFault) {
    digitalWrite(relayPin, RELAY_OPEN);
    return;
  }

  // OCR state machine
  switch (relayState) {
    case NORMAL:
      digitalWrite(relayPin, RELAY_CLOSE);
      if (current > pickupCurrent) {
        digitalWrite(relayPin, RELAY_OPEN);
        relayState   = RECLOSE_WAIT;
        tripTime     = millis();
        attemptCount = 0;
      }
      break;

    case RECLOSE_WAIT:
      if (millis() - tripTime >= recloseDelay) {
        digitalWrite(relayPin, RELAY_CLOSE);
        relayState = TRIPPED;
      }
      break;

    case TRIPPED:
      if (current > pickupCurrent) {
        attemptCount++;
        digitalWrite(relayPin, RELAY_OPEN);
        tripTime = millis();
        relayState = (attemptCount >= maxAttempts) ? LOCKOUT : RECLOSE_WAIT;
      } else {
        relayState = NORMAL;
      }
      break;

    case LOCKOUT:
      digitalWrite(relayPin, RELAY_OPEN);
      break;
  }
}

// =============================================================
//  PI CONTROLLER
// =============================================================
ZoneState classifyZone(float v) {
  if      (v > ZONE_HIGH) return ZONE_TOO_HIGH;
  else if (v < ZONE_LOW)  return ZONE_TOO_LOW;
  else                    return ZONE_NORMAL;
}

void runManualPI() {
  lastVW = readVW();
  float error = targetVoltage - lastVW;
  int stepCorr = 0;
  if (fabs(error) < MANUAL_DEADBAND) {
    integral *= 0.95f;
  } else {
    integral += error * PI_DT;
    integral  = constrain(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
    float out = (Kp * error) + (Ki * integral);
    stepCorr  = constrain((int)roundf(out * STEPS_PER_VOLT), -MANUAL_MAX_CORR, MANUAL_MAX_CORR);
    setStep(constrain(currentStep + stepCorr, 0, MAX_CAL_STEP));
  }
}

void runAutoPI() {
  lastSignal = readADS_Zone();
  lastVW     = readVW();
  ZoneState zone = classifyZone(lastSignal);
  float error = 0.0f;
  int stepCorr = 0;
  if (zone == ZONE_NORMAL) {
    integral *= 0.95f;
  } else {
    error = (zone == ZONE_TOO_HIGH) ? (lastSignal - ZONE_HIGH) : (lastSignal - ZONE_LOW);
    if (fabs(error) >= AUTO_DEADBAND) {
      integral += error * PI_DT;
      integral  = constrain(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
      float out = (Kp * error) + (Ki * integral);
      stepCorr  = constrain((int)roundf(out * STEPS_PER_VOLT), -AUTO_MAX_CORR, AUTO_MAX_CORR);
      setStep(constrain(currentStep + stepCorr, 0, MAX_CAL_STEP));
    }
  }
}

// =============================================================
//  TERMINAL DISPLAY
// =============================================================
const char* relayStateStr() {
  if (tempFault) return "TEMP FAULT";
  switch (relayState) {
    case NORMAL:       return "CLOSED  [OK]";
    case TRIPPED:      return "TRIPPED";
    case RECLOSE_WAIT: return "RECLOSING...";
    case LOCKOUT:      return "!! LOCKOUT !!";
    default:           return "UNKNOWN";
  }
}

const char* controlModeStr() {
  switch (controlMode) {
    case MODE_MANUAL: return "MANUAL";
    case MODE_AUTO:   return "AUTO";
    default:          return "IDLE";
  }
}

ZoneState lastZone = ZONE_NORMAL;
const char* zoneStr() {
  switch (lastZone) {
    case ZONE_TOO_HIGH: return "TOO HIGH  [+]";
    case ZONE_TOO_LOW:  return "TOO LOW   [-]";
    default:            return "NORMAL    [=]";
  }
}

void printDashboard() {
  lastZone = classifyZone(lastSignal);
  float vwExpected = (float)currentStep / STEPS_PER_VOLT;

  Serial.println();
  Serial.println("  +===========================================+");
  Serial.println("  |       X9C103S SYSTEM DASHBOARD            |");
  Serial.println("  +===========================================+");

  // ── Control ──────────────────────────────────────────────
  Serial.println("  |  [ CONTROL ]                              |");
  Serial.printf( "  |  Mode               : %-26s|\n", controlModeStr());
  if (controlMode == MODE_MANUAL) {
    Serial.printf("  |  Target VW         : %-5.2f V            |\n", targetVoltage);
  } else if (controlMode == MODE_AUTO) {
    Serial.printf("  |  Zone Bounds       : %.3f V  to  %.3f V  |\n", ZONE_LOW, ZONE_HIGH);
    Serial.printf("  |  Electrode Referece: %+.4f V             |\n", lastSignal);
    Serial.printf("  |  Zone Status       : %-26s|\n", zoneStr());
  }
  Serial.printf(  "  |  Wiper Step        : %2d / %2d           |\n", currentStep, MAX_CAL_STEP);
  Serial.printf(  "  |  VW Expected       : %.3f V              |\n", vwExpected);

  Serial.println("  |-------------------------------------------|");

  // ── Monitoring ───────────────────────────────────────────
  Serial.println("  |  [ MONITORING ]                           |");
  Serial.printf( "  |  Temp         : %.2f C  (Trip @ %.1f C)   |\n", lastTemp, TEMP_TRIP);
  Serial.printf( "  |  Temp Fault   : %-26s|\n", tempFault ? "YES  [!!]" : "NO   [OK]");
  Serial.printf( "  |  OCR Current  : %.4f A  (Pickup @ %.1f A) |\n", lastOCR, pickupCurrent);
  Serial.printf( "  |  Relay        : %-26s|\n", relayStateStr());
  if (relayState == TRIPPED || relayState == RECLOSE_WAIT) {
    Serial.printf("  |  Reclose Att. : %d / %d                  |\n", attemptCount, maxAttempts);
  }

  Serial.println("  |-------------------------------------------|");

  // ── Injection ────────────────────────────────────────────
  Serial.println("  |  [ INJECTION (INA226) ]                   |");
  Serial.printf( "  |  Voltage Inj  : %.4f V                    |\n", lastVInj);
  Serial.printf( "  |  Current Inj  : %.4f A                    |\n", lastIInj);

  Serial.println("  +===========================================+");
  Serial.println("  | Type: '1'=Manual  '2'=Auto  'M'=Menu      |");
  Serial.println("  +===========================================+");
  Serial.println();
}

// =============================================================
//  MENU & SERIAL INPUT
// =============================================================
void printMenu() {
  Serial.println();
  Serial.println("  +===========================================+");
  Serial.println("  |       RECTIFIER CONTROL SYSTEM            |");
  Serial.println("  |        Select Operating Mode              |");
  Serial.println("  +===========================================+");
  Serial.println("  |  [1]  MANUAL — Set target voltage         |");
  Serial.println("  |  [2]  AUTO   — ADS1115 zone control       |");
  Serial.println("  |  [M]  Return to this menu anytime         |");
  Serial.println("  +===========================================+");
  Serial.println();
}

void handleSerialInput() {
  if (!Serial.available()) return;
  String input = Serial.readStringUntil('\n');
  input.trim();
  if (input.length() == 0) return;

  if (input == "M" || input == "m") {
    controlMode = MODE_NONE;
    integral    = 0.0f;
    printMenu();
    return;
  }

  if (controlMode == MODE_NONE) {
    if (input == "1") {
      controlMode = MODE_MANUAL;
      integral    = 0.0f;
      Serial.println("  [MANUAL] Enter target voltage (0.0 - 5.0 V):");
    } else if (input == "2") {
      controlMode = MODE_AUTO;
      integral    = 0.0f;
      Serial.printf("  [AUTO] Zone control active: %.3fV to %.3fV\n", ZONE_LOW, ZONE_HIGH);
    } else {
      Serial.println("  [ERROR] Type '1' for MANUAL or '2' for AUTO.");
    }
    return;
  }

  if (controlMode == MODE_MANUAL) {
    float req = input.toFloat();
    if (req < MIN_VOLTAGE || req > MAX_VOLTAGE) {
      Serial.printf("  [ERROR] %.2f V out of range. Enter 0.0 to 5.0 V.\n", req);
      return;
    }
    targetVoltage = req;
    integral      = 0.0f;
    Serial.printf("  [SET] Target: %.2f V  (est. step: %d)\n\n",
                  targetVoltage, (int)roundf(targetVoltage * STEPS_PER_VOLT));
  }
}

// =============================================================
//  SETUP
// =============================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  WiFi.begin(ssid, password);
  Serial.print("Connecting to Wifi.");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.println("Connected!");
  client.setServer(mqtt_server, 1883);


  // ── Protection pins ───────────────────────────────────────
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, RELAY_CLOSE);

  // ── Digipot pins ──────────────────────────────────────────
  pinMode(CS,  OUTPUT); digitalWrite(CS,  HIGH);
  pinMode(INC, OUTPUT); digitalWrite(INC, LOW);
  pinMode(UD,  OUTPUT); digitalWrite(UD,  HIGH);

  // ── ESP32 ADC (GPIO35) ────────────────────────────────────
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  pinMode(ADC_PIN, INPUT);

  // ── I2C Devices ───────────────────────────────────────────
  Wire.begin();

  if (!ads.begin()) {
    Serial.println("  [ERROR] ADS1115 not found!");
    while (1);
  }
  ads.setGain(GAIN_ONE);  // ±4.096V for differential zone reading

  ina.setMaxCurrentShunt(0.8, 0.1);
  if (!ina.begin()) {
    Serial.println("  [ERROR] INA226 not found!");
    while (1);
  }

  // ── Init digipot ──────────────────────────────────────────
  Serial.println("  [INIT] Winding potentiometer to step 0...");
  resetToZero();
  Serial.println("  [INIT] Done.");
  delay(200);

  printMenu();

  lastPITime      = millis();
  lastMonitorTime = millis();
}

// =============================================================
//  LOOP
// =============================================================
void loop() {
  handleSerialInput();

  if (!client.connected()) {
    reconnect();
  }
  client.loop();  

  // ── PI Controller (200ms) ─────────────────────────────────
  if (millis() - lastPITime >= (unsigned long)PI_INTERVAL_MS) {
    lastPITime = millis();

    if (controlMode == MODE_MANUAL && (targetVoltage > 0.0f || currentStep > 0)) {
      runManualPI();
    } else if (controlMode == MODE_AUTO) {
      runAutoPI();
    }
  }

  // ── Monitoring + Display (1000ms) ─────────────────────────
  if (millis() - lastMonitorTime >= MONITOR_INTERVAL) {
    lastMonitorTime = millis();

    // Read all sensors
    lastTemp = readTemperature();
    lastOCR  = measureCurrent();
    lastVRef = readADS_Ref();
    lastVW   = readVW();
    lastVInj = ina.getBusVoltage_mV() / 1000.0f;
    lastIInj = (ina.getCurrent_mA() / 1000.0f) * INA_K;

    if (controlMode == MODE_AUTO) {
      lastSignal = readADS_Zone();
    }

    // Run protection logic
    runProtection(lastTemp, lastOCR);

    // Print dashboard
    printDashboard();


    StaticJsonDocument<200> doc;
    doc["bus_voltage"] = lastVInj;
    doc["bus_current"] = lastIInj;
    doc["electrode"]   = lastVRef;
    doc["temp"]        = lastTemp;
    char payload[200];
    serializeJson(doc, payload);
    String combinedTopic = topic + "data";
    client.publish(combinedTopic.c_str(), payload);

  }
}