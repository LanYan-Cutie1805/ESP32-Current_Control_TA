const int ntcPin = 33;
const int sensorPin = 34;
const int relayPin = 26;
const float Vref = 3.3;
const int ADC_res = 4095;

#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <INA226.h>
Adafruit_ADS1115 ads;
INA226 ina(0x40);

#define RELAY_OPEN  LOW
#define RELAY_CLOSE HIGH

const float tempTrip = 35.0;
const float tempReset = 32.5;  // hysteresis


bool tempFault = false;

// NTC parameters
const float R_fixed = 10000.0;   // 10k resistor
const float Beta = 3950.0;
const float T0_K = 298.2;         // 25°C in Kelvin
const float R0 = 10000.0;        // 10k at 25°C

// OCR 
float calibration = 2.12;   // adjust from your test
float pickupCurrent = 2.0;  // set your pickup
const int sampleCount = 200;

// State machine
enum State { NORMAL, TRIPPED, RECLOSE_WAIT, LOCKOUT };
State state = NORMAL;
int attemptCount = 0;
unsigned long tripTime = 0;


// Reclose settings
const int maxAttempts = 3;
const int recloseDelay = 300; // ms

const int CS  = 14;
const int INC = 12;
const int UD  = 13;

int position = 0;   // current wiper position (0–99)

// 🔧 force real position = 0
void resetToZero() {
  digitalWrite(CS, LOW);
  digitalWrite(UD, LOW);

  for (int i = 0; i < 150; i++) {
    digitalWrite(INC, LOW);
    delayMicroseconds(5);
    digitalWrite(INC, HIGH);
    delayMicroseconds(5);
  }

  digitalWrite(CS, HIGH);
  position = 0;
}

// 🎯 move to absolute position
void moveTo(int target) {
  int steps = target - position;
  if (steps == 0) return;

  bool dir = (steps > 0);
  steps = abs(steps);

  digitalWrite(CS, LOW);
  digitalWrite(UD, dir);

  for (int i = 0; i < steps; i++) {
    digitalWrite(INC, LOW);
    delayMicroseconds(5);
    digitalWrite(INC, HIGH);
    delayMicroseconds(5);
  }

  digitalWrite(CS, HIGH);
  position = target;
}


void setup() {
  Serial.begin(115200);
  pinMode(relayPin, OUTPUT); 
  digitalWrite(relayPin, HIGH);
    Wire.begin(); // default ESP32 SDA=21, SCL=22

  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS1115!");
    while (1);
  }
    
  // Set gain (important for voltage range)
  ads.setGain(GAIN_TWO);  
  // GAIN_ONE = ±2.048V (0.0625 mV)

  Serial.println("ADS1115 Initialized");


  pinMode(CS, OUTPUT);
  pinMode(INC, OUTPUT);
  pinMode(UD, OUTPUT);

  digitalWrite(CS, HIGH);
  digitalWrite(INC, HIGH);

  resetToZero(); // 🔥 important

  ina.setMaxCurrentShunt(0.8, 0.1);

  if (!ina.begin()) {
    Serial.println("INA226 not found!");
    while (1);
  }

  Serial.println("INA226 ready");
}

float readTemperature() {
  int samples = 20;
  float sum = 0;

  // Averaging ADC
  for (int i = 0; i < samples; i++) {
    sum += analogRead(ntcPin);
    delay(5);
  }

  float adcValue = sum / samples;

  // Convert ADC → voltage
  float Vout = (adcValue * Vref) / ADC_res;

  // Avoid division error
  if (Vout <= 0.01) return -100;

  // Calculate NTC resistance
  float R_ntc = R_fixed * ((Vref / Vout) - 1.0);

  // Beta equation
  float tempK = 1.0 / ((1.0 / T0_K) + (1.0 / Beta) * log(R_ntc / R0));

  float tempC = tempK - 273.15;

  return tempC;
}


float measureCurrent() {
  float offset = 0;
  const int sampleCount = 500;
  for (int i = 0; i < sampleCount; i++) {
    offset += analogRead(sensorPin);
    delayMicroseconds(200);
  }
  offset /= sampleCount;

  float offsetVoltage = (offset * Vref) / ADC_res;

  float sum = 0;

  for (int i = 0; i < sampleCount; i++) {
    int adcValue = analogRead(sensorPin);

    float voltage = (adcValue * Vref) / ADC_res;
    float centered = voltage - offsetVoltage;

    sum += centered * centered;
  }

  float Vrms = sqrt(sum / sampleCount);
  float current = Vrms * calibration;

  if (current < 0.05) current = 0;

  return current;
}


void loop() {
  float temp = readTemperature();

  if (!tempFault && temp >= tempTrip) {
    tempFault = true;
    Serial.println("TEMP TRIP!");
  }

  if (tempFault && temp <= tempReset) {
    tempFault = false;
    Serial.println(" | TEMP RESET!");
  }

  if (tempFault) {
    digitalWrite(relayPin, RELAY_OPEN);
  } else {
    digitalWrite(relayPin, RELAY_CLOSE);
  }

  int16_t adc0 = ads.readADC_SingleEnded(0);

  // Convert to voltage
  float voltage = adc0 * 0.0625 / 1000.0;

  // calibration
  float a = 1.03;
  float b = 0.01;

  float voltage_calibrated = a * voltage + b;

float current = measureCurrent();

  Serial.print("Current_Rec: ");
  Serial.println(current);

  switch (state) {

    case NORMAL:
      if (current > pickupCurrent) {
        Serial.println("TRIP!");
        digitalWrite(relayPin, LOW); // Open relay
        state = RECLOSE_WAIT;
        tripTime = millis();
        attemptCount = 0;
      }
      break;

    case RECLOSE_WAIT:
      if (millis() - tripTime >= recloseDelay) {
        Serial.println("Reclosing...");
        digitalWrite(relayPin, HIGH); // Close relay
        state = TRIPPED;
      }
      break;

    case TRIPPED:
      if (current > pickupCurrent) {
        attemptCount++;
        Serial.print("Reclose failed. Attempt: ");
        Serial.println(attemptCount);

        digitalWrite(relayPin, LOW); // Open again
        tripTime = millis();

        if (attemptCount >= maxAttempts) {
          Serial.println("LOCKOUT!");
          state = LOCKOUT;
        } else {
          state = RECLOSE_WAIT;
        }
      } else {
        // Success
        Serial.println("Reclose successful");
        state = NORMAL;
      }
      break;

    case LOCKOUT:
      digitalWrite(relayPin, LOW); // Keep open
      // Manual reset required (optional)
      break;
  }

  moveTo(1);  // adjust after calibration (~1V)
  //delay(10000);

  //moveTo(0);
  //delay(1000);


  float voltage_inj = ina.getBusVoltage_mV() / 1000.0; // convert mV → V
  float current_inj = ina.getCurrent_mA() / 1000.0;            

  // calibration factor
  float k = 1.25;

  current_inj *= k;

  Serial.print("Voltage_Inject: ");
  Serial.print(voltage_inj);
  Serial.print(" V | Current_Inject: ");
  Serial.print(current_inj);
  Serial.print(" mA");
  Serial.print(" | Voltage_Ref: ");
  Serial.print(voltage_calibrated);
  Serial.println(" V");
  Serial.print("Temp: ");
  Serial.print(temp);
  Serial.print(" | Temp_Trip: ");
  Serial.println(tempFault);
  delay (1000);
}