#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1115 ads; 

const int pwmPin = 33;
const int feedbackPin = 26;
const int pwmFreq =  5000;
const int pwmResolution = 10;
const int pwmMax = (1 << pwmResolution) - 1;
const int ADCmax = 4095;
unsigned long lastPrint = 0;
const int currentPin = 34;
const int ntcPin    = 32;

float Kp = 30.0;
float Ki = 0.1;
float integral = 0;
float dutyFloat = 0;

const float Vref      = 3.3f;
const int   ADC_RES   = 4095;
const float R_fixed   = 10000.0f;
const float Beta      = 3950.0f;
const float T0_K      = 298.2f;
const float R0        = 10000.0f;

float readTemperature() {
  float sum = 0;
  for (int i = 0; i < 20; i++) { sum += analogRead(ntcPin); delay(5); }
  float Vout = (sum / 20.0f * Vref) / ADC_RES;
  if (Vout <= 0.01f) return -100.0f;
  float R_ntc = R_fixed * ((Vref / Vout) - 1.0f);
  float tempK = 1.0f / ((1.0f / T0_K) + (1.0f / Beta) * log(R_ntc / R0));
  return tempK - 273.15f;
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  ledcAttach(pwmPin, pwmFreq, pwmResolution);
  ledcWrite(pwmPin, 0);
  analogReadResolution(12);

  Wire.begin(21,22);
  if (!ads.begin()) { Serial.println("ADS1115 NOT FOUND"); while (1); } Serial.println("ADS1115 OK");
  ads.begin(0x48);
  ads.setGain(GAIN_ONE);
  pinMode(ntcPin, INPUT);
}

void loop() {
  int16_t adc0 = ads.readADC_SingleEnded(1);
  float voltage = adc0 * 0.1875 / 1000.0 * 0.704;
  
  long sum = 0;
  for (int i = 0; i < 16; i++) {
    sum += analogRead(feedbackPin);
  }
  int rawAdc = sum / 16;
  static float filteredAdc = 0;
  filteredAdc = 0.95 * filteredAdc + 0.05 * rawAdc;
  int adc = (int)filteredAdc;
  float adcVoltage = adc * Vref / ADCmax;
  float actualVoltage = adcVoltage * 2.0 - 0.2;
  if (actualVoltage < 0.05) {
    actualVoltage = 0.0;
  }

  long currentSum = 0;
  for (int i = 0; i < 16; i++) {
    currentSum += analogRead(currentPin);
  }
  int currentRaw = currentSum / 16;
  float currentVoltage = currentRaw * Vref / ADCmax;
  float current_i = currentVoltage * 2.0 + 0.1;
  float realCurrent = (current_i - 0.1) * 0.54;
  float outputVoltage  = current_i * 20 - 6.7;

//  int duty = (actualVoltage / 5.0) * pwmMax;

  float error = 0;
  if (voltage > 0.85) {
    error = 1.025 - voltage;
  }
  else if (voltage < 1.21) {
    error = 1.025 - voltage;
  }
  else {
    error = 0;
  }

  integral += error;
  integral = constrain(integral, -200, 200);
  dutyFloat += Kp * error + Ki * integral;

  dutyFloat = constrain(dutyFloat, 0, pwmMax * 0.97);
  int duty = (int)dutyFloat;
  ledcWrite(pwmPin, duty);
  float dutyPercent = (100.0 * duty) / pwmMax;
  duty = constrain(duty, 0, pwmMax * 97 / 100);

  float temp = readTemperature();
  if (millis() - lastPrint >= 200) {
    lastPrint = millis();

    Serial.print("current_A: ");
    Serial.print(realCurrent, 2);
    Serial.print(" A | ");
    Serial.print("bus_voltage_V: ");
    Serial.print(outputVoltage, 2);
    Serial.print(" V | ");
    Serial.print("Duty: ");
    Serial.print(dutyPercent, 1);
    Serial.print(" % | ");
    Serial.print("electrode_V: ");
    Serial.print(voltage, 2);
    Serial.println(" V | ");
    Serial.print("temp: ");
    Serial.print(temp, 2);
    Serial.println(" (C)");
  }
}




