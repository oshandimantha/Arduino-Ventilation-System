// Arduino Ventilation System with L298N Motor Driver
// Features:
// - Wind speed measurement using optical encoder
// - Temperature & humidity monitoring using DHT11
// - Fan1 (relay) control based on wind speed (rpm)
// - Fan2 (motor via L298N) speed control directly proportional to temperature
// - 16×2 LCD display showing fan statuses

#include <LiquidCrystal.h>
#include <DHT.h>

// Pin Definitions
#define DHT_PIN         A0      // DHT11 data pin
#define DHTTYPE         DHT11
#define ENCODER_PIN     2       // Optical encoder (INT0)

// Fan1: relay-controlled mechanical fan
const int FAN1_RELAY_PIN = 12;
const unsigned int FAN1_RPM_THRESHOLD = 20; // Turn F1 ON above 20 RPM

// Fan2: L298N Motor Driver (electric fan)
const int IN3_PIN = 8;
const int IN4_PIN = 7;
const int ENB_PIN = 6; // PWM output

// LCD Pins (avoid conflicts)
const int LCD_RS = 13;
const int LCD_EN = 11;
const int LCD_D4 = 10;
const int LCD_D5 = 9;
const int LCD_D6 = A2;
const int LCD_D7 = A3;

// Constants
const float         MAX_TEMP        = 40.0;
const unsigned long SAMPLE_INTERVAL = 2000;
const unsigned long DEBOUNCE_US     = 20000UL;

// Globals
volatile unsigned long pulseCount       = 0;
volatile unsigned long lastDebounceTime = 0;
unsigned long          lastSampleTime   = 0;
float                  temperature      = 0, humidity = 0;
unsigned int           rpm              = 0;

LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
DHT dht(DHT_PIN, DHTTYPE);

void countPulse() {
  unsigned long now = micros();
  if (now - lastDebounceTime > DEBOUNCE_US) {
    pulseCount++;
    lastDebounceTime = now;
  }
}

void setup() {
  Serial.begin(9600);
  lcd.begin(16, 2);
  lcd.print(F("Vent Sys Init"));
  delay(1000);

  // Pin modes
  pinMode(FAN1_RELAY_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);
  pinMode(ENB_PIN, OUTPUT);
  pinMode(ENCODER_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), countPulse, FALLING);
  dht.begin();
  lastSampleTime = millis();
}

void loop() {
  if (millis() - lastSampleTime < SAMPLE_INTERVAL) return;
  lastSampleTime = millis();

  // Compute RPM
  detachInterrupt(digitalPinToInterrupt(ENCODER_PIN));
  rpm = (pulseCount * 30) / 2;
  pulseCount = 0;
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), countPulse, FALLING);

  // Read DHT11
  humidity    = dht.readHumidity();
  temperature = dht.readTemperature();
  if (isnan(humidity) || isnan(temperature)) {
    Serial.println(F("DHT read failed"));
  }

  // === Fan1 control by RPM threshold ===
  bool fan1On = (rpm < FAN1_RPM_THRESHOLD);
  digitalWrite(FAN1_RELAY_PIN, fan1On ? HIGH : LOW);

  // === Fan2 control: PWM proportional to temperature 0-40°C → 0-100% ===
  int tempC = constrain((int)temperature, 0, (int)MAX_TEMP);
  int pwmPercent = map(tempC, 0, (int)MAX_TEMP, 0, 100);
  int pwmVal = map(pwmPercent, 0, 100, 0, 255);
  analogWrite(ENB_PIN, pwmVal);
  digitalWrite(IN3_PIN, HIGH); // forward direction
  digitalWrite(IN4_PIN, LOW);

  // Serial log
  Serial.print(F("Temp:")); Serial.print(temperature);
  Serial.print(F("C Hum:")); Serial.print(humidity);
  Serial.print(F("% RPM:")); Serial.print(rpm);
  Serial.print(F(" F1:")); Serial.print(fan1On ? "ON" : "OFF");
  Serial.print(F(" F2:")); Serial.print(pwmPercent);
  Serial.println(F("%"));

  // Update LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("F1:"));
  lcd.print(fan1On ? F("ON ") : F("OFF"));
  lcd.setCursor(0, 1);
  lcd.print(F("F2:"));
  lcd.print(pwmPercent);
  lcd.print('%');
}
