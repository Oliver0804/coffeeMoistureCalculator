#include <Wire.h>
#include <Adafruit_SHT31.h>
#include <HX711.h>
#include <LedControl.h>
#include <PID_v1_bc.h>

// Relay pins
const int HEATER_PIN = 19;
const int FAN_PIN = 18;
const int MOTOR_PIN = 17;

// Load cell pins
const int LOADCELL_DOUT_PIN = 3;
const int LOADCELL_SCK_PIN = 2;

// Button pins
const int BUTTON_PLUS_PIN = 5;
const int BUTTON_MINUS_PIN = 6;
const int BUTTON_CONFIRM_PIN = 7;

// LED display pins
const int DIN_PIN = 11;
const int CLK_PIN = 13;
const int CS_PIN = 10;

// GPIO pin for protection switch
const int PROTECTION_SWITCH_PIN = 4;

// PID constants
const double Kp = 2, Ki = 5, Kd = 1;
double setPoint = 85, input, output;

Adafruit_SHT31 sht31 = Adafruit_SHT31();
HX711 scale;
LedControl lc = LedControl(DIN_PIN, CLK_PIN, CS_PIN, 1);
//PID_v1_bc myPID(&input, &output, &setPoint, Kp, Ki, Kd, DIRECT);
PID myPID(&input, &output, &setPoint, Kp, Ki, Kd, DIRECT);

float calculateMoisture(float currentWeight, float dryWeight, float wetWeight) {
  float moisturePercentage = 0;
  moisturePercentage = (wetWeight - dryWeight) / dryWeight * 100;
  return moisturePercentage;
}

void setup() {
  pinMode(HEATER_PIN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(PROTECTION_SWITCH_PIN, INPUT);

  pinMode(BUTTON_PLUS_PIN, INPUT_PULLUP);
  pinMode(BUTTON_MINUS_PIN, INPUT_PULLUP);
  pinMode(BUTTON_CONFIRM_PIN, INPUT_PULLUP);

  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(2280.f); // calibration factor
  scale.tare();

  sht31.begin(0x44);

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255);
  myPID.SetSampleTime(1000);

  lc.shutdown(0, false);
  lc.setIntensity(0, 8);
  lc.clearDisplay(0);
}

void loop() {
  // Protection switch
  if (digitalRead(PROTECTION_SWITCH_PIN) == LOW) {
    digitalWrite(HEATER_PIN, LOW);
    digitalWrite(FAN_PIN, LOW);
    digitalWrite(MOTOR_PIN, LOW);
    displayError();
    return;
  }

  // Read temperature and humidity
  float temperature = sht31.readTemperature();
  float humidity = sht31.readHumidity();

  // Read the current weight
  float currentWeight = scale.get_units(10);

  // Calculate the moisture percentage
  float moisturePercentage = calculateMoisture(currentWeight, 0, 100);

  // Update PID input
  input = temperature;
  myPID.Compute();

  // Heater control
  if (output >= 5 && moisturePercentage >= 20) {
    digitalWrite(HEATER_PIN, HIGH);
  } else {
    digitalWrite(HEATER_PIN, LOW);
  }

  // Fan control
  if (temperature > 40) {
    digitalWrite(FAN_PIN, HIGH);
  } else if (temperature < 35) {
    digitalWrite(FAN_PIN, LOW);
  }

  // Motor control
  digitalWrite(MOTOR_PIN, HIGH);

  // 顯示溫度和含水率
 
  displayTemperature(temperature);
  displayMoisturePercentage(moisturePercentage);

  // 處理按鈕輸入
  handleButtonInput();

  // 延遲
  delay(1000);
}

void displayError() {
  // 在七段顯示器上顯示 "Err"
  lc.setChar(0, 3, 'E', false);
  lc.setChar(0, 2, 'r', false);
  lc.setChar(0, 1, 'r', false);
}

void displayTemperature(float temperature) {
  // 在七段顯示器上顯示溫度
  int temp = (int)temperature;
  lc.setDigit(0, 3, temp / 100, false);
  lc.setDigit(0, 2, (temp % 100) / 10, false);
  lc.setDigit(0, 1, temp % 10, false);
  lc.setDigit(0, 0, 0, false);
}

void displayMoisturePercentage(float moisturePercentage) {
  // 在七段顯示器上顯示含水率
  int moisture = (int)moisturePercentage;
  lc.setDigit(0, 7, moisture / 100, false);
  lc.setDigit(0, 6, (moisture % 100) / 10, false);
  lc.setDigit(0, 5, moisture % 10, false);
  lc.setDigit(0, 4, 0, false);
}

void handleButtonInput() {
  static unsigned long lastButtonPress = 0;
  unsigned long now = millis();

  if (now - lastButtonPress < 500) {
    return;
  }

  if (digitalRead(BUTTON_PLUS_PIN) == LOW) {
    setPoint += 5;
    lastButtonPress = now;
  } else if (digitalRead(BUTTON_MINUS_PIN) == LOW) {
    setPoint -= 5;
    lastButtonPress = now;
  } else if (digitalRead(BUTTON_CONFIRM_PIN) == LOW) {
    lastButtonPress = now;
    delay(5000);
    myPID.SetMode(AUTOMATIC);
  }
}
