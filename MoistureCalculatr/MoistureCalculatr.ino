#include <Wire.h>
#include <Adafruit_SHT31.h>
#include <HX711.h>
#include <PID_v1_bc.h>
//SDA 21
//SCL 22

// Relay pins
const int HEATER_PIN = 26;
const int FAN_PIN = 25;
const int MOTOR_PIN = 32;
const int NULL_PIN = 33;
// Load cell pins
const int LOADCELL_DOUT_PIN = 13;
const int LOADCELL_SCK_PIN = 14;

// Button pins
const int BUTTON_PLUS_PIN = 23;
const int BUTTON_MINUS_PIN = 5;
const int BUTTON_CONFIRM_PIN = 19;

// LED display pins
const int DIN_PIN = 17;
const int CLK_PIN = 18;
const int CS_PIN1 = 16;
const int CS_PIN2 = 15;
#define MAX_INPUT 8 // 最大的輸入數位
byte segments[MAX_INPUT]; // 每個數位的LED段的狀態
int x = 0;
bool run_mode = 0;
float setMoisturePercentage = 25;

float currentWeight = 0;
float goWeight = 0;
float wetWeight = 0;
float dryWeight = 0;
// GPIO pin for protection switch
const int PROTECTION_SWITCH_PIN = 34;

// PID constants
const double Kp = 20, Ki = 4, Kd = 8;
double setPoint = 75, input, output;

Adafruit_SHT31 sht31 = Adafruit_SHT31();
HX711 scale;


//PID_v1_bc myPID(&input, &output, &setPoint, Kp, Ki, Kd, DIRECT);
PID myPID(&input, &output, &setPoint, Kp, Ki, Kd, DIRECT);

int WindowSize = 250;
unsigned long windowStartTime;

float calculateMoisture(float currentWeight) {
  //float dryWeight = currentWeight / 100 * dryWeightPer100g;
  //float wetWeight = currentWeight / 100 * wetWeightPer100g;
  float moisturePercentage = 0;
  //goWeight
  moisturePercentage = 100 - ((goWeight - currentWeight) / goWeight * 100);
  return moisturePercentage;
}

void setup() {

  Serial.begin(115200);
  Serial.println("init GPIO...");
  pinMode(HEATER_PIN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(NULL_PIN, OUTPUT);
  digitalWrite(NULL_PIN, LOW);
  pinMode(PROTECTION_SWITCH_PIN, INPUT);

  pinMode(BUTTON_PLUS_PIN, INPUT_PULLUP);
  pinMode(BUTTON_MINUS_PIN, INPUT_PULLUP);
  pinMode(BUTTON_CONFIRM_PIN, INPUT_PULLUP);

  Serial.println("init scale...");

  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

  scale.set_scale(105.5f);                      // this value is obtained by calibrating the scale with known weights; see the README for details
  scale.tare();                // reset the scale to 0

  Serial.println("init sht31...");
  sht31.begin(0x44);

  Serial.println("init pid...");
  myPID.SetOutputLimits(0, WindowSize);
  myPID.SetSampleTime(100);
  myPID.SetMode(AUTOMATIC);

  Serial.println("init 7seg...");
  initialise();

  Serial.println("init ok...");

}

void loop() {
  // Protection switch
  if (digitalRead(PROTECTION_SWITCH_PIN) == HIGH) {
    digitalWrite(HEATER_PIN, LOW);
    digitalWrite(FAN_PIN, LOW);
    digitalWrite(MOTOR_PIN, LOW);
    displayError();
    Serial.println("Err");
    delay(500);
    return;
  } else {

  }

  // Read temperature and humidity
  float temperature = sht31.readTemperature();
  float humidity = sht31.readHumidity();


  // Read the current weight
  currentWeight = scale.get_units(10);
  //moisturePercentage=20;
  // Calculate the moisture percentage
  float moisturePercentage = calculateMoisture(currentWeight);

  // Update PID input
  if (run_mode == true) {
    
    // Motor control
    digitalWrite(MOTOR_PIN, HIGH);
    
    Serial.println("PID running...");

    input = temperature;
    myPID.Compute();
    // Heater control

    if (output >= (WindowSize / 2) && moisturePercentage >= 20) {
      digitalWrite(HEATER_PIN, HIGH);
      Serial.println("HEATING");
    } else {
      digitalWrite(HEATER_PIN, LOW);
    }

    // Fan control
    if (temperature > 30) {
      digitalWrite(FAN_PIN, HIGH);
      Serial.println("FAN WORKING");
    } else if (temperature < 28) {
      digitalWrite(FAN_PIN, LOW);
    }
    if (moisturePercentage < setMoisturePercentage) {
      run_mode = 0;
      digitalWrite(HEATER_PIN, LOW);
      digitalWrite(FAN_PIN, LOW);
      digitalWrite(MOTOR_PIN, LOW);
      Serial.println("Drying completed...");
      delay(1000);
    }
  }
  /*
    if (millis() - windowStartTime > WindowSize)
    { //time to shift the Relay Window
    windowStartTime += WindowSize;
    }
    if (output < millis() - windowStartTime) {
    digitalWrite(HEATER_PIN, HIGH);
    }
    else {
    digitalWrite(HEATER_PIN, LOW);
    }
  */

  Serial.println("==========");
  Serial.print("Running: ");
  Serial.println(run_mode);
  Serial.print("Temperature: ");
  Serial.println(temperature);
  Serial.print("Humidity: ");
  Serial.println(humidity);
  Serial.print("Current Weight: ");
  Serial.println(currentWeight);
  Serial.print("Moisture Percentage: ");
  Serial.println(moisturePercentage);
  Serial.print("PID Output: ");
  Serial.println(output);
  Serial.print("windowStartTime: ");
  Serial.println(windowStartTime);

  // Handle button input
  handleButtonInput();

  //show_7seg(int(temperature*10),0);
  show_7seg(int(currentWeight * 1), 1, CS_PIN1);
  if (run_mode == 1) {
    show_7seg(int(moisturePercentage * 10), 2, CS_PIN2);
  } else {
    show_7seg(int(setPoint * 1), 0, CS_PIN2);
  }
  // Delay
  delay(100);
}

// Update the displayError function
void displayError() {
}

// Update the displayTemperature function
void displayTemperature(float temperature) {
  char buf[5];
  //snprintf(buf, sizeof(buf), "%03d", (int)temperature);
}

// Update the displayMoisturePercentage function
void displayMoisturePercentage(float moisturePercentage) {
  char buf[5];
  //snprintf(buf, sizeof(buf), "%03d", (int)moisturePercentage);
}

void handleButtonInput() {
  static unsigned long lastButtonPress = 0;
  unsigned long now = millis();

  if (now - lastButtonPress < 500) {
    return;
  }

  if (digitalRead(BUTTON_PLUS_PIN) == LOW) {
    if (setPoint <= 250) {
      setPoint += 5;
    }
    lastButtonPress = now;
    Serial.println("++++++++");

  } else if (digitalRead(BUTTON_MINUS_PIN) == LOW) {
    if (setPoint >= 5) {
      setPoint -= 5;
    }
    lastButtonPress = now;
    Serial.println("-------");

  } else if (digitalRead(BUTTON_CONFIRM_PIN) == LOW) {
    lastButtonPress = now;
    goWeight = currentWeight;
    Serial.println("@@@@@@@");
    run_mode = !run_mode;
    myPID.SetMode(AUTOMATIC);
  }


  Serial.print("Setpoint: ");
  Serial.println(setPoint);

}


void initialise()
{
  digitalWrite(CS_PIN1, HIGH);
  digitalWrite(CS_PIN2, HIGH);
  pinMode(DIN_PIN, OUTPUT);
  pinMode(CS_PIN1, OUTPUT);
  pinMode(CS_PIN2, OUTPUT);
  pinMode(CLK_PIN, OUTPUT);

  // For test mode (all digits on) set to 0x01. Normally we want this off (0x00)
  output7seg(0x0f, 0x0, 2, CS_PIN1);

  // Set all digits off initially
  output7seg(0x0c, 0x0, 2, CS_PIN1);

  // Set brightness for the digits to high(er) level than default minimum (Intensity Register Format)
  output7seg(0x0a, 0x02, 2, CS_PIN1);

  // Set decode mode for ALL digits to output actual ASCII chars rather than just
  // individual segments of a digit
  output7seg(0x09, 0xFF, 2, CS_PIN1);

  // Set first digit (right most) to '5'
  output7seg(0x01, 0x05, 2, CS_PIN1);

  // Set next digits to 8 7 6 (Code B Font)
  output7seg(0x02, 0x06, 2, CS_PIN1);
  output7seg(0x03, 0x07, 2, CS_PIN1);
  output7seg(0x04, 0x08, 2, CS_PIN1);

  // If first four digits not set it will display rubbish data (Code B Font) so use 'blank' from Register Data
  output7seg(0x05, 0x0F, 2, CS_PIN1);
  output7seg(0x06, 0x0F, 2, CS_PIN1);
  output7seg(0x07, 0x0F, 2, CS_PIN1);
  output7seg(0x08, 0x0F, 2, CS_PIN1);

  // Ensure ALL digits are displayed (Scan Limit Register)
  output7seg(0x0b, 0x07, 2, CS_PIN1);

  // Turn display ON (boot up = shutdown display)
  output7seg(0x0c, 0x01, 2, CS_PIN1);


  /////
  // For test mode (all digits on) set to 0x01. Normally we want this off (0x00)
  output7seg(0x0f, 0x0, 2, CS_PIN2);

  // Set all digits off initially
  output7seg(0x0c, 0x0, 2, CS_PIN2);

  // Set brightness for the digits to high(er) level than default minimum (Intensity Register Format)
  output7seg(0x0a, 0x02, 2, CS_PIN2);

  // Set decode mode for ALL digits to output actual ASCII chars rather than just
  // individual segments of a digit
  output7seg(0x09, 0xFF, 2, CS_PIN2);

  // Set first digit (right most) to '5'
  output7seg(0x01, 0x05, 2, CS_PIN2);

  // Set next digits to 8 7 6 (Code B Font)
  output7seg(0x02, 0x06, 2, CS_PIN2);
  output7seg(0x03, 0x07, 2, CS_PIN2);
  output7seg(0x04, 0x08, 2, CS_PIN2);

  // If first four digits not set it will display rubbish data (Code B Font) so use 'blank' from Register Data
  output7seg(0x05, 0x0F, 2, CS_PIN2);
  output7seg(0x06, 0x0F, 2, CS_PIN2);
  output7seg(0x07, 0x0F, 2, CS_PIN2);
  output7seg(0x08, 0x0F, 2, CS_PIN2);

  // Ensure ALL digits are displayed (Scan Limit Register)
  output7seg(0x0b, 0x07, 2, CS_PIN2);

  // Turn display ON (boot up = shutdown display)
  output7seg(0x0c, 0x01, 2, CS_PIN2);
}


void output7seg(byte address, byte data, byte dpPosition, int num_cs)
{
  digitalWrite(num_cs, LOW);

  // 如果dpPosition有效，顯示小數點
  if (dpPosition > 0 && dpPosition < MAX_INPUT) {
    // 設定小數點對應位置的段
    data |= B10000000;
  } else if (dpPosition == 0xFF) {
    // 傳送兩個位元組（16位）
    // 參數：shiftOut(dataPin, clockPin, bitOrder, value)
    data = 0x0F;
    shiftOut(DIN_PIN, CLK_PIN, MSBFIRST, address);
    shiftOut(DIN_PIN, CLK_PIN, MSBFIRST, data);

    digitalWrite(num_cs, HIGH);
    return ;
  }

  // 傳送兩個位元組（16位）
  // 參數：shiftOut(dataPin, clockPin, bitOrder, value)
  shiftOut(DIN_PIN, CLK_PIN, MSBFIRST, address);
  shiftOut(DIN_PIN, CLK_PIN, MSBFIRST, data);

  digitalWrite(num_cs, HIGH);
}

void test() {
  static unsigned long count = 0;
  unsigned long remainder;

  byte tenmillions = count / 10000000;
  remainder = count % 10000000;

  byte millions = remainder / 1000000;
  remainder = remainder % 1000000;

  byte hundredthou = remainder / 100000;
  remainder = remainder % 100000;

  byte tenthou = remainder / 10000;
  remainder = remainder % 10000;

  byte thou = remainder / 1000;
  remainder = remainder % 1000;

  byte hundreds = remainder / 100;
  remainder = remainder % 100;

  byte tens = remainder / 10;
  remainder = remainder % 10;

  //output(0x08, tenmillions); // million
  //output(0x07, millions); // million
  //output(0x06, hundredthou); // hundred thou
  //output(0x05, tenthou); // ten thou
  output7seg(0x01, thou, 0, CS_PIN1); // thousands
  output7seg(0x02, hundreds, 0, CS_PIN1); // hundreds
  output7seg(0x03, tens, 1, CS_PIN1); // tens
  output7seg(0x04, remainder, 0, CS_PIN1); // units
  //delay(50);

  output7seg(0x01, thou, 0, CS_PIN2); // thousands
  output7seg(0x02, hundreds, 0, CS_PIN2); // hundreds
  output7seg(0x03, tens, 0, CS_PIN2); // tens
  output7seg(0x04, remainder, 0, CS_PIN2); // units

}


void show_7seg(int num, int dp, int cs_num) {
  unsigned long rem;
  if (num < 0)num = 0;
  byte tenmillions = num / 10000000;
  rem = num % 10000000;

  byte millions = rem / 1000000;
  rem = rem % 1000000;

  byte hundredthou = rem / 100000;
  rem = rem % 100000;

  byte tenthou = rem / 10000;
  rem = rem % 10000;

  byte thou = rem / 1000;
  rem = rem % 1000;

  byte hundreds = rem / 100;
  rem = rem % 100;

  byte tens = rem / 10;
  rem = rem % 10;
  int a = 0, b = 0, c = 0, d = 0;
  switch (dp) {
    case 4:
      a = 1;
      break;
    case 3:
      b = 1;
      break;
    case 2:
      c = 1;
      break;
    case 1:
      d = 1;
      break;
  }

  output7seg(0x01, thou, a, cs_num); // thousands
  output7seg(0x02, hundreds, b, cs_num); // hundreds
  output7seg(0x03, tens, c, cs_num); // tens
  output7seg(0x04, rem, d, cs_num); // units


}
