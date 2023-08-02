#include <Wire.h>
#include <Adafruit_SHT31.h>
#include <HX711.h>
#include <PID_v1_bc.h>
#include <Arduino.h>
#
/* SHT3xt
  SDA 21
  SCL 22
*/
//引腳定應 start
// Relay pins
const int HEATER_PIN_1 = 26;
const int FAN_PIN = 25; // 無風扇需控制則使用-1
const int HEATER_PIN_2 = 33; //two 25

const int MOTOR_PIN_1 = 32;
const int MOTOR_PIN_2 = 32;

const int NULL_PIN = 33;
// Load cell pins
const int LOADCELL_DOUT_PIN = 13;
const int LOADCELL_SCK_PIN = 14;

// Button pins
const int BUTTON_PLUS_PIN = 27;//原先23因為板上硬體設計卡在1.7會導致誤觸
const int BUTTON_MINUS_PIN = 5;
const int BUTTON_ENTER_PIN = 19;
const int BUTTON_PUSH_COUNT = 4; //防電子彈跳
// LED display pins
const int DIN_PIN = 17;
const int CLK_PIN = 18;
const int CS_PIN1 = 16;
const int CS_PIN2 = 15;
const int CS_PIN3 = 2;
const int CS_PIN4 = 4;

// GPIO pin for protection switch
const int PROTECTION_SWITCH_PIN = 34;
//引腳定應 end


//功能測試 預設0
int test_init = 0;
int test_loop = 0;


#define MAX_INPUT 8 // 最大的輸入數位
byte segments[MAX_INPUT]; // 每個數位的LED段的狀態
//int x = 0;
bool run_mode = 0;//系統工作狀態
bool bucketRun1, bucketRun2 = 0;
//重量相關
float setMoisturePercentage = 10;//設定損失重量百分比
float demoMoisturePercentage = 48.6;//廠商測試結果
//重量計算用
float show_percnetage_1, show_percnetage_2 = 0;
float currentWeight = 0;
float moisturePercentage = 0;
float totalWeight = 0;
float goWeight_1, goWeight_2 = 0;
float calculateMoisturePercentage_1, calculateMoisturePercentage_2 = 0;
float wetWeight = 0;
float dryWeight = 0;
HX711 scale;

// PID constants
const double Kp = 15, Ki = 4, Kd = 8;
double setPoint_1 = 75, input_1, output_1;
double setPoint_2 = 75, input_2, output_2;
PID myPID1(&input_1, &output_1, &setPoint_1, Kp, Ki, Kd, DIRECT);
PID myPID2(&input_2, &output_2, &setPoint_2, Kp, Ki, Kd, DIRECT);
int WindowSize = 250;
unsigned long windowStartTime;

//溫濕度
float temperature_1, humidity_1 ;
float temperature_2, humidity_2 ;
Adafruit_SHT31 sht31_44 = Adafruit_SHT31();
Adafruit_SHT31 sht31_45 = Adafruit_SHT31();

//GPIO interrupt
struct Button {
  const uint8_t PIN;
  uint32_t numberKeyPresses;
  bool pressed;
};

Button buttonPlus = {BUTTON_PLUS_PIN, 0, false};
Button buttonMinus = {BUTTON_MINUS_PIN, 0, false};
Button buttonEnter = {BUTTON_ENTER_PIN, 0, false};

void IRAM_ATTR isrPlus(void* arg) {
  Button* s = static_cast<Button*>(arg);
  s->numberKeyPresses += 1;
  s->pressed = true;
}

void IRAM_ATTR isrMinus(void* arg) {
  Button* s = static_cast<Button*>(arg);
  s->numberKeyPresses += 1;
  s->pressed = true;
}

void IRAM_ATTR isrEnter(void* arg) {
  Button* s = static_cast<Button*>(arg);
  s->numberKeyPresses += 1;
  s->pressed = true;
}

// 設置一些全域參數
const int MIN_TEMP_FOR_FAN = 28;
const int MAX_TEMP_FOR_FAN = 30;
const int MIN_MOISTURE_FOR_HEATER = 20;

//nowWeight 現在重量
//targetWeight原始重量
float calculateMoisture(float nowWeight, float initialWeight) {
  // 檢查參數是否合法
  if (nowWeight <= 0 || initialWeight <= 0 || nowWeight > initialWeight) {
    Serial.println("Err.");
    return -1;
  }

  // 計算出初始B成分的重量
  float initialB = initialWeight * 0.514; // 51.4 是 B 成分的初始百分比

  // 從當前重量中減去B成分的重量，得到A成分的現有重量
  float nowA = nowWeight - initialB;

  // 根據A成分的初始重量和現有重量，計算出A成分損失的百分比
  float lossPercent = (1 - (nowA / (initialWeight * 0.486))) * 100; // 48.6 是 A 成分的初始百分比
  Serial.print("lossPercent: ");
  Serial.println(lossPercent);
  return lossPercent;
}
/*
  float calculateMoisture(float nowWeight, float targetWeight) {
  if (targetWeight == 0) {
    Serial.println("Error: targetWeight cannot be zero");
    return -1; // 回傳一個錯誤值
  }
  float moisturePercentage = ((targetWeight - nowWeight) / targetWeight * 100);
  if (moisturePercentage < 0) {
    moisturePercentage = 0; // 如果計算出的濕度百分比為負，則設置為 0
    Serial.println("Error..");
  }
  Serial.println("$$$$");
  Serial.print("**** nowWeight:");
  Serial.println(nowWeight);
  Serial.print("**** targetWeight");
  Serial.println(targetWeight);
  Serial.print("**** moisturePercentage:");
  Serial.println(moisturePercentage);
  Serial.println("****");
  return moisturePercentage;
  }*/

void init_gpio() {
  Serial.println("init GPIO...");
  //控制相關
  pinMode(HEATER_PIN_1, OUTPUT);
  pinMode(HEATER_PIN_2, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);
  pinMode(MOTOR_PIN_1, OUTPUT);
  pinMode(NULL_PIN, OUTPUT);
  digitalWrite(NULL_PIN, LOW);
  //保護開關,未啟用
  pinMode(PROTECTION_SWITCH_PIN, INPUT);
  Serial.println("GPIO ready...");
}

void init_interruptGPIO() {
  Serial.println("init interruptGPIO...");
  pinMode(buttonEnter.PIN, INPUT_PULLUP);
  attachInterruptArg(buttonEnter.PIN, isrEnter, &buttonEnter, FALLING);
  pinMode(buttonPlus.PIN, INPUT_PULLUP);
  attachInterruptArg(buttonPlus.PIN, isrPlus, &buttonPlus, FALLING);
  pinMode(buttonMinus.PIN, INPUT_PULLUP);
  attachInterruptArg(buttonMinus.PIN, isrMinus, &buttonMinus, FALLING);
  Serial.println("interruptGPIO ready...");
}

void test_gpio(int enable) {
  Serial.println("test GPIO...");
  while (0) {
    controlFan(1);
    delay(1000);
    controlFan(0);
    delay(1000);

  }
  while (enable) {
    Serial.println("test motor...");
    digitalWrite(MOTOR_PIN_1, HIGH);
    delay(5000);
    digitalWrite(MOTOR_PIN_2, HIGH);
    delay(5000);
    digitalWrite(MOTOR_PIN_1, LOW);
    digitalWrite(MOTOR_PIN_2, LOW);
    delay(5000);
    Serial.println("test fans...");
    digitalWrite(FAN_PIN, HIGH);
    delay(5000);
    digitalWrite(FAN_PIN, LOW);
    delay(5000);
    Serial.println("test heater1...");
    digitalWrite(HEATER_PIN_1, HIGH);
    digitalWrite(HEATER_PIN_2, LOW);
    delay(5000);
    Serial.println("test heater2...");
    digitalWrite(HEATER_PIN_2, HIGH);
    digitalWrite(HEATER_PIN_1, LOW);
    delay(5000);
    Serial.println("test heaterall...");
    digitalWrite(HEATER_PIN_1, HIGH);
    digitalWrite(HEATER_PIN_2, HIGH);
    delay(5000);
    digitalWrite(HEATER_PIN_1, LOW);
    digitalWrite(HEATER_PIN_2, LOW);
    Serial.println("test okay...");
    delay(5000);
    //while (1) {}
  }
}
void init_scale() {
  Serial.println("init scale...");
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(105.5f);                      // this value is obtained by calibrating the scale with known weights; see the README for details
  scale.tare();                // reset the scale to 0
  Serial.println("scale ready...");
}
void init_sht31() {
  Serial.println("init sht31...");
  sht31_44.begin(0x44);
  sht31_45.begin(0x45);
  Serial.println("sht31 ready...");
}

void init_pid() {
  Serial.println("init pid...");
  myPID1.SetOutputLimits(0, WindowSize);
  myPID1.SetSampleTime(100);
  myPID1.SetMode(AUTOMATIC);
  myPID2.SetOutputLimits(0, WindowSize);
  myPID2.SetSampleTime(100);
  myPID2.SetMode(AUTOMATIC);
  Serial.println("pid ready...");
}

// 定義輔助函數來控制硬體設備
void controlDevice(int pin, bool state) {
  digitalWrite(pin, state ? HIGH : LOW);
}

void controlHeater1(bool state) {
  controlDevice(HEATER_PIN_1, state);
  if (state) Serial.println("HEATING 1");
}
void controlHeater2(bool state) {
  controlDevice(HEATER_PIN_2, state);
  if (state) Serial.println("HEATING 2 ");
}


void controlFan(bool state) {
  if (FAN_PIN != -1) {
    controlDevice(FAN_PIN, state);
    if (state) Serial.println("FAN WORKING");
    else Serial.println("FAN STOPPED");
  } else {
    Serial.println("DON'T HAVE FAN");
  }
}

void controlMotor1(bool state) {
  controlDevice(MOTOR_PIN_1, state);
  if (state) Serial.println("MOTOR RUNNING1");
  else Serial.println("MOTOR STOPPED1");
}


void controlMotor2(bool state) {
  if (MOTOR_PIN_1 == MOTOR_PIN_2) {
    Serial.println("MOTOR ONLY 1 MOTOR");
  } else {
    controlDevice(MOTOR_PIN_2, state);
    if (state) Serial.println("MOTOR RUNNING2");
    else Serial.println("MOTOR STOPPED2");
  }
}

void resetButton() {
  Serial.println("reset button state");
  buttonPlus.pressed = false;
  buttonMinus.pressed = false;
  buttonEnter.pressed = false;
  buttonPlus.numberKeyPresses = 0;
  buttonMinus.numberKeyPresses = 0;
  buttonEnter.numberKeyPresses = 0;
}
void checkButton() {
  if (buttonPlus.pressed) {
    Serial.printf("buttonPlus has been pressed %u times\n", buttonPlus.numberKeyPresses);
    if (buttonPlus.numberKeyPresses >= BUTTON_PUSH_COUNT) {
      bucketRun1 = !bucketRun1;
      Serial.println("buttonPlus++++++++");
    }
    if (bucketRun1) {
      goWeight_1 = getWeight();
      myPID1.SetMode(AUTOMATIC);
    }
    buttonPlus.numberKeyPresses = 0;
    buttonPlus.pressed = false;
  }
  if (buttonMinus.pressed) {
    Serial.printf("buttonMinus has been pressed %u times\n", buttonMinus.numberKeyPresses);
    if (buttonMinus.numberKeyPresses >= BUTTON_PUSH_COUNT) {
      bucketRun2 = !bucketRun2;
      Serial.println("buttonMinus-------");
    }
    if (bucketRun2) {
      goWeight_2 = getWeight();
      myPID2.SetMode(AUTOMATIC);
    }
    buttonMinus.numberKeyPresses = 0;
    buttonMinus.pressed = false;
  }
  if (buttonEnter.pressed) {
    Serial.printf("buttonEnter has been pressed %u times\n", buttonEnter.numberKeyPresses);
    if (buttonEnter.numberKeyPresses >= BUTTON_PUSH_COUNT) {
      Serial.println("buttonEnter@@@@@");
      run_mode = !run_mode;
    }
    buttonEnter.numberKeyPresses = 0;
    buttonEnter.pressed = false;
  }
}
// 兩組pid控制
void runPid() {
  if (bucketRun1 || bucketRun2) {
    controlMotor1(true);
  } else {
    controlMotor1(false);
  }
  if (bucketRun1 == true) {
    //controlMotor1(true);
    controlFan(true);
    Serial.println("1 PID running...");
    input_1 = temperature_1;
    myPID1.Compute();
    controlHeater1(output_1 >= (WindowSize / 2) );
    if (show_percnetage_1 < setMoisturePercentage) {//setMoisturePercentage=10
      bucketRun1 = false;
      controlHeater1(false);
      controlFan(false);
      //controlMotor1(false);
      Serial.println("Drying 1 completed...");
    } else {
      Serial.println("1 Drying...");
    }
  } else {
    controlHeater1(false);
    //controlFan(false);
    //controlMotor1(false);
  }
  if (bucketRun2 == true) {
    //controlMotor2(true);
    Serial.println("2 PID running...");
    input_2 = temperature_2;
    myPID2.Compute();
    controlHeater2(output_2 >= (WindowSize / 2) );
    if (show_percnetage_2 < setMoisturePercentage) {//setMoisturePercentage=10
      bucketRun2 = false;
      controlHeater2(false);
      //controlFan(false);
      //controlMotor2(false);
      Serial.println("Drying 2 completed...");
    } else {
      Serial.println("2 Drying...");
    }
  } else {
    controlHeater2(false);
    //controlFan(false);
    //controlMotor2(false);
  }
}

void print_info(int debugLevel) {
  if (debugLevel) {//
    Serial.print("==========");
    Serial.print(String(millis() / 1000));
    Serial.println("==========");
    Serial.print("Running: ");
    Serial.print(run_mode);
    Serial.print("\t bucketRun: ");
    Serial.print(bucketRun1);
    Serial.print("\t");
    Serial.println(bucketRun2);
    Serial.print("show_percne_1: ");
    Serial.print(show_percnetage_1);
    Serial.print("\t show_percne_2: ");
    Serial.println(show_percnetage_2);
    Serial.print("Temperature1: ");
    Serial.print(temperature_1);
    Serial.print("\t Humidity1: ");
    Serial.println(humidity_1);
    Serial.print("Temperature2: ");
    Serial.print(temperature_2);
    Serial.print("\t Humidity2: ");
    Serial.println(humidity_2);
    Serial.print("Current Weight: ");
    Serial.print(currentWeight);
    Serial.print("\t Total Weight: ");
    Serial.println(totalWeight);
    Serial.print("goWeight_1: ");
    Serial.print(goWeight_1);
    Serial.print("\t goWeight_2: ");
    Serial.println(goWeight_2);
    //calculateMoisturePercentage_1
    //Serial.print("Moisture Percentage: ");
    //Serial.println(moisturePercentage);
    Serial.print("calMoisture_1: ");
    Serial.print(calculateMoisturePercentage_1);
    Serial.print("\t calMoisture_2: ");
    Serial.println(calculateMoisturePercentage_2);
    Serial.print("PID Output1: ");
    Serial.print(output_1);
    Serial.print("\t PID setPoint1: ");
    Serial.println(setPoint_1);
    Serial.print("PID Output2: ");
    Serial.print(output_2);
    Serial.print("\t PID setPoint2: ");
    Serial.println(setPoint_2);
    Serial.print("windowStartTime: ");
    Serial.println(windowStartTime);
  }
}

int getTempHumi() {
  // 讀取溫度和濕度
  temperature_1 = sht31_44.readTemperature();
  humidity_1 = sht31_44.readHumidity();
  temperature_2 = sht31_45.readTemperature();
  humidity_2 = sht31_45.readHumidity();
  return 0;
}

float getWeight() {
  // Read the current weight
  currentWeight = scale.get_units(10);
  //moisturePercentage = calculateMoisture(currentWeight);
  return currentWeight;
}

void checkProtectionSwitch() {
  if (digitalRead(PROTECTION_SWITCH_PIN) == HIGH) {
    digitalWrite(HEATER_PIN_1, LOW);
    digitalWrite(HEATER_PIN_2, LOW);
    digitalWrite(FAN_PIN, LOW);
    digitalWrite(MOTOR_PIN_1, LOW);
    digitalWrite(MOTOR_PIN_2, LOW);
    Serial.println("Err");
    delay(500);
  }
}

int count = 0;
void oneBucket() {
  show_7seg(int(count++ * 1), 1, CS_PIN1);
  if (run_mode == 1) {
    show_7seg(int(moisturePercentage * 10), 2, CS_PIN2);
  } else {
    show_7seg(int(setPoint_1 * 1), 0, CS_PIN2);
  }
}
/*
  待機時顯示
  重量
  設定溫度

  啟動後顯示
  含水量(48.6%遞減 目標10%停止)
  當前溫度
*/
void twoBucket() {
  //CS_PIN1,2
  //bucketRun1 = 1; //for test
  if (bucketRun1) {
    //啟動
    calculateMoisturePercentage_1 = calculateMoisture(getWeight(), goWeight_1);// 計算損失水分百分比
    //calculateMoisturePercentage_1 = calculateMoisture(50, 100);//for test
    show_percnetage_1 = demoMoisturePercentage - (map(calculateMoisturePercentage_1, 0, 100, 0, demoMoisturePercentage));
    Serial.println(show_percnetage_1);
    show_7seg(int(show_percnetage_1 * 10), 2, CS_PIN1);
    show_7seg(int(temperature_1 * 10), 2, CS_PIN2);//顯示溫度
  } else {
    show_7seg(int(currentWeight * 1), 1, CS_PIN1);//沒觸發顯示重量
    show_7seg(int(setPoint_1 * 1), 1, CS_PIN2);//沒觸發顯示溫度

  }
  //CS_PIN3,4
  //bucketRun2=1;//for test
  if (bucketRun2) {
    //啟動
    calculateMoisturePercentage_2 = calculateMoisture(getWeight(), goWeight_2);
    //calculateMoisturePercentage_2 = calculateMoisture(90, 100);//for test
    show_percnetage_2 = demoMoisturePercentage - (map(calculateMoisturePercentage_2, 0, 100, 0, demoMoisturePercentage));
    Serial.println(show_percnetage_2);
    show_7seg(int(show_percnetage_2), 2, CS_PIN3);
    show_7seg(int(temperature_2 * 10), 2, CS_PIN4);//顯示溫度
  } else {
    show_7seg(int(currentWeight * 1), 1, CS_PIN3);//沒觸發顯示重量
    show_7seg(int(setPoint_2 * 1), 1, CS_PIN4);//沒觸發顯示溫度
  }
}

void setup() {
  Serial.begin(115200);
  init_gpio();
  init_interruptGPIO();
  test_gpio(test_init);
  init_scale();
  init_sht31();
  init_pid();
  init_7seg();
  setPoint_1 = 75;
  setPoint_2 = 75;
  resetButton();
  //calculateMoisture(-3.5,-2.5);
  //calculateMoisture(-3.5,-4.5);
  //while(1){}
}

void loop() {
  checkButton();
  //oneBucket();
  twoBucket();
  checkProtectionSwitch();
  getTempHumi();
  totalWeight = getWeight();

  runPid(); //溫度控制運算
  print_info(true); //true 打印資訊

  //test code
  //testPulseEffect();測試脈衝影響
}

void testPulseEffect() {
  int xxx = 0;
  while (true) { // 確保此迴圈能一直執行，原本 while(0) 會導致程式碼塊不被執行
    show_7seg(int(xxx++ * 1), 1, CS_PIN1);
    if (xxx > 250) xxx = 0;
    Serial.println("Heater LOW");
    digitalWrite(HEATER_PIN_1, LOW);
    digitalWrite(HEATER_PIN_2, LOW);
    digitalWrite(FAN_PIN, LOW);
    digitalWrite(MOTOR_PIN_1, LOW);
    digitalWrite(MOTOR_PIN_2, LOW);
    delay(250);
    Serial.println("Heater HIGH");
    digitalWrite(HEATER_PIN_1, HIGH);
    digitalWrite(HEATER_PIN_2, HIGH);
    digitalWrite(FAN_PIN, HIGH);
    digitalWrite(MOTOR_PIN_1, HIGH);
    digitalWrite(MOTOR_PIN_2, HIGH);
    delay(250);
  }
}



// Update the displayMoisturePercentage function
void displayMoisturePercentage(float moisturePercentage) {
  char buf[5];
  //snprintf(buf, sizeof(buf), "%03d", (int)moisturePercentage);
}




void init_7seg() {
  Serial.println("init 7seg...");
  digitalWrite(CS_PIN1, HIGH);
  digitalWrite(CS_PIN2, HIGH);
  digitalWrite(CS_PIN3, HIGH);
  digitalWrite(CS_PIN4, HIGH);
  pinMode(DIN_PIN, OUTPUT);
  pinMode(CS_PIN1, OUTPUT);
  pinMode(CS_PIN2, OUTPUT);
  pinMode(CS_PIN3, OUTPUT);
  pinMode(CS_PIN4, OUTPUT);
  pinMode(CLK_PIN, OUTPUT);
  //設置初始化
  setup_7seg(CS_PIN1);
  setup_7seg(CS_PIN2);
  setup_7seg(CS_PIN3);
  setup_7seg(CS_PIN4);
  Serial.println("7seg ready...");
  delay(500);
}

void setup_7seg(int pin) {
  output7seg(0x0f, 0x0, 2, pin);
  output7seg(0x0c, 0x0, 2, pin);
  output7seg(0x0a, 0x02, 2, pin);
  output7seg(0x09, 0xFF, 2, pin);
  output7seg(0x01, 0x00, 2, pin);
  output7seg(0x02, 0x00, 2, pin);
  output7seg(0x03, 0x00, 2, pin);
  output7seg(0x04, 0x01, 2, pin);
  output7seg(0x0b, 0x07, 2, pin);
  output7seg(0x0c, 0x01, 2, pin);
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


// 除法和取餘數操作
void divide(unsigned long &num, byte &result, unsigned long divider) {
  result = num / divider;
  num %= divider;
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
