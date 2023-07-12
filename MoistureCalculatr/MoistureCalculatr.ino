#include <Wire.h>
#include <Adafruit_SHT31.h>
#include <HX711.h>
#include <PID_v1_bc.h>
#include <Arduino.h>

/* SHT3xt
  SDA 21
  SCL 22
*/
//引腳定應 start
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
const int BUTTON_ENTER_PIN = 19;

// LED display pins
const int DIN_PIN = 17;
const int CLK_PIN = 18;
const int CS_PIN1 = 16;
const int CS_PIN2 = 15;


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

//重量相關
float setMoisturePercentage = 25;//損失重量百分比
float currentWeight = 0;
float moisturePercentage = 0;
float goWeight = 0;
float wetWeight = 0;
float dryWeight = 0;
HX711 scale;


// PID constants
const double Kp = 20, Ki = 4, Kd = 8;
double setPoint = 75, input, output;
PID myPID(&input, &output, &setPoint, Kp, Ki, Kd, DIRECT);
int WindowSize = 250;
unsigned long windowStartTime;

//溫濕度
float temperature, humidity ;
Adafruit_SHT31 sht31 = Adafruit_SHT31();


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


float calculateMoisture(float currentWeight) {
  //float dryWeight = currentWeight / 100 * dryWeightPer100g;
  //float wetWeight = currentWeight / 100 * wetWeightPer100g;
  float moisturePercentage = 0;
  //goWeight
  moisturePercentage = 100 - ((goWeight - currentWeight) / goWeight * 100);

  Serial.print("goWeight");
  Serial.println(goWeight);
  Serial.print("currentWeight:");
  Serial.println(currentWeight);
  Serial.print("moisturePercentage:");
  Serial.println(moisturePercentage);

  return moisturePercentage;
}
void init_gpio() {
  Serial.println("init GPIO...");
  //控制相關
  pinMode(HEATER_PIN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(NULL_PIN, OUTPUT);
  digitalWrite(NULL_PIN, LOW);

  //保護開關,未啟用
  pinMode(PROTECTION_SWITCH_PIN, INPUT);
  //按鈕相關
  //pinMode(BUTTON_PLUS_PIN, INPUT_PULLUP);
  //pinMode(BUTTON_MINUS_PIN, INPUT_PULLUP);
  //pinMode(BUTTON_ENTER_PIN, INPUT_PULLUP);

  Serial.println("GPIO ready...");
}


void init_interruptGPIO() {
  Serial.println("init interruptGPIO...");
    pinMode(buttonPlus.PIN, INPUT_PULLUP);
    attachInterruptArg(buttonPlus.PIN, isrPlus, &buttonPlus, FALLING);
    pinMode(buttonMinus.PIN, INPUT_PULLUP);
    attachInterruptArg(buttonMinus.PIN, isrMinus, &buttonMinus, FALLING);
    pinMode(buttonEnter.PIN, INPUT_PULLUP);
    attachInterruptArg(buttonEnter.PIN, isrEnter, &buttonEnter, FALLING);
  Serial.println("interruptGPIO ready...");
}
void test_gpio(int enable) {
  Serial.println("test GPIO...");
  while (enable) {
    digitalWrite(MOTOR_PIN, HIGH);
    delay(1000);
    digitalWrite(FAN_PIN, HIGH);
    delay(1000);
    digitalWrite(HEATER_PIN, HIGH);
    delay(1000);
    digitalWrite(HEATER_PIN, LOW);
    digitalWrite(FAN_PIN, LOW);
    digitalWrite(MOTOR_PIN, LOW);
    Serial.println("test okay...");
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
  sht31.begin(0x44);
  Serial.println("sht31 ready...");
}

void init_pid() {
  Serial.println("init pid...");
  myPID.SetOutputLimits(0, WindowSize);
  myPID.SetSampleTime(100);
  myPID.SetMode(AUTOMATIC);
  Serial.println("pid ready...");
}

// 定義輔助函數來控制硬體設備
void controlDevice(int pin, bool state) {
  digitalWrite(pin, state ? HIGH : LOW);
}

void controlHeater(bool state) {
  controlDevice(HEATER_PIN, state);
  if (state) Serial.println("HEATING");
}

void controlFan(bool state) {
  controlDevice(FAN_PIN, state);
  if (state) Serial.println("FAN WORKING");
}

void controlMotor(bool state) {
  controlDevice(MOTOR_PIN, state);
  if (state) Serial.println("MOTOR RUNNING");
  else Serial.println("MOTOR STOPPED");
}


void checkButton() {
  if (buttonPlus.pressed) {
    Serial.printf("buttonPlus has been pressed %u times\n", buttonPlus.numberKeyPresses);
    buttonPlus.pressed = false;
  }
  if (buttonMinus.pressed) {
    Serial.printf("buttonMinus has been pressed %u times\n", buttonMinus.numberKeyPresses);
    buttonMinus.pressed = false;
  }
  if (buttonEnter.pressed) {
    Serial.printf("buttonEnter has been pressed %u times\n", buttonEnter.numberKeyPresses);
    buttonEnter.pressed = false;
  }
}
// 主函數
void runPid() {
  if (run_mode == true) {
    controlMotor(true);
    Serial.println("PID running...");

    input = temperature;
    myPID.Compute();
    controlHeater(output >= (WindowSize / 2) && moisturePercentage >= MIN_MOISTURE_FOR_HEATER);

    if (temperature > MAX_TEMP_FOR_FAN) {
      controlFan(true);
    } else if (temperature < MIN_TEMP_FOR_FAN) {
      controlFan(false);
    }

    if (moisturePercentage < setMoisturePercentage) {
      run_mode = false;
      controlHeater(false);
      controlFan(false);
      controlMotor(false);
      Serial.println("Drying completed...");
      delay(1000);
    } else {
      Serial.println("Drying...");
    }
  } else {
    controlHeater(false);
    controlFan(false);
    controlMotor(false);
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
}
void print_info(int debugLevel) {
  if (debugLevel) {
    Serial.print("==========");
    Serial.print(String(millis()/1000));
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
  }
}

int getTempHumi() {

  // 讀取溫度和濕度
  temperature = sht31.readTemperature();
  humidity = sht31.readHumidity();
  return 0;
}

int getWeight() {
  // Read the current weight
  currentWeight = scale.get_units(10);
  moisturePercentage = calculateMoisture(currentWeight);
  return 0;
}

void checkProtectionSwitch() {
  if (digitalRead(PROTECTION_SWITCH_PIN) == HIGH) {
    digitalWrite(HEATER_PIN, LOW);
    digitalWrite(FAN_PIN, LOW);
    digitalWrite(MOTOR_PIN, LOW);
    displayError();
    Serial.println("Err");
    delay(500);
  }
}

void loop() {
  checkButton();
  checkProtectionSwitch();
  getTempHumi();
  getWeight();

  // Calculate the moisture percentage
  //moisturePercentage = 30; //for test PID runing
  // Update PID input
  /*
    if (run_mode == true) {
    // Motor control
    digitalWrite(MOTOR_PIN, HIGH);
    Serial.println("PID running...");
    input = temperature;
    myPID.Compute();
    // Heater control
    //沒有重量的時候也不運行
    //moisturePercentage=30;
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
    } else {
      Serial.println("Drying...");

    }
    } else {
    digitalWrite(HEATER_PIN, LOW);
    digitalWrite(FAN_PIN, LOW);
    digitalWrite(MOTOR_PIN, LOW);
    }*/

  runPid(); //溫度控制運算
  print_info(true); //true 打印資訊
  // Handle button input
  //handleButtonInput();
  readButtonInterrupt();
  //show_7seg(int(temperature*10),0);
  show_7seg(int(currentWeight * 1), 1, CS_PIN1);
  if (run_mode == 1) {
    show_7seg(int(moisturePercentage * 10), 2, CS_PIN2);
  } else {
    show_7seg(int(setPoint * 1), 0, CS_PIN2);
  }
  // Delay
  delay(100);
  //*test
  //testPulseEffect();測試脈衝影響
}

void testPulseEffect() {
  int xxx = 0;
  while (true) { // 確保此迴圈能一直執行，原本 while(0) 會導致程式碼塊不被執行
    show_7seg(int(xxx++ * 1), 1, CS_PIN1);
    if (xxx > 250) xxx = 0;
    Serial.println("加熱器 LOW");
    digitalWrite(HEATER_PIN, LOW);
    digitalWrite(FAN_PIN, LOW);
    digitalWrite(MOTOR_PIN, LOW);
    delay(250);
    Serial.println("加熱器 HIGH");
    digitalWrite(HEATER_PIN, HIGH);
    digitalWrite(FAN_PIN, HIGH);
    digitalWrite(MOTOR_PIN, HIGH);
    delay(250);
  }
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

void readButtonInterrupt(){
      if (buttonPlus.pressed) {
        if (setPoint <= 250) {
            setPoint += 5;
        }
        buttonPlus.pressed = false;
        Serial.println("++++++++");
    } 

    if (buttonMinus.pressed) {
        if (setPoint >= 5) {
            setPoint -= 5;
        }
        buttonMinus.pressed = false;
        Serial.println("-------");
    }

    if (buttonEnter.pressed) {
        goWeight = currentWeight;
        Serial.println("@@@@@@@");
        run_mode = !run_mode;
        myPID.SetMode(AUTOMATIC);
        buttonEnter.pressed = false;
    }

    Serial.print("Setpoint: ");
    Serial.println(setPoint);
  
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

  } else if (digitalRead(BUTTON_ENTER_PIN) == LOW) {
    lastButtonPress = now;
    goWeight = currentWeight;
    Serial.println("@@@@@@@");
    run_mode = !run_mode;
    myPID.SetMode(AUTOMATIC);
  }
  Serial.print("Setpoint: ");
  Serial.println(setPoint);

}


void init_7seg_old()
{
  Serial.println("init 7seg...");
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


  //第二組LCD屏幕初始化
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
  Serial.println("init ready...");

}


void init_7seg() {
  Serial.println("init 7seg...");
  digitalWrite(CS_PIN1, HIGH);
  digitalWrite(CS_PIN2, HIGH);
  pinMode(DIN_PIN, OUTPUT);
  pinMode(CS_PIN1, OUTPUT);
  pinMode(CS_PIN2, OUTPUT);
  pinMode(CLK_PIN, OUTPUT);
  //設置初始化
  setup_7seg(CS_PIN1);
  setup_7seg(CS_PIN2);
  Serial.println("7seg ready...");
}

void setup_7seg(int pin) {
  output7seg(0x0f, 0x0, 2, pin);
  output7seg(0x0c, 0x0, 2, pin);
  output7seg(0x0a, 0x02, 2, pin);
  output7seg(0x09, 0xFF, 2, pin);
  output7seg(0x01, 0x05, 2, pin);
  output7seg(0x02, 0x06, 2, pin);
  output7seg(0x03, 0x07, 2, pin);
  output7seg(0x04, 0x08, 2, pin);
  output7seg(0x05, 0x0F, 2, pin);
  output7seg(0x06, 0x0F, 2, pin);
  output7seg(0x07, 0x0F, 2, pin);
  output7seg(0x08, 0x0F, 2, pin);
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
  if (num < 0) num = 0;

  unsigned long num_temp = num;
  byte values[8];
  unsigned long dividers[8] = {10000000, 1000000, 100000, 10000, 1000, 100, 10, 1};

  // 使用迴圈進行除法和取餘數操作
  for (int i = 0; i < 8; ++i) {
    divide(num_temp, values[i], dividers[i]);
  }

  int dp_states[4] = {0};
  if (dp >= 1 && dp <= 4) {
    dp_states[dp - 1] = 1;
  }

  // 使用迴圈呼叫output7seg
  for (int i = 0; i < 4; ++i) {
    output7seg(i + 1, values[3 - i], dp_states[i], cs_num);
  }
}
/*
  void show_7seg_old(int num, int dp, int cs_num) {
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

  }*/
