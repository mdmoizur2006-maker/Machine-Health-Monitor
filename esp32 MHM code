
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <BluetoothSerial.h>
#include <math.h>

// ---------- DISPLAY ----------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ---------- SENSORS ----------
Adafruit_ADXL345_Unified adxl = Adafruit_ADXL345_Unified(12345);
BluetoothSerial SerialBT;

bool adxlConnected = false;
bool voltageConnected = false;
bool currentConnected = false;
bool ntcConnected = false;

// ---------- PINS ----------
#define VOLT_PIN 35
#define CURRENT_PIN 32
#define NTC_PIN 34
#define BUTTON_PIN 18
#define ALARM_PIN 2
#define OUTPUT_PIN 4

// ---------- CALIBRATION ----------
const float ADC_REFERENCE = 3.3;
const float VOLTAGE_FACTOR = 5.0;

const float CURRENT_ZERO_V = 1.65;
const float CURRENT_SENSITIVITY = 0.100;

const float NTC_FIXED_R = 10000.0;
const float NTC_R0 = 10000.0;
const float NTC_BETA = 3950.0;
const float NTC_T0 = 298.15;

// ---------- WARNING LIMITS ----------
const float MAX_TEMP = 40.0;
const float MAX_VIBRATION_G = 0.20;
const float VIBRATION_CHANGE_LIMIT = 0.10;

// ---------- SENSOR VALUES ----------
float voltage = 0;
float current = 0;
float temperature = NAN;
float vibration = 0;

float voltageSensorOutput = 0;
float currentSensorOutput = 0;

float previousVibration = 0;
float vibrationChange = 0;

bool vibrationInitialized = false;

float stableVibration = 0;
bool stableBaselineSaved = false;

unsigned long vibrationStartTime = 0;
unsigned long vibrationStableStart = 0;

const unsigned long VIBRATION_STARTUP_MS = 2000;
const unsigned long VIBRATION_STABLE_MS = 2000;
const float VIBRATION_CHANGE_PERCENT = 10.0;

bool vibrationWarning = false;


// ---------- SCREEN ----------
int screenNumber = 0;

// ---------- ALARM ----------
bool alarmActive = false;
bool alarmAcknowledged = false;
bool alarmState = false;

unsigned long lastBlink = 0;
unsigned long vibrationEventUntil = 0;

int highTemperatureCount = 0;
const int TEMP_CONFIRM_COUNT = 3;

// ---------- TIMERS ----------
unsigned long lastRead = 0;
unsigned long lastSend = 0;
unsigned long lastADXLCheck = 0;
unsigned long lastDisplay = 0;

const unsigned long SENSOR_INTERVAL = 200;
const unsigned long BLUETOOTH_INTERVAL = 500;
const unsigned long ADXL_CHECK_INTERVAL = 1000;
const unsigned long DISPLAY_INTERVAL = 200;

const unsigned long LONG_PRESS_MS = 1000;
const unsigned long VIBRATION_EVENT_MS = 2000;

// ---------- BUTTON ----------
unsigned long buttonDownTime = 0;
bool buttonWasDown = false;
bool longPressDone = false;

// ---------- READ ANALOG VOLTAGE ----------
float readAnalogVoltage(int pin) {
  uint32_t total = 0;

  for (int i = 0; i < 10; i++) {
    total += analogRead(pin);
    delayMicroseconds(200);
  }

  float raw = total / 10.0;
  return raw * ADC_REFERENCE / 4095.0;
}

// ---------- READ NTC TEMPERATURE ----------
float readNTC() {
  uint32_t total = 0;

  for (int i = 0; i < 10; i++) {
    total += analogRead(NTC_PIN);
    delayMicroseconds(200);
  }

  float raw = total / 10.0;

  // Open-circuit / out-of-range check
  if (raw < 5 || raw > 4090) {
    return NAN;
  }

  // Circuit:
  // 3.3V -- 10k fixed resistor -- ADC -- NTC -- GND

  float resistance =
      NTC_FIXED_R * raw / (4095.0 - raw);

  // Reject implausible readings
  if (resistance < 500.0 || resistance > 200000.0) {
    return NAN;
  }

  float kelvin = 1.0 /
      ((1.0 / NTC_T0) +
       log(resistance / NTC_R0) / NTC_BETA);

  float result = kelvin - 273.15;

  if (isnan(result) || isinf(result)) {
    return NAN;
  }

  return result;
}

// ---------- CHECK ADXL345 CONNECTION ----------
void checkADXLConnection() {
  // Default ADXL345 I2C address is 0x53.
  Wire.beginTransmission(0x53);
  bool present = (Wire.endTransmission() == 0);

  if (!present) {
    adxlConnected = false;
    vibrationInitialized = false;
    vibration = 0;
    vibrationChange = 0;
    vibrationEventUntil = 0;

    Serial.println("ADXL345 NOT CONNECTED");
    return;
  }

  // Initialize again if the sensor was disconnected
  // and has now been reconnected.
  if (!adxlConnected) {
    adxlConnected = adxl.begin();

    if (adxlConnected) {
      adxl.setRange(ADXL345_RANGE_4_G);
      vibrationInitialized = false;
      vibrationChange = 0;

      Serial.println("ADXL345 CONNECTED");
    }
  }
}

// ---------- READ ALL SENSORS ----------
void readSensors() {
  // Voltage sensor
  voltageSensorOutput = readAnalogVoltage(VOLT_PIN);
  voltageConnected = voltageSensorOutput > 0.10;

  if (voltageConnected) {
    voltage = voltageSensorOutput * VOLTAGE_FACTOR;
  } else {
    voltage = 0;
  }

  // Current sensor
  currentSensorOutput = readAnalogVoltage(CURRENT_PIN);

  currentConnected =
      currentSensorOutput > 0.10 &&
      currentSensorOutput < 3.20;

  if (currentConnected) {
    current =
        (currentSensorOutput - CURRENT_ZERO_V)
        / CURRENT_SENSITIVITY;
  } else {
    current = 0;
  }

  // Temperature sensor
  temperature = readNTC();
  ntcConnected = !isnan(temperature);

  if (!ntcConnected) {
    highTemperatureCount = 0;
  } else if (temperature > MAX_TEMP) {
    if (highTemperatureCount < TEMP_CONFIRM_COUNT) {
      highTemperatureCount++;
    }
  } else {
    highTemperatureCount = 0;
  }

  
  // ---------- ADXL345 VIBRATION ----------
  if (adxlConnected) {
    sensors_event_t event;
    adxl.getEvent(&event);

    float ax = event.acceleration.x / 9.80665;
    float ay = event.acceleration.y / 9.80665;
    float az = event.acceleration.z / 9.80665;

    float magnitude = sqrt(ax * ax + ay * ay + az * az);
    vibration = fabs(magnitude - 1.0);

    unsigned long now = millis();

    if (vibrationStartTime == 0) {
      vibrationStartTime = now;
      vibrationStableStart = now;
      stableBaselineSaved = false;
      vibrationInitialized = false;
      vibrationWarning = false;
      previousVibration = vibration;
    }

    if (!stableBaselineSaved) {
      if (now - vibrationStartTime >= VIBRATION_STARTUP_MS) {
        stableVibration = vibration;
        previousVibration = vibration;
        stableBaselineSaved = true;
        vibrationInitialized = true;
        vibrationStableStart = now;
        vibrationChange = 0;
        vibrationWarning = false;
      }
    } else {
      float difference = fabs(vibration - stableVibration);
      float reference = fmax(stableVibration, 0.05);

      vibrationChange = (difference / reference) * 100.0;

      if (vibrationChange > VIBRATION_CHANGE_PERCENT) {
        vibrationWarning = true;
        vibrationStableStart = 0;
      } else {
        vibrationWarning = false;

        if (vibrationStableStart == 0) {
          vibrationStableStart = now;
        }

        if (now - vibrationStableStart >= VIBRATION_STABLE_MS) {
          stableVibration = vibration;
          vibrationChange = 0;
        }
      }
    }
  } else {
    vibration = 0;
    vibrationChange = 0;
    vibrationInitialized = false;
    stableBaselineSaved = false;
    vibrationStartTime = 0;
    vibrationStableStart = 0;
    vibrationWarning = false;
  }
}


// ---------- ABNORMAL STATUS ----------

int abnormalSensorScreen() {
  if (adxlConnected &&
      stableBaselineSaved &&
      vibrationWarning) {
    return 0;
  }

  if (ntcConnected &&
      highTemperatureCount >= TEMP_CONFIRM_COUNT) {
    return 1;
  }

  return -1;
}


// ---------- UPDATE ALARM ----------

void updateAlarm() {
  bool warning = (abnormalSensorScreen() != -1);

  if (!warning || alarmAcknowledged) {
    alarmActive = false;
    alarmAcknowledged = warning ? true : false;
    alarmState = false;

    digitalWrite(ALARM_PIN, LOW);
    digitalWrite(OUTPUT_PIN, HIGH);
    return;
  }

  alarmActive = true;

  // Blink GPIO 2 during a warning.
  if (millis() - lastBlink >= 250) {
    lastBlink = millis();
    alarmState = !alarmState;

    digitalWrite(ALARM_PIN, alarmState ? HIGH : LOW);
  }

  // GPIO 4 OFF while the alarm warning is active.
  digitalWrite(OUTPUT_PIN, LOW);
}


// ---------- DRAW OLED ----------
void drawScreen() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);

  if (screenNumber == 0) {
    display.setTextSize(2);
    display.setCursor(0, 20);
    display.println("MACHINE");
    display.println("MONITOR");
  }

  else if (screenNumber == 1) {
    display.println("MACHINE MONITOR");
    display.println("----------------");
    display.println("VIBRATION");

    if (adxlConnected) {
      display.setTextSize(2);
      display.print(vibration, 3);
      display.println(" g");
      display.setTextSize(1);
    } else {
      display.println("NOT CONNECTED");
    }
  }

  else if (screenNumber == 2) {
    display.println("MACHINE MONITOR");
    display.println("----------------");
    display.println("TEMPERATURE");

    if (ntcConnected) {
      display.setTextSize(2);
      display.print(temperature, 1);
      display.println(" C");
      display.setTextSize(1);
    } else {
      display.println("NOT CONNECTED");
    }
  }

  else if (screenNumber == 3) {
    display.println("MACHINE MONITOR");
    display.println("----------------");
    display.println("VOLTAGE");

    if (voltageConnected) {
      display.setTextSize(2);
      display.print(voltage, 2);
      display.println(" V");
      display.setTextSize(1);
    } else {
      display.println("NOT CONNECTED");
    }
  }

  else if (screenNumber == 4) {
    display.println("MACHINE MONITOR");
    display.println("----------------");
    display.println("CURRENT");

    if (currentConnected) {
      display.setTextSize(2);
      display.print(current, 2);
      display.println(" A");
      display.setTextSize(1);
    } else {
      display.println("NOT CONNECTED");
    }
  }

  else if (screenNumber == 5) {
    int abnormal = abnormalSensorScreen();

    display.println("MACHINE MONITOR");
    display.println("----------------");
    display.println("ABNORMAL STATUS");

    if (abnormal == 0) {
      display.println("VIBRATION WARNING");
      display.print("Value: ");
      display.print(vibration, 3);
      display.println(" g");
      display.print("Change: ");
      display.print(vibrationChange, 3);
      display.println(" g");
    } else if (abnormal == 1) {
      display.println("HIGH TEMPERATURE");
      display.print("Value: ");
      display.print(temperature, 1);
      display.println(" C");
    } else {
      display.println("NO ABNORMALITY");
    }

    display.println();

    if (alarmAcknowledged) {
      display.println("ALARM ACKNOWLEDGED");
    } else if (alarmActive) {
      display.println("ALARM ACTIVE");
    } else {
      display.println("ALARM OFF");
    }
  }

  if (screenNumber >= 1 && screenNumber <= 4) {
    display.println();
    display.println("Click: Next Screen");
  } else if (screenNumber == 5) {
    display.println("Click: Next Screen");
  }

  display.display();
}

// ---------- BLUETOOTH DATA ----------

void sendBluetoothData() {
  SerialBT.printf("%.2f,%.2f,%.2f,%.2f\n",
                  vibration,
                  temperature,
                  voltage,
                  current);

  Serial.printf("BT DATA: %.2f,%.2f,%.2f,%.2f\n",
                vibration,
                temperature,
                voltage,
                current);
}


// ---------- ACKNOWLEDGE ALARM ----------
void acknowledgeAlarm() {
  alarmAcknowledged = true;
  alarmActive = false;
  alarmState = false;

  digitalWrite(ALARM_PIN, LOW);
}

// ---------- BUTTON ----------
void handleButton() {
  bool down = (digitalRead(BUTTON_PIN) == LOW);

  if (down && !buttonWasDown) {
    buttonDownTime = millis();
    longPressDone = false;
  }

  if (down && !longPressDone &&
      millis() - buttonDownTime >= LONG_PRESS_MS) {
    acknowledgeAlarm();
    longPressDone = true;
  }

  if (!down && buttonWasDown && !longPressDone) {
    screenNumber++;

    // Cycle through screens 1 to 5.
    if (screenNumber > 5) {
      screenNumber = 1;
    }

    drawScreen();
  }

  buttonWasDown = down;
}

// ---------- BLUETOOTH COMMANDS ----------
void handleBluetooth() {
  static String command = "";

  while (SerialBT.available()) {
    char c = SerialBT.read();

    if (c == '\n' || c == '\r') {
      command.trim();
      command.toUpperCase();

      if (command == "2" || command == "STOP") {
        acknowledgeAlarm();
        SerialBT.println("Alarm acknowledged");
      }

      command = "";
    } else if (command.length() < 20) {
      command += c;
    }
  }
}

// ---------- SETUP ----------
void setup() {
  Serial.begin(115200);

  // Ensure alarm LED starts OFF.
  pinMode(ALARM_PIN, OUTPUT);
digitalWrite(ALARM_PIN, LOW);

pinMode(OUTPUT_PIN, OUTPUT);
digitalWrite(OUTPUT_PIN, HIGH);

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  analogReadResolution(12);

  // ESP32 I2C pins
  Wire.begin(21, 22);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED NOT FOUND");
    while (true) {
      digitalWrite(ALARM_PIN, LOW);
      delay(100);
    }
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Machine Monitor");
  display.println("Starting...");
  display.display();

  // Initialize ADXL345
  adxlConnected = adxl.begin();

  if (adxlConnected) {
    adxl.setRange(ADXL345_RANGE_4_G);
    Serial.println("ADXL345 CONNECTED");
  } else {
    Serial.println("ADXL345 NOT CONNECTED");
  }

  // Bluetooth
  SerialBT.begin("MachineMonitor");
  Serial.println("Bluetooth: MachineMonitor");

  screenNumber = 0;
  drawScreen();

  delay(1000);
  screenNumber = 1;
}

// ---------- MAIN LOOP ----------
void loop() {
  handleButton();
  handleBluetooth();

  // Check if ADXL345 has been unplugged or reconnected.
  if (millis() - lastADXLCheck >= ADXL_CHECK_INTERVAL) {
    lastADXLCheck = millis();
    checkADXLConnection();
  }

  if (millis() - lastRead >= SENSOR_INTERVAL) {
    lastRead = millis();

    readSensors();
    updateAlarm();
  }

  if (millis() - lastDisplay >= DISPLAY_INTERVAL) {
    lastDisplay = millis();
    drawScreen();
  }

  if (millis() - lastSend >= BLUETOOTH_INTERVAL) {
    lastSend = millis();
    sendBluetoothData();
  }
}
