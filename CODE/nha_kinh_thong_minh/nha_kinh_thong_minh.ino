/*********************************************************
  CHƯƠNG TRÌNH ARDUINO UNO R3 - ESP8266 GIAO TIẾP UART   
            DỰ ÁN NHÀ KÍNH THỦY SINH
  KHOA HỌC KĨ THUẬT DÀNH CHO CẤP TRUNG HỌC TỈNH PHÚ YÊN 
                  NĂM HỌC 2023 - 2024
                  Created by Doan Ha 
**********************************************************/

// ============================================================
// 1. INCLUDES
// ============================================================
#include <Wire.h>
#include <DFRobot_DHT20.h>
#include <MQ135.h>
#include <Servo.h>
#include <LiquidCrystal_I2C.h>
#include <MFRC522.h>
#include <SPI.h>
#include <SerialCommand.h>

// ============================================================
// 2. PIN DEFINITIONS
// ============================================================
// -- Công tắc hành trình --
#define PIN_CT_DONG  5   // Công tắc đóng mái
#define PIN_CT_MO    7   // Công tắc mở mái

// -- Motor mái trượt (L298N) --
#define PIN_MOTOR_IN1  2
#define PIN_MOTOR_IN2  4

// -- Thiết bị điều khiển --
#define PIN_TUOICAY    6
#define PIN_FAN        8
#define PIN_LED       11
#define PIN_PHUNSUONG 12
#define PIN_LOCK      10

// -- Cảm biến --
#define PIN_CB_DAD  A0  // Cảm biến độ ẩm đất (analog)
#define PIN_CB_KK   A1  // Cảm biến không khí MQ135 (analog)
#define PIN_CB_AS   A3  // Cảm biến ánh sáng (analog)
#define PIN_CB_MUA   3  // Cảm biến mưa (digital)

// ============================================================
// 3. CONSTANTS & CONFIGURATION
// ============================================================
// -- Thời gian đọc cảm biến (ms) --
static const unsigned long SENSOR_INTERVAL   = 2000;  // 2 giây
static const unsigned long LCD_INTERVAL      = 3000;  // 3 giây

// -- Hysteresis (vùng chết) để tránh rung relay --
static const float HYSTERESIS_DAD  = 10.0;  // ±10% cho độ ẩm đất
static const float HYSTERESIS_TEMP =  3.0;  // ±3°C cho nhiệt độ
static const float HYSTERESIS_PPM  = 30.0;  // ±30 ppm cho khí
static const float HYSTERESIS_AS   = 15.0;  // ±15% cho ánh sáng

// -- Số mẫu lọc trung vị --
#define MEDIAN_SAMPLES 5

// ============================================================
// 4. OBJECTS
// ============================================================
SerialCommand cmd;
DFRobot_DHT20 dht;
MQ135 sensorKK(PIN_CB_KK);
LiquidCrystal_I2C lcd(0x27, 16, 2);  // Địa chỉ I2C phổ biến: 0x27

// ============================================================
// 5. GLOBAL VARIABLES
// ============================================================
// -- Giá trị cảm biến --
float valDoAmDat  = 0;   // Phần trăm độ ẩm đất (0-100%)
float valNhietDo  = 0;   // Nhiệt độ (°C)
float valDoAmKK   = 0;   // Độ ẩm không khí (%)
float valAnhSang  = 0;   // Ánh sáng (0-100%)
float valPPM      = 0;   // Chất lượng không khí (ppm)
int   valMua      = 1;   // Cảm biến mưa (1 = không mưa, 0 = mưa)

// -- Ngưỡng hiệu chỉnh (có thể thay đổi từ App) --
float threshDAD = 15.0;   // Ngưỡng độ ẩm đất (%)
float threshND  = 30.0;   // Ngưỡng nhiệt độ (°C)
float threshKK  = 150.0;  // Ngưỡng khí độc (ppm)
float threshAS  = 50.0;   // Ngưỡng ánh sáng (%)

// -- Cờ trạng thái thiết bị (1 = đang bật thủ công từ App) --
bool flagManualLed       = false;
bool flagManualFan       = false;
bool flagManualTuoiCay   = false;
bool flagManualPhunSuong = false;
bool flagAutoMode        = true;
bool flagManualNocNha    = false;

// -- Trạng thái mái trượt --
enum RoofState { ROOF_IDLE, ROOF_OPENING, ROOF_CLOSING };
RoofState roofState = ROOF_IDLE;

// -- Timer không blocking --
unsigned long lastSensorRead = 0;
unsigned long lockStartTime  = 0;
bool lockActive = false;
int  lockPhase  = 0;  // 0=idle, 1=welcome, 2=denied

// ============================================================
// 6. HELPER FUNCTIONS
// ============================================================

/**
 * Đọc giá trị analog với bộ lọc trung vị (median filter)
 * để loại bỏ nhiễu spike. Đọc MEDIAN_SAMPLES lần, sắp xếp,
 * lấy giá trị giữa.
 */
int readAnalogMedian(int pin) {
  int samples[MEDIAN_SAMPLES];
  for (int i = 0; i < MEDIAN_SAMPLES; i++) {
    samples[i] = analogRead(pin);
    delayMicroseconds(100);
  }
  // Insertion sort (mảng nhỏ, nhanh hơn qsort)
  for (int i = 1; i < MEDIAN_SAMPLES; i++) {
    int key = samples[i];
    int j = i - 1;
    while (j >= 0 && samples[j] > key) {
      samples[j + 1] = samples[j];
      j--;
    }
    samples[j + 1] = key;
  }
  return samples[MEDIAN_SAMPLES / 2];
}

/**
 * Parse argument từ SerialCommand an toàn.
 * Trả 0 nếu không có argument (tránh crash khi atoi(NULL)).
 */
int parseCommandArg() {
  char *arg = cmd.next();
  if (arg == NULL) return 0;
  return atoi(arg);
}

/**
 * Dừng motor mái trượt.
 */
void stopMotor() {
  digitalWrite(PIN_MOTOR_IN1, LOW);
  digitalWrite(PIN_MOTOR_IN2, LOW);
  roofState = ROOF_IDLE;
}

// ============================================================
// 7. SENSOR READING FUNCTIONS
// ============================================================

void readAllSensors() {
  // -- Độ ẩm đất --
  int rawDAD = readAnalogMedian(PIN_CB_DAD);
  valDoAmDat = map(rawDAD, 0, 1023, 0, 100);

  // -- Nhiệt độ & Độ ẩm không khí (DHT20) --
  float tempC = dht.getTemperature();
  float tempH = dht.getHumidity();
  if (!isnan(tempC) && !isnan(tempH)) {
    valNhietDo = tempC;
    valDoAmKK  = tempH;
  }

  // -- Ánh sáng --
  int rawAS = readAnalogMedian(PIN_CB_AS);
  valAnhSang = map(rawAS, 0, 1023, 0, 100);

  // -- Chất lượng không khí --
  valPPM = sensorKK.getPPM();

  // -- Mưa --
  valMua = digitalRead(PIN_CB_MUA);
}

/**
 * Gửi dữ liệu cảm biến qua Serial (UART → ESP8266).
 * Format: "KEY VALUE\n" — SerialCommand bên ESP8266 parse được.
 */
void sendSensorData() {
  Serial.print(F("DAD "));
  Serial.println((int)valDoAmDat);

  Serial.print(F("ND "));
  Serial.println(valNhietDo);

  Serial.print(F("doam "));
  Serial.println(valDoAmKK);

  Serial.print(F("KK "));
  Serial.println(valPPM);

  Serial.print(F("Anh sang: "));
  Serial.println((int)valAnhSang);
}

// ============================================================
// 8. AUTO CONTROL FUNCTIONS (with hysteresis)
// ============================================================

void autoControlTuoiCay() {
  if (!flagAutoMode || flagManualTuoiCay) return;

  // Chỉ tưới khi đất khô VÀ không mưa
  if (valDoAmDat <= threshDAD && valMua == 1) {
    digitalWrite(PIN_TUOICAY, HIGH);
  } else if (valDoAmDat >= threshDAD + HYSTERESIS_DAD || valMua == 0) {
    digitalWrite(PIN_TUOICAY, LOW);
  }
}

void autoControlPhunSuong() {
  if (!flagAutoMode || flagManualPhunSuong) return;

  if (valNhietDo > threshND) {
    digitalWrite(PIN_PHUNSUONG, HIGH);
  } else if (valNhietDo < threshND - HYSTERESIS_TEMP) {
    digitalWrite(PIN_PHUNSUONG, LOW);
  }
}

void autoControlFan() {
  if (!flagAutoMode || flagManualFan) return;

  if (valPPM >= threshKK) {
    digitalWrite(PIN_FAN, HIGH);
  } else if (valPPM < threshKK - HYSTERESIS_PPM) {
    digitalWrite(PIN_FAN, LOW);
  }
}

void autoControlLed() {
  if (!flagAutoMode || flagManualLed) return;

  if (valAnhSang < threshAS) {
    digitalWrite(PIN_LED, HIGH);
  } else if (valAnhSang > threshAS + HYSTERESIS_AS) {
    digitalWrite(PIN_LED, LOW);
  }
}

/**
 * Điều khiển mái trượt dạng state machine.
 * Kiểm tra công tắc hành trình để dừng motor khi đạt vị trí.
 */
void autoControlRoof() {
  int ct_dong = digitalRead(PIN_CT_DONG);
  int ct_mo   = digitalRead(PIN_CT_MO);

  if (flagAutoMode && !flagManualNocNha) {
    // Mưa VÀ đất đã ẩm → đóng mái
    if (valMua == 0 || valDoAmDat >= threshDAD) {
      if (roofState != ROOF_CLOSING) {
        roofState = ROOF_CLOSING;
      }
    }
    // Không mưa VÀ đất khô → mở mái
    else if (valMua == 1 && valDoAmDat < threshDAD) {
      if (roofState != ROOF_OPENING) {
        roofState = ROOF_OPENING;
      }
    }
  }

  // Thực thi trạng thái motor
  switch (roofState) {
    case ROOF_OPENING:
      if (ct_dong == 0) {  // Đã chạm công tắc đóng (mở hết)
        stopMotor();
      } else {
        digitalWrite(PIN_MOTOR_IN1, HIGH);
        digitalWrite(PIN_MOTOR_IN2, LOW);
      }
      break;

    case ROOF_CLOSING:
      if (ct_mo == 0) {  // Đã chạm công tắc mở (đóng hết)
        stopMotor();
      } else {
        digitalWrite(PIN_MOTOR_IN1, LOW);
        digitalWrite(PIN_MOTOR_IN2, HIGH);
      }
      break;

    case ROOF_IDLE:
    default:
      // Motor đã dừng, không làm gì
      break;
  }
}

// ============================================================
// 9. COMMAND HANDLERS (nhận lệnh từ ESP8266 qua UART)
// ============================================================

void cmdLed() {
  int val = parseCommandArg();
  digitalWrite(PIN_LED, val);
  flagManualLed = (val == 1);
}

void cmdFan() {
  int val = parseCommandArg();
  digitalWrite(PIN_FAN, val);
  flagManualFan = (val == 1);
}

void cmdPhunSuong() {
  int val = parseCommandArg();
  digitalWrite(PIN_PHUNSUONG, val);
  flagManualPhunSuong = (val == 1);
}

void cmdTuoiCay() {
  int val = parseCommandArg();
  digitalWrite(PIN_TUOICAY, val);
  flagManualTuoiCay = (val == 1);
}

void cmdAutoMode() {
  int val = parseCommandArg();
  flagAutoMode = (val == 1);
  if (!flagAutoMode) {
    // Reset ngưỡng về mặc định khi tắt auto
    threshDAD = 15.0;
    threshND  = 30.0;
  }
}

void cmdNocNha() {
  int val = parseCommandArg();
  flagManualNocNha = (val == 1);
  if (val == 1) {
    roofState = ROOF_OPENING;
    Serial.println(F("mo noc"));
  } else {
    roofState = ROOF_CLOSING;
    Serial.println(F("dong noc"));
  }
}

/**
 * Khóa cửa — non-blocking sử dụng millis().
 * Phase 1: Hiển thị Welcome, mở khóa 3s.
 * Phase 2: Hiển thị Denied 3s.
 */
void cmdLock() {
  int val = parseCommandArg();
  if (val == 1) {
    digitalWrite(PIN_LOCK, HIGH);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("   WELCOME TO   "));
    lcd.setCursor(3, 1);
    lcd.print(F("GREENHOUSE"));
    lockStartTime = millis();
    lockActive = true;
    lockPhase = 1;
  } else {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F(" Access Denied!"));
    lockStartTime = millis();
    lockActive = true;
    lockPhase = 2;
  }
}

// -- Cập nhật non-blocking cho lock --
void updateLock() {
  if (!lockActive) return;
  if (millis() - lockStartTime >= 3000) {
    if (lockPhase == 1) {
      digitalWrite(PIN_LOCK, LOW);
    }
    lcd.clear();
    lockActive = false;
    lockPhase = 0;
  }
}

// -- Command hiệu chỉnh ngưỡng --
void cmdSetDAD()  { threshDAD = parseCommandArg(); }
void cmdSetND()   { threshND  = parseCommandArg(); }
void cmdSetKK()   { threshKK  = parseCommandArg(); }
void cmdSetAS()   { threshAS  = parseCommandArg(); }

// ============================================================
// 10. SETUP & LOOP
// ============================================================

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // -- Cấu hình chân I/O --
  pinMode(PIN_CT_DONG, INPUT_PULLUP);
  pinMode(PIN_CT_MO,   INPUT_PULLUP);
  pinMode(PIN_MOTOR_IN1, OUTPUT);
  pinMode(PIN_MOTOR_IN2, OUTPUT);
  pinMode(PIN_CB_DAD, INPUT);
  pinMode(PIN_CB_KK,  INPUT);
  pinMode(PIN_CB_MUA, INPUT);
  pinMode(PIN_FAN,       OUTPUT);
  pinMode(PIN_LED,       OUTPUT);
  pinMode(PIN_TUOICAY,   OUTPUT);
  pinMode(PIN_PHUNSUONG, OUTPUT);
  pinMode(PIN_LOCK,      OUTPUT);

  // -- Khởi tạo thiết bị --
  lcd.init();
  lcd.backlight();
  dht.begin();

  // -- Đăng ký lệnh Serial --
  cmd.addCommand("led",      cmdLed);
  cmd.addCommand("fan",      cmdFan);
  cmd.addCommand("phunsuong", cmdPhunSuong);
  cmd.addCommand("tuoicay",  cmdTuoiCay);
  cmd.addCommand("automode", cmdAutoMode);
  cmd.addCommand("nocnha",   cmdNocNha);
  cmd.addCommand("lock",     cmdLock);
  cmd.addCommand("MDAD",     cmdSetDAD);
  cmd.addCommand("MND",      cmdSetND);
  cmd.addCommand("MKK",      cmdSetKK);
  cmd.addCommand("MAS",      cmdSetAS);
}

void loop() {
  // Nhận lệnh từ ESP8266
  cmd.readSerial();

  // Cập nhật trạng thái khóa (non-blocking)
  updateLock();

  // Hiển thị LCD khi không có lock message
  if (!lockActive) {
    lcd.setCursor(0, 0);
    lcd.print(F("<<Scan Your Card>>"));
  }

  // Đọc cảm biến theo interval (tránh đọc quá nhanh)
  unsigned long now = millis();
  if (now - lastSensorRead >= SENSOR_INTERVAL) {
    lastSensorRead = now;

    readAllSensors();
    sendSensorData();

    // Điều khiển tự động
    autoControlTuoiCay();
    autoControlPhunSuong();
    autoControlFan();
    autoControlLed();
  }

  // Mái trượt cần kiểm tra liên tục (công tắc hành trình)
  autoControlRoof();
}