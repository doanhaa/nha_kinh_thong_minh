/*************************************************************
  ESP8266 — Kết nối ERa IoT Platform & RFID
  DỰ ÁN NHÀ KÍNH THỦY SINH
  Giao tiếp UART với Arduino UNO R3

  Download latest ERa library here:
    https://github.com/eoh-jsc/era-lib/releases/latest
    https://www.arduino.cc/reference/en/libraries/era

    ERa website:                https://e-ra.io
    ERa blog:                   https://iotasia.org
    ERa forum:                  https://forum.eoh.io
    Follow us:                  https://www.fb.com/EoHPlatform
 *************************************************************/

// Enable debug console
// Set CORE_DEBUG_LEVEL = 3 first
// #define ERA_DEBUG

// ============================================================
// 1. CONFIGURATION
// ============================================================
#define DEFAULT_MQTT_HOST "mqtt1.eoh.io"
#define ERA_AUTH_TOKEN "226afa54-cab0-4b3a-9eea-2e2ac5b0a027"

const char ssid[] = "ESP8266";
const char pass[] = "kakaaka8lan";

// ============================================================
// 2. INCLUDES
// ============================================================
#include <SPI.h>
#include <MFRC522.h>
#include <SerialCommand.h>
#include <SoftwareSerial.h>
#include <ESP8266WiFi.h>
#include <ERa.hpp>
#include <ERa/ERaTimer.hpp>

// ============================================================
// 3. PIN DEFINITIONS
// ============================================================
/*
  RFID MFRC522 wiring:
  • 3.3V = 3.3V
  • RST  = D3
  • GND  = GND
  • MISO = D6
  • MOSI = D7
  • SCK  = D5
  • SDA  = D4
*/
#define PIN_RFID_SS   D4
#define PIN_RFID_RST  D3
#define PIN_LED_GREEN D1
#define PIN_LED_RED   D2
#define PIN_BUZZER    D8

// ============================================================
// 4. CONSTANTS
// ============================================================
static const unsigned long ACCESS_DELAY  = 2000;
static const unsigned long DENIED_DELAY  = 1000;
static const unsigned long WIFI_CHECK_INTERVAL = 30000;  // 30s

// UID thẻ RFID được phép truy cập
static const char AUTHORIZED_UID[] = "09 0C 8D 9D";

// ============================================================
// 5. OBJECTS & VARIABLES
// ============================================================
MFRC522 mfrc522(PIN_RFID_SS, PIN_RFID_RST);
SerialCommand cmd;
ERaTimer timer;

unsigned long lastWifiCheck = 0;

// ============================================================
// 6. HELPER FUNCTIONS
// ============================================================

/**
 * Gửi lệnh qua Serial (UART → Arduino UNO).
 * Format: "COMMAND VALUE\n"
 */
void sendCommand(const char* command, int value) {
  Serial.print(command);
  Serial.print(' ');
  Serial.println(value);
}

/**
 * Parse argument từ SerialCommand an toàn.
 */
int parseCommandArg() {
  char *arg = cmd.next();
  if (arg == NULL) return 0;
  return atoi(arg);
}

/**
 * Kiểm tra và reconnect WiFi nếu bị mất kết nối.
 */
void checkWifiConnection() {
  unsigned long now = millis();
  if (now - lastWifiCheck < WIFI_CHECK_INTERVAL) return;
  lastWifiCheck = now;

  if (WiFi.status() != WL_CONNECTED) {
    WiFi.disconnect();
    WiFi.begin(ssid, pass);
  }
}

// ============================================================
// 7. RFID FUNCTIONS
// ============================================================

/**
 * Xử lý quẹt thẻ RFID — tách riêng khỏi loop() để tránh
 * dùng return sớm gây skip các hàm khác.
 */
void handleRFID() {
  // Kiểm tra có thẻ mới không
  if (!mfrc522.PICC_IsNewCardPresent()) return;
  if (!mfrc522.PICC_ReadCardSerial()) return;

  // Đọc UID thẻ
  String content = "";
  for (byte i = 0; i < mfrc522.uid.size; i++) {
    content.concat(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " ");
    content.concat(String(mfrc522.uid.uidByte[i], HEX));
  }
  content.toUpperCase();

  // So sánh UID
  if (content.substring(1) == AUTHORIZED_UID) {
    // Thẻ hợp lệ → gửi lệnh mở khóa cho Arduino
    Serial.println(F("lock 1"));
    digitalWrite(PIN_LED_GREEN, HIGH);
    delay(ACCESS_DELAY);
    digitalWrite(PIN_LED_GREEN, LOW);
  } else {
    // Thẻ không hợp lệ
    Serial.println(F("lock 0"));
    digitalWrite(PIN_LED_RED, HIGH);
    tone(PIN_BUZZER, 1000);
    delay(DENIED_DELAY);
    digitalWrite(PIN_LED_RED, LOW);
    noTone(PIN_BUZZER);
  }

  // Dừng đọc thẻ hiện tại
  mfrc522.PICC_HaltA();
}

// ============================================================
// 8. ERA_WRITE HANDLERS (nhận lệnh từ App → gửi cho Arduino)
// ============================================================

// V0: Điều khiển LED
ERA_WRITE(V0) {
  int value = param.getInt();
  sendCommand("led", value);
  ERa.virtualWrite(V0, value);
}

// V1: Điều khiển quạt
ERA_WRITE(V1) {
  int value = param.getInt();
  sendCommand("fan", value);
  ERa.virtualWrite(V1, value);
}

// V2: Điều khiển tưới cây
ERA_WRITE(V2) {
  int value = param.getInt();
  sendCommand("tuoicay", value);
  ERa.virtualWrite(V2, value);
}

// V3: Điều khiển phun sương
ERA_WRITE(V3) {
  int value = param.getInt();
  sendCommand("phunsuong", value);
  ERa.virtualWrite(V3, value);
}

// V4: Bật/tắt chế độ tự động
ERA_WRITE(V4) {
  int value = param.getInt();
  sendCommand("automode", value);
  ERa.virtualWrite(V4, value);
}

// V5: Điều khiển nóc nhà
ERA_WRITE(V5) {
  int value = param.getInt();
  sendCommand("nocnha", value);
  ERa.virtualWrite(V5, value);
}

// V11: Hiệu chỉnh ngưỡng độ ẩm đất
ERA_WRITE(V11) {
  int value = param.getInt();
  sendCommand("MDAD", value);
  ERa.virtualWrite(V11, value);
}

// V12: Hiệu chỉnh ngưỡng nhiệt độ
ERA_WRITE(V12) {
  int value = param.getInt();
  sendCommand("MND", value);
  ERa.virtualWrite(V12, value);
}

// V13: Hiệu chỉnh ngưỡng khí độc
ERA_WRITE(V13) {
  int value = param.getInt();
  sendCommand("MKK", value);
  ERa.virtualWrite(V13, value);
}

// V14: Hiệu chỉnh ngưỡng ánh sáng
ERA_WRITE(V14) {
  int value = param.getInt();
  sendCommand("MAS", value);
  ERa.virtualWrite(V14, value);
}

// ============================================================
// 9. SERIAL COMMAND HANDLERS (nhận data từ Arduino)
// ============================================================

void onDoAmDat() {
  int value = parseCommandArg();
  ERa.virtualWrite(V8, value);
}

void onNhietDo() {
  int value = parseCommandArg();
  ERa.virtualWrite(V6, value);
}

void onDoAmKK() {
  int value = parseCommandArg();
  ERa.virtualWrite(V7, value);
}

void onKhongKhi() {
  int value = parseCommandArg();
  ERa.virtualWrite(V9, value);
}

// ============================================================
// 10. SETUP & LOOP
// ============================================================

void setup() {
  Serial.begin(115200);

  // -- RFID --
  SPI.begin();
  mfrc522.PCD_Init();

  // -- LED & Buzzer --
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_RED,   OUTPUT);
  pinMode(PIN_BUZZER,    OUTPUT);
  noTone(PIN_BUZZER);

  // -- ERa Platform --
  ERa.begin(ssid, pass);

  // Gửi giá trị mặc định cho App
  ERa.virtualWrite(V4,  1);    // Auto mode ON
  ERa.virtualWrite(V11, 15);   // Ngưỡng DAD
  ERa.virtualWrite(V12, 35);   // Ngưỡng ND
  ERa.virtualWrite(V13, 150);  // Ngưỡng KK
  ERa.virtualWrite(V14, 50);   // Ngưỡng AS

  // -- Đăng ký lệnh nhận từ Arduino --
  cmd.addCommand("DAD",  onDoAmDat);
  cmd.addCommand("ND",   onNhietDo);
  cmd.addCommand("KK",   onKhongKhi);
  cmd.addCommand("doam", onDoAmKK);
}

void loop() {
  ERa.run();
  cmd.readSerial();
  handleRFID();
  checkWifiConnection();
}