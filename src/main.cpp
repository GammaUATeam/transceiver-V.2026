#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <ArduinoJson.h>
#include "esp_sleep.h"
#include "esp_pm.h"
#include <time.h>
#include <sys/time.h>
#include "esp_system.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "driver/rtc_io.h"

// +++ ДОДАТКОВІ БІБЛІОТЕКИ ДЛЯ WIFI ТА НАЛАШТУВАНЬ +++
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <DNSServer.h>

// +++ КРИПТОГРАФІЯ +++
#include "mbedtls/aes.h"
#include "mbedtls/sha256.h"

// +++ БІБЛІОТЕКИ ДЛЯ ДИСПЛЕЮ +++
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <U8g2_for_Adafruit_GFX.h>

// ==========================================
// ПІНИ ДЛЯ HELTEC WIRELESS TRACKER V1.1 / V1.2
// ==========================================
#define VEXT_PIN 3
#define ADC_CTRL_PIN 2

#define RADIO_SCLK_PIN 9
#define RADIO_MISO_PIN 11
#define RADIO_MOSI_PIN 10
#define RADIO_CS_PIN 8
#define RADIO_DIO1_PIN 14
#define RADIO_RST_PIN 12
#define RADIO_BUSY_PIN 13

#define LORA_FREQ 433.0

#define GPS_RX 33
#define GPS_TX 34

#define BTN 0
#define BATTERY_ADC_PIN 1

#define BLE_SWITCH_PIN 46
#define SOS_SWITCH_PIN 47
#define DISPLAY_BTN_PIN 17

// ПІН ДЛЯ АПАРАТНОГО ВИМИКАЧА (ТУМБЛЕРА)
#define POWER_SWITCH_PIN 4

#define TFT_MOSI 42
#define TFT_SCLK 41
#define TFT_CS 38
#define TFT_DC 40
#define TFT_RST 39
#define TFT_BL 21

SPIClass tftSPI(HSPI);

class CustomTFT : public Adafruit_ST7735
{
public:
  CustomTFT(SPIClass *spi, int8_t cs, int8_t dc, int8_t rst) : Adafruit_ST7735(spi, cs, dc, rst) {}
  void setOffset(int8_t x, int8_t y)
  {
    _xstart = x;
    _ystart = y;
  }
};

CustomTFT tft(&tftSPI, TFT_CS, TFT_DC, TFT_RST);
GFXcanvas16 canvas(160, 80);
U8G2_FOR_ADAFRUIT_GFX u8g2ForCanvas;

const uint16_t COLOR_GREEN = 0x07E0;
const uint16_t COLOR_YELLOW = 0x07FF;
const uint16_t COLOR_RED = 0x001F;
const uint16_t COLOR_BLUE = 0xF800;

volatile unsigned long popupTimer = 0;
volatile bool showPopup = false;
char popupText[20] = "";
uint16_t popupColor = ST77XX_WHITE;

volatile bool displayActive = false;
volatile unsigned long displayTimer = 0;
volatile unsigned long lastDisplayBtnTime = 0;
volatile bool forceImmediateUpdate = false;
volatile uint8_t currentPage = 0;
#define TOTAL_PAGES 3

void triggerPopup(const char *text, uint16_t color)
{
  strncpy(popupText, text, sizeof(popupText) - 1);
  popupText[sizeof(popupText) - 1] = '\0';
  popupColor = color;
  popupTimer = millis();
  showPopup = true;

  displayActive = true;
  displayTimer = millis();
  forceImmediateUpdate = true;
}

String settings_password = "SuperStrongPassword123";
String settings_name = "Команда 1";

Preferences preferences;
WebServer server(80);
DNSServer dnsServer;
bool isConfigMode = false;
bool isSystemInitialized = false;

#define MESH_SIGNATURE 0xB5
uint8_t derivedKey[16];
bool isKeyInitialized = false;

struct __attribute__((packed)) LoRaBody
{
  char deviceId[8];
  char name[16];
  uint32_t uid;
  int32_t lat;
  int32_t lon;
  int16_t altitude;
  uint8_t voltage;
  uint8_t battery;
  uint8_t type;
  uint16_t crc;
};

struct FullPacket
{
  uint32_t timestamp;
  uint8_t flags;
  uint8_t ttl;
  LoRaBody body;
};

#define ENCRYPTED_SIZE sizeof(LoRaBody)
#define IV_SIZE 16
#define RADIO_PACKET_SIZE (1 + 1 + 4 + 1 + IV_SIZE + ENCRYPTED_SIZE)

const char *DEVICE_ID_CONST = "h1";
#define LOCAL_DISPLAY_ID DEVICE_ID_CONST
#define DEFAULT_TTL 3
#define baud_rate 115200

const int RX_WINDOW_SECONDS = 25;
const int TX_WINDOW_SECONDS = 20;
const int LONG_LISTEN_INTERVAL_CYCLES = 200;

#define FLAG_GPS_VALID 0x01
#define FLAG_RELAYED 0x02
#define TYPE_POS 0
#define TYPE_SOS 1

int loopCounter = 0;
bool isLongListenMode = false;
bool hasSentThisMinute = false;
bool isGpsAwake = true;

const float ADC_FILTER_ALPHA = 0.1;
const int ADC_SAMPLES = 20;
const float BATTERY_MIN_VOLTAGE = 3.20;
const float BATTERY_MAX_VOLTAGE = 4.20;
float filteredBatteryVoltage = 0.0;

HardwareSerial gpsSerial(1);
TinyGPSPlus gps;
SX1262 radio = new Module(RADIO_CS_PIN, RADIO_DIO1_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN);

TaskHandle_t DisplayTaskHandle;
esp_pm_lock_handle_t display_pm_lock;

volatile bool rxFlag = false;
volatile unsigned long isrLastTime = 0;
volatile unsigned long lastSosIsrTime = 0;
volatile bool wifiBtnFlag = false;
volatile bool sosChangedFlag = false;

#if defined(ESP8266) || defined(ESP32)
ICACHE_RAM_ATTR
#endif
void setFlag(void) { rxFlag = true; }

#if defined(ESP8266) || defined(ESP32)
ICACHE_RAM_ATTR
#endif
void wifiBtnISR(void)
{
  unsigned long currentTime = (unsigned long)(esp_timer_get_time() / 1000ULL);
  if (currentTime - isrLastTime > 400)
  {
    wifiBtnFlag = true;
    isrLastTime = currentTime;
  }
}

#if defined(ESP8266) || defined(ESP32)
ICACHE_RAM_ATTR
#endif
void displayBtnISR()
{
  unsigned long currentTime = (unsigned long)(esp_timer_get_time() / 1000ULL);
  if (currentTime - lastDisplayBtnTime > 300)
  {
    if (!displayActive)
    {
      displayActive = true;
      currentPage = 0;
    }
    else
    {
      currentPage = (currentPage + 1) % TOTAL_PAGES;
    }
    forceImmediateUpdate = true;
    displayTimer = millis();
    lastDisplayBtnTime = currentTime;
  }
}

#if defined(ESP8266) || defined(ESP32)
ICACHE_RAM_ATTR
#endif
void safeWakeupISR(void)
{
  unsigned long currentTime = (unsigned long)(esp_timer_get_time() / 1000ULL);
  if (currentTime - lastSosIsrTime > 250)
  {
    sosChangedFlag = true;
    lastSosIsrTime = currentTime;
  }
}

void smartDelay(int ms)
{
  unsigned long start = millis();
  while (millis() - start < (unsigned long)ms)
  {
    if (sosChangedFlag || wifiBtnFlag || rxFlag)
      break;
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

struct RelayItem
{
  FullPacket packet;
  unsigned long queuedTime;
  unsigned long delayMs;
  bool active;
  uint8_t retries;
};
#define QUEUE_SIZE 5
#define MAX_RELAY_RETRIES 3
RelayItem relayQueue[QUEUE_SIZE];
int queueHead = 0;
int queueTail = 0;

JsonDocument docDisplay;

unsigned long lastSendTime = 0;
unsigned long sendInterval = 0;

bool isLoRaAwake = true;
unsigned long lastGpsUpdateMs = 0;
const unsigned long GPS_TIMEOUT_MS = 3600000;

#define MESSAGE_HISTORY_SIZE 50
uint32_t messageHistory[MESSAGE_HISTORY_SIZE];
int historyIndex = 0;

BLECharacteristic *pCharacteristic;
BLEServer *pServer;
bool deviceConnected = false;
bool oldDeviceConnected = false;
bool bleEnabled = false;
unsigned long lastBleRetryTime = 0;

#define SERVICE_UUID "a066b5b0-c522-4aa9-b148-8f24f37fcba6"
#define CHARACTERISTIC_UUID "d792d09f-1d6e-422b-991a-e2933e7d848b"

void enterDeepSleep();
void sendGPSData();
bool sendSafeLoRa(FullPacket &packet);
void receiveLoRaMessage();
void relayBlind(uint8_t ttl, uint32_t timestamp, uint8_t flags, uint8_t *iv, uint8_t *cipherData);
void enqueueRelay(FullPacket &packet);
void processRelayQueue();
bool isPacketUnique(FullPacket &packet);
void addHashToHistory(uint32_t hash);
uint32_t calculatePacketHash(FullPacket &packet);
void syncTimeFromMesh(uint32_t remoteTimestamp, uint8_t remoteFlags);
void syncWithGPS();
float readBatteryVoltage();
int calculateBatteryLevel(float voltage);
bool isSOS();
void setupPowerSaving();
void sleepLoRa();
void wakeLoRa();
void sleepGPS();
void wakeGPS();
void processGPS(int maxBytes);
void updateBLE(FullPacket &packet, bool isLocal);
uint16_t calculateCRC(uint8_t *data, size_t len);
void generateKeyFromPassword(const char *password);
bool aesCtrEncryptDecrypt(const uint8_t *input, uint8_t *output, size_t length, uint8_t *iv);
void loadSettings();
void setupConfigMode();
void handleRoot();
void handleSave();

class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    deviceConnected = true;
    Serial.println("BLE connected");
  }
  void onDisconnect(BLEServer *pServer)
  {
    deviceConnected = false;
    Serial.println("BLE disconnected");
  }
};

void loadSettings()
{
  preferences.begin("lora_config", false);
  settings_name = preferences.getString("dev_name", "Команда 1");
  settings_password = preferences.getString("aes_pass", "SuperStrongPassword123");
  preferences.end();
}

void setupConfigMode()
{
  WiFi.mode(WIFI_AP);
  String apName = "GAMMA-" + String(DEVICE_ID_CONST);
  WiFi.softAP(apName.c_str(), "");
  dnsServer.start(53, "*", WiFi.softAPIP());
  server.on("/", handleRoot);
  server.on("/save", HTTP_POST, handleSave);
  server.onNotFound(handleRoot);
  server.begin();
  Serial.println("AP Started: " + apName);
}

void handleRoot()
{
  String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<title>LoRa Settings</title><style>body{font-family:sans-serif;padding:20px;background:#f0f0f0}input{display:block;width:100%;padding:10px;margin:10px 0;box-sizing:border-box}button{background:#007bff;color:white;padding:15px;border:none;width:100%;font-size:16px}</style></head><body>";
  html += "<h2>Налаштування</h2><form action='/save' method='POST'><label>Ім'я (до 15 символів):</label>";
  html += "<input type='text' name='devname' value='" + settings_name + "' maxlength='15'><label>Ключ шифрування:</label>";
  html += "<input type='text' name='pass' value='" + settings_password + "'><button type='submit'>ЗБЕРЕГТИ І ПЕРЕЗАВАНТАЖИТИ</button></form></body></html>";
  server.send(200, "text/html", html);
}

void handleSave()
{
  if (server.hasArg("devname") && server.hasArg("pass"))
  {
    String newName = server.arg("devname");
    String newPass = server.arg("pass");
    if (newName.length() > 0 && newPass.length() > 0)
    {
      preferences.begin("lora_config", false);
      preferences.putString("dev_name", newName);
      preferences.putString("aes_pass", newPass);
      preferences.end();

      String html = "<html><body><h1>Збережено!</h1><p>Перезавантаження...</p></body></html>";
      server.send(200, "text/html", html);
      delay(100);

      triggerPopup("REBOOT", COLOR_RED);
      delay(3000);

      showPopup = false;
      displayActive = false;
      delay(200);
      radio.sleep();

      ESP.restart();
    }
  }
  else
  {
    server.send(400, "text/plain", "Error: Empty fields");
  }
}

// ==========================================
// ФУНКЦІЯ ГЛИБОКОГО СНУ (ФІКС ФАНТОМНОГО ПЕРЕЗАВАНТАЖЕННЯ)
// ==========================================
void enterDeepSleep()
{
  Serial.println("Power switch is OFF. Going to Deep Sleep...");

  if (isSystemInitialized)
  {
    digitalWrite(TFT_BL, LOW);
    tft.fillScreen(ST77XX_BLACK);

    if (isLoRaAwake)
      radio.sleep();
    if (isGpsAwake)
      sleepGPS();

    tftSPI.end();
    SPI.end();
    gpsSerial.end();
  }

  digitalWrite(VEXT_PIN, LOW);
  digitalWrite(ADC_CTRL_PIN, LOW);
  digitalWrite(TFT_BL, LOW);

  gpio_hold_en((gpio_num_t)VEXT_PIN);
  gpio_hold_en((gpio_num_t)ADC_CTRL_PIN);
  gpio_hold_en((gpio_num_t)TFT_BL);

  gpio_reset_pin((gpio_num_t)TFT_MOSI);
  gpio_reset_pin((gpio_num_t)TFT_SCLK);
  gpio_reset_pin((gpio_num_t)TFT_CS);
  gpio_reset_pin((gpio_num_t)TFT_DC);
  gpio_reset_pin((gpio_num_t)TFT_RST);

  gpio_reset_pin((gpio_num_t)GPS_RX);
  gpio_reset_pin((gpio_num_t)GPS_TX);

  gpio_reset_pin((gpio_num_t)35);

  gpio_reset_pin((gpio_num_t)RADIO_MOSI_PIN);
  gpio_reset_pin((gpio_num_t)RADIO_SCLK_PIN);
  gpio_reset_pin((gpio_num_t)RADIO_MISO_PIN);

  pinMode(RADIO_CS_PIN, OUTPUT);
  digitalWrite(RADIO_CS_PIN, HIGH);
  gpio_hold_en((gpio_num_t)RADIO_CS_PIN);

  pinMode(RADIO_RST_PIN, OUTPUT);
  digitalWrite(RADIO_RST_PIN, HIGH);
  gpio_hold_en((gpio_num_t)RADIO_RST_PIN);

  gpio_deep_sleep_hold_en();

  // +++ ФІКС BOOT-LOOP: Зберігаємо живлення RTC-домену +++
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);

  // Вмикаємо жорстку підтяжку (pull-up) і вимикаємо притягування до землі (pull-down)
  rtc_gpio_pullup_en((gpio_num_t)POWER_SWITCH_PIN);
  rtc_gpio_pulldown_dis((gpio_num_t)POWER_SWITCH_PIN);

  // Дочекатися, поки Serial відправить всі байти, щоб не розбудити чіп перериванням UART
  Serial.flush();

  esp_sleep_enable_ext0_wakeup((gpio_num_t)POWER_SWITCH_PIN, 0);
  esp_deep_sleep_start();
}

// ==========================================
// ЗАДАЧА ДИСПЛЕЮ НА ЯДРІ 0 (Core 0)
// ==========================================
void displayTask(void *pvParameters)
{
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, LOW);

  tftSPI.begin(TFT_SCLK, -1, TFT_MOSI, TFT_CS);
  tft.initR(INITR_MINI160x80);
  tft.setRotation(1);
  tft.invertDisplay(true);

  for (int x = 0; x <= 2; x++)
  {
    for (int y = 23; y <= 28; y++)
    {
      tft.setOffset(x, y);
      tft.fillScreen(ST77XX_BLACK);
    }
  }

  tft.setOffset(0, 26);

  u8g2ForCanvas.begin(canvas);
  u8g2ForCanvas.setFontMode(1);
  u8g2ForCanvas.setFontDirection(0);

  esp_pm_lock_acquire(display_pm_lock);
  bool wasActive = true;

  canvas.fillScreen(ST77XX_BLACK);
  canvas.setTextWrap(false);
  canvas.setTextColor(COLOR_GREEN);
  canvas.setTextSize(2);

  char splashText[20];
  sprintf(splashText, "GAMMA-%s", DEVICE_ID_CONST);

  int pxWidth = strlen(splashText) * 12;
  int x = (160 - pxWidth) / 2;
  int y = (80 - 16) / 2;

  canvas.setCursor(x > 0 ? x : 0, y);
  canvas.print(splashText);

  tft.drawRGBBitmap(0, 0, canvas.getBuffer(), 160, 80);
  digitalWrite(TFT_BL, HIGH);

  vTaskDelay(1000 / portTICK_PERIOD_MS);

  displayActive = true;
  displayTimer = millis();
  forceImmediateUpdate = true;

  for (;;)
  {
    if (displayActive || showPopup)
    {
      if (!wasActive)
      {
        esp_pm_lock_acquire(display_pm_lock);
        wasActive = true;
        readBatteryVoltage();
      }

      unsigned long currentMillis = millis();

      if (showPopup)
      {
        if (currentMillis - popupTimer <= 3000)
        {
          static unsigned long lastPopupUpdate = 0;
          if (forceImmediateUpdate || currentMillis - lastPopupUpdate > 200)
          {
            forceImmediateUpdate = false;
            lastPopupUpdate = currentMillis;

            canvas.fillScreen(ST77XX_BLACK);
            canvas.setTextWrap(false);

            canvas.setTextSize(3);
            canvas.setTextColor(popupColor);

            int len = strlen(popupText);
            int pxWidth = len * 18;
            int x = (160 - pxWidth) / 2;
            int y = (80 - 24) / 2;

            canvas.setCursor(x > 0 ? x : 0, y);
            canvas.print(popupText);

            tft.drawRGBBitmap(0, 0, canvas.getBuffer(), 160, 80);
            if (digitalRead(TFT_BL) == LOW)
              digitalWrite(TFT_BL, HIGH);
          }
        }
        else
        {
          showPopup = false;
          forceImmediateUpdate = true;
          displayTimer = currentMillis;
        }
      }
      else if (currentMillis - displayTimer <= 5000)
      {
        static unsigned long lastUpdate = 0;

        if (forceImmediateUpdate || currentMillis - lastUpdate > 500)
        {
          if (forceImmediateUpdate)
          {
            readBatteryVoltage();
          }

          forceImmediateUpdate = false;
          lastUpdate = currentMillis;

          canvas.fillScreen(ST77XX_BLACK);
          canvas.setTextWrap(false);

          canvas.setTextSize(1);
          canvas.setCursor(2, 2);

          if (isSOS())
          {
            canvas.setTextColor(COLOR_RED);
            canvas.print("! SOS !");
          }
          else
          {
            canvas.setTextColor(ST77XX_WHITE);
            canvas.printf("%s", DEVICE_ID_CONST);

            canvas.setCursor(30, 2);
            if (isConfigMode)
            {
              canvas.setTextColor(COLOR_YELLOW);
              canvas.print("WIFI");
            }
            else if (bleEnabled)
            {
              if (deviceConnected)
              {
                canvas.setTextColor(COLOR_GREEN);
              }
              else
              {
                canvas.setTextColor(COLOR_BLUE);
              }
              canvas.print("BLE");
            }
          }

          char timeStr[10] = "--:--";
          struct tm timeinfo;
          if (getLocalTime(&timeinfo, 0))
          {
            sprintf(timeStr, "%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min);
          }
          canvas.setTextColor(ST77XX_WHITE);
          canvas.setCursor(65, 2);
          canvas.print(timeStr);

          int batPct = calculateBatteryLevel(filteredBatteryVoltage);
          uint16_t batColor = COLOR_GREEN;
          if (batPct <= 20)
            batColor = COLOR_RED;
          else if (batPct <= 50)
            batColor = COLOR_YELLOW;

          canvas.setTextColor(batColor);
          if (batPct == 100)
            canvas.setCursor(110, 2);
          else if (batPct >= 10)
            canvas.setCursor(116, 2);
          else
            canvas.setCursor(122, 2);
          canvas.printf("%d%%", batPct);

          canvas.drawRect(138, 1, 18, 9, batColor);
          canvas.fillRect(156, 3, 2, 5, batColor);
          int batW = map(batPct, 0, 100, 0, 14);
          canvas.fillRect(140, 3, batW, 5, batColor);

          canvas.drawFastHLine(0, 12, 160, ST77XX_WHITE);

          int y = 18;

          if (currentPage == 0)
          {
            canvas.setTextSize(1);
            canvas.setCursor(2, y);

            if (!isGpsAwake)
            {
              canvas.setTextColor(COLOR_BLUE);
              canvas.print("GPS: Sleeping Zzz");
            }
            else if (gps.satellites.isValid() && gps.satellites.value() > 0)
            {
              canvas.setTextColor(COLOR_GREEN);
              canvas.printf("GPS Lock: %d Sats", gps.satellites.value());
            }
            else
            {
              canvas.setTextColor(COLOR_YELLOW);
              canvas.print("GPS: Searching...");
            }
            canvas.setTextColor(ST77XX_WHITE);

            y += 14;
            canvas.setTextSize(2);
            canvas.setCursor(2, y);
            canvas.printf("%-9.5f", gps.location.isValid() ? gps.location.lat() : 0.0);

            y += 20;
            canvas.setCursor(2, y);
            canvas.printf("%-9.5f", gps.location.isValid() ? gps.location.lng() : 0.0);
          }
          else if (currentPage == 1)
          {
            u8g2ForCanvas.setFont(u8g2_font_6x13_t_cyrillic);
            u8g2ForCanvas.setForegroundColor(ST77XX_WHITE);
            u8g2ForCanvas.setCursor(2, y + 11);
            u8g2ForCanvas.print("Node: ");
            u8g2ForCanvas.print(settings_name);

            y += 14;
            canvas.setTextSize(1);
            canvas.setTextColor(ST77XX_WHITE);
            canvas.setCursor(2, y);
            canvas.printf("Device: GAMMA-%s", DEVICE_ID_CONST);

            y += 14;
            canvas.setCursor(2, y);
            canvas.printf("Alt:  %.0f m", gps.altitude.isValid() ? gps.altitude.meters() : 0.0);

            y += 14;
            canvas.setCursor(2, y);
            canvas.printf("Volt: %.2f V", filteredBatteryVoltage);
          }
          else if (currentPage == 2)
          {
            canvas.setTextSize(1);
            canvas.setTextColor(ST77XX_WHITE);
            canvas.setCursor(2, y);
            canvas.print("Encryption Key:");

            y += 12;
            canvas.setCursor(2, y);
            canvas.printf("%s", settings_password.c_str());

            y += 16;
            canvas.setCursor(2, y);

            if (isConfigMode)
            {
              canvas.setTextColor(COLOR_YELLOW);
              canvas.print("IP: ");
              canvas.print(WiFi.softAPIP().toString().c_str());
              canvas.setTextColor(ST77XX_WHITE);
            }
            else
            {
              int relayCount = 0;
              for (int i = 0; i < QUEUE_SIZE; i++)
              {
                if (relayQueue[i].active)
                  relayCount++;
              }
              canvas.printf("Queue: %d msg", relayCount);
            }

            y += 14;
            canvas.setCursor(2, y);

            if (isSOS())
            {
              canvas.setTextColor(COLOR_RED);
              canvas.print("   !!! SOS ACTIVE !!!   ");
            }
            else
            {
              canvas.setTextColor(COLOR_GREEN);
              canvas.print("   Status: Normal       ");
            }
          }

          int dotX = 72;
          for (int i = 0; i < TOTAL_PAGES; i++)
          {
            if (i == currentPage)
              canvas.fillCircle(dotX + (i * 8), 76, 2, ST77XX_WHITE);
            else
              canvas.drawCircle(dotX + (i * 8), 76, 2, ST77XX_WHITE);
          }

          tft.drawRGBBitmap(0, 0, canvas.getBuffer(), 160, 80);

          if (digitalRead(TFT_BL) == LOW)
          {
            digitalWrite(TFT_BL, HIGH);
          }
        }
      }
      else
      {
        displayActive = false;
      }
    }
    else
    {
      if (wasActive)
      {
        digitalWrite(TFT_BL, LOW);
        tft.fillScreen(ST77XX_BLACK);
        esp_pm_lock_release(display_pm_lock);
        wasActive = false;
      }
    }

    if (displayActive || showPopup)
    {
      vTaskDelay(50 / portTICK_PERIOD_MS);
    }
    else
    {
      vTaskDelay(250 / portTICK_PERIOD_MS);
    }
  }
}

// ==========================================
// SETUP
// ==========================================
void setup()
{
  Serial.begin(115200);

  gpio_hold_dis((gpio_num_t)VEXT_PIN);
  gpio_hold_dis((gpio_num_t)ADC_CTRL_PIN);
  gpio_hold_dis((gpio_num_t)TFT_BL);
  gpio_hold_dis((gpio_num_t)RADIO_CS_PIN);
  gpio_hold_dis((gpio_num_t)RADIO_RST_PIN);
  gpio_deep_sleep_hold_dis();

  rtc_gpio_deinit((gpio_num_t)POWER_SWITCH_PIN);

  setenv("TZ", "EET-2EEST,M3.5.0/3,M10.5.0/4", 1);
  tzset();

  pinMode(POWER_SWITCH_PIN, INPUT_PULLUP);
  delay(10);

  if (digitalRead(POWER_SWITCH_PIN) == HIGH)
  {
    enterDeepSleep();
  }

  pinMode(VEXT_PIN, OUTPUT);
  digitalWrite(VEXT_PIN, HIGH);

  pinMode(ADC_CTRL_PIN, OUTPUT);
  digitalWrite(ADC_CTRL_PIN, LOW);

  pinMode(BTN, INPUT_PULLUP);
  pinMode(BLE_SWITCH_PIN, INPUT_PULLUP);
  pinMode(SOS_SWITCH_PIN, INPUT_PULLUP);
  pinMode(DISPLAY_BTN_PIN, INPUT_PULLUP);

  loadSettings();

  attachInterrupt(digitalPinToInterrupt(BTN), wifiBtnISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(DISPLAY_BTN_PIN), displayBtnISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(SOS_SWITCH_PIN), safeWakeupISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BLE_SWITCH_PIN), safeWakeupISR, CHANGE);

  generateKeyFromPassword(settings_password.c_str());
  randomSeed(esp_random());

  setupPowerSaving();
  esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "disp_lock", &display_pm_lock);

  analogReadResolution(12);
  readBatteryVoltage();

  xTaskCreatePinnedToCore(displayTask, "DisplayTask", 4096, NULL, 1, &DisplayTaskHandle, 0);

  gpsSerial.setRxBufferSize(2048);
  gpsSerial.begin(baud_rate, SERIAL_8N1, GPS_RX, GPS_TX);

  SPI.begin(RADIO_SCLK_PIN, RADIO_MISO_PIN, RADIO_MOSI_PIN, RADIO_CS_PIN);
  int state = radio.begin(LORA_FREQ);
  if (state != RADIOLIB_ERR_NONE)
  {
    Serial.println("LoRa failed!");
    while (1)
      delay(100);
  }

  radio.setOutputPower(22);
  radio.setSpreadingFactor(7);
  radio.setBandwidth(125.0);
  radio.setCodingRate(5);
  radio.setCRC(true);
  radio.setDio1Action(setFlag);

  BLEDevice::init(("GAMMA-" + String(DEVICE_ID_CONST)).c_str());
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();

  for (int i = 0; i < QUEUE_SIZE; i++)
    relayQueue[i].active = false;

  wakeLoRa();

  lastSendTime = millis();
  sendInterval = random(2000, 10000);

  isSystemInitialized = true;
}

// ==========================================
// ФУНКЦІЇ СНУ GPS (Жорсткий режим)
// ==========================================
void sleepGPS()
{
  if (!isGpsAwake)
    return;

  gpsSerial.print("$PCAS03,0,0,0,0,0,0,0,0,0,0,,,0,0*02\r\n");
  delay(20);
  gpsSerial.print("$PCAS04,1*18\r\n");
  delay(20);
  gpsSerial.print("$PCAS04,5*1C\r\n");

  gpsSerial.flush();
  isGpsAwake = false;
}

void wakeGPS()
{
  if (isGpsAwake)
    return;

  gpsSerial.print("\r\n\r\n");
  delay(50);
  gpsSerial.print("$PCAS03,1,0,0,1,1,0,0,0,0,0,,,0,0*02\r\n");
  isGpsAwake = true;
}

// ==========================================
// CRYPTO
// ==========================================
void generateKeyFromPassword(const char *password)
{
  mbedtls_sha256_context ctx;
  mbedtls_sha256_init(&ctx);
  mbedtls_sha256_starts(&ctx, 0);
  mbedtls_sha256_update(&ctx, (const unsigned char *)password, strlen(password));
  unsigned char hash[32];
  mbedtls_sha256_finish(&ctx, hash);
  mbedtls_sha256_free(&ctx);
  memcpy(derivedKey, hash, 16);
  isKeyInitialized = true;
}

bool aesCtrEncryptDecrypt(const uint8_t *input, uint8_t *output, size_t length, uint8_t *iv)
{
  if (!isKeyInitialized)
    return false;
  mbedtls_aes_context aes;
  mbedtls_aes_init(&aes);
  mbedtls_aes_setkey_enc(&aes, derivedKey, 128);
  size_t nc_off = 0;
  uint8_t stream_block[16];
  uint8_t nonce_counter[16];
  memcpy(nonce_counter, iv, 16);
  int ret = mbedtls_aes_crypt_ctr(&aes, length, &nc_off, nonce_counter, stream_block, input, output);
  mbedtls_aes_free(&aes);
  return (ret == 0);
}

// ==========================================
// LOOP
// ==========================================
void loop()
{
  if (digitalRead(POWER_SWITCH_PIN) == HIGH)
  {
    delay(50);
    if (digitalRead(POWER_SWITCH_PIN) == HIGH)
    {
      enterDeepSleep();
    }
  }

  if (sosChangedFlag)
    sosChangedFlag = false;

  if (wifiBtnFlag)
  {
    wifiBtnFlag = false;
    if (!isConfigMode)
    {
      Serial.println("Кнопка натиснута. Перехід у WiFi AP...");
      triggerPopup("WIFI AP", COLOR_YELLOW);

      esp_pm_config_esp32s3_t pm_config = {.max_freq_mhz = 80, .min_freq_mhz = 80, .light_sleep_enable = false};
      esp_pm_configure(&pm_config);
      if (bleEnabled)
      {
        pServer->getAdvertising()->stop();
        BLEDevice::deinit(false);
        bleEnabled = false;
      }
      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);
      delay(100);
      wakeGPS();
      isConfigMode = true;
      setupConfigMode();
    }
    else
    {
      Serial.println("Вихід з режиму налаштувань. Перезавантаження...");
      triggerPopup("REBOOT", COLOR_RED);
      delay(3000);

      showPopup = false;
      displayActive = false;
      delay(200);
      radio.sleep();
      ESP.restart();
    }
  }

  if (isConfigMode)
  {
    dnsServer.processNextRequest();
    server.handleClient();
    delay(5);
    return;
  }

  static bool lastSosState = digitalRead(SOS_SWITCH_PIN);
  bool currentSosState = digitalRead(SOS_SWITCH_PIN);

  if (currentSosState == LOW && lastSosState == HIGH)
  {
    Serial.println("!!! SOS АКТИВОВАНО - МИТТЄВА ВІДПРАВКА !!!");
    triggerPopup("! SOS !", COLOR_RED);

    hasSentThisMinute = false;
    lastSendTime = millis();
    sendInterval = 0;
    wakeGPS();
  }
  else if (currentSosState == HIGH && lastSosState == LOW)
  {
    if (showPopup && strncmp(popupText, "! SOS !", 7) == 0)
    {
      showPopup = false;
      forceImmediateUpdate = true;
      displayTimer = millis();
    }
  }
  lastSosState = currentSosState;

  bool switchState = digitalRead(BLE_SWITCH_PIN);
  if (switchState == LOW && bleEnabled)
  {
    triggerPopup("BLE OFF", COLOR_YELLOW);
    pServer->getAdvertising()->stop();
    bleEnabled = false;
    deviceConnected = false;
  }
  else if (switchState == HIGH && !bleEnabled)
  {
    triggerPopup("BLE ON", COLOR_BLUE);
    pServer->getAdvertising()->start();
    bleEnabled = true;
  }

  if (!deviceConnected && oldDeviceConnected)
  {
    if (millis() - lastBleRetryTime > 500)
    {
      if (bleEnabled)
        pServer->startAdvertising();
      lastBleRetryTime = millis();
      oldDeviceConnected = deviceConnected;
    }
  }
  if (deviceConnected && !oldDeviceConnected)
  {
    oldDeviceConnected = deviceConnected;
    lastBleRetryTime = 0;
  }

  processGPS(64);

  bool activeWindow = true;
  bool txAllowed = true;
  struct tm timeinfo;
  static int lastMinute = -1;

  if (getLocalTime(&timeinfo, 0))
  {
    if (timeinfo.tm_min != lastMinute)
    {
      lastMinute = timeinfo.tm_min;
      loopCounter++;
      isLongListenMode = (loopCounter % LONG_LISTEN_INTERVAL_CYCLES == 0);
      hasSentThisMinute = false;

      lastSendTime = millis();
      sendInterval = random(1000, 15000);
    }
    if (!isLongListenMode)
    {
      activeWindow = (timeinfo.tm_sec < RX_WINDOW_SECONDS);
      txAllowed = (timeinfo.tm_sec < TX_WINDOW_SECONDS);
    }
    else
    {
      activeWindow = true;
      txAllowed = (timeinfo.tm_sec < TX_WINDOW_SECONDS);
    }
  }

  if (activeWindow)
  {
    if (!isLoRaAwake)
      wakeLoRa();
    if (!isGpsAwake)
      wakeGPS();

    if (rxFlag)
    {
      rxFlag = false;
      receiveLoRaMessage();

      unsigned long safeOffset = (sendInterval > 1000) ? (sendInterval - 1000) : 0;
      if (millis() - lastSendTime >= safeOffset)
      {
        sendInterval += random(500, 1500);
      }
    }
    else
    {
      if (txAllowed)
      {
        processRelayQueue();

        if (millis() - lastSendTime >= sendInterval && !hasSentThisMinute)
        {
          sendGPSData();
          if (!isSOS())
          {
            hasSentThisMinute = true;
          }
          else
          {
            lastSendTime = millis();
            sendInterval = 10000;
          }
        }
      }
    }
    smartDelay(10);
  }
  else
  {
    if (isLoRaAwake)
      sleepLoRa();
    if (isGpsAwake)
      sleepGPS();

    processGPS(1024);
    smartDelay(100);
  }
}

// ==========================================
// RX / TX / HELPERS
// ==========================================
void setupPowerSaving()
{
  esp_pm_config_esp32s3_t pm_config = {
      .max_freq_mhz = 80,
      .min_freq_mhz = 10,
      .light_sleep_enable = true};
  esp_pm_configure(&pm_config);
}

bool sendSafeLoRa(FullPacket &packet)
{
  packet.body.crc = calculateCRC((uint8_t *)&packet.body, ENCRYPTED_SIZE - sizeof(uint16_t));
  if (!isLoRaAwake)
    wakeLoRa();
  int cad = radio.scanChannel();
  if (cad == RADIOLIB_LORA_DETECTED || cad == RADIOLIB_PREAMBLE_DETECTED)
  {
    radio.startReceive();
    return false;
  }
  uint8_t iv[IV_SIZE];
  uint8_t cipherBuffer[ENCRYPTED_SIZE];
  for (int i = 0; i < IV_SIZE; i++)
    iv[i] = (uint8_t)esp_random();
  if (!aesCtrEncryptDecrypt((uint8_t *)&packet.body, cipherBuffer, ENCRYPTED_SIZE, iv))
    return false;

  uint8_t txBuffer[256];
  int idx = 0;
  txBuffer[idx++] = MESH_SIGNATURE;
  txBuffer[idx++] = packet.ttl;
  memcpy(&txBuffer[idx], &packet.timestamp, 4);
  idx += 4;
  txBuffer[idx++] = packet.flags;
  memcpy(&txBuffer[idx], iv, IV_SIZE);
  idx += IV_SIZE;
  memcpy(&txBuffer[idx], cipherBuffer, ENCRYPTED_SIZE);
  idx += ENCRYPTED_SIZE;

  int state = radio.transmit(txBuffer, idx);
  radio.startReceive();
  return (state == RADIOLIB_ERR_NONE);
}

void relayBlind(uint8_t ttl, uint32_t timestamp, uint8_t flags, uint8_t *iv, uint8_t *cipherData)
{
  smartDelay(random(50, 150));
  int cad = radio.scanChannel();
  if (cad == RADIOLIB_LORA_DETECTED || cad == RADIOLIB_PREAMBLE_DETECTED)
  {
    radio.startReceive();
    return;
  }
  uint8_t txBuffer[256];
  int idx = 0;
  txBuffer[idx++] = MESH_SIGNATURE;
  txBuffer[idx++] = ttl - 1;
  memcpy(&txBuffer[idx], &timestamp, 4);
  idx += 4;
  txBuffer[idx++] = flags;
  memcpy(&txBuffer[idx], iv, IV_SIZE);
  idx += IV_SIZE;
  memcpy(&txBuffer[idx], cipherData, ENCRYPTED_SIZE);
  idx += ENCRYPTED_SIZE;
  radio.transmit(txBuffer, idx);
  radio.startReceive();
}

void receiveLoRaMessage()
{
  if (!isLoRaAwake)
    return;
  size_t len = radio.getPacketLength();
  if (len < RADIO_PACKET_SIZE)
    return;
  uint8_t rxBuffer[256];
  int state = radio.readData(rxBuffer, len);
  if (state != RADIOLIB_ERR_NONE)
    return;

  int idx = 0;
  uint8_t sig = rxBuffer[idx++];
  if (sig != MESH_SIGNATURE)
    return;

  uint8_t incomingTTL = rxBuffer[idx++];
  uint32_t incomingTimestamp;
  memcpy(&incomingTimestamp, &rxBuffer[idx], 4);
  idx += 4;
  uint8_t incomingFlags = rxBuffer[idx++];
  syncTimeFromMesh(incomingTimestamp, incomingFlags);

  uint8_t iv[IV_SIZE];
  uint8_t cipherBuffer[ENCRYPTED_SIZE];
  uint8_t plainBuffer[ENCRYPTED_SIZE];
  memcpy(iv, &rxBuffer[idx], IV_SIZE);
  idx += IV_SIZE;
  memcpy(cipherBuffer, &rxBuffer[idx], ENCRYPTED_SIZE);
  idx += ENCRYPTED_SIZE;

  aesCtrEncryptDecrypt(cipherBuffer, plainBuffer, ENCRYPTED_SIZE, iv);

  LoRaBody tempBody;
  memcpy(&tempBody, plainBuffer, ENCRYPTED_SIZE);
  uint16_t receivedCRC = tempBody.crc;
  uint16_t calculatedCRC = calculateCRC((uint8_t *)&tempBody, ENCRYPTED_SIZE - sizeof(uint16_t));

  if (receivedCRC == calculatedCRC)
  {
    FullPacket fullRx;
    fullRx.timestamp = incomingTimestamp;
    fullRx.flags = incomingFlags;
    fullRx.ttl = incomingTTL;
    memcpy(&fullRx.body, &tempBody, ENCRYPTED_SIZE);

    if (strncmp(fullRx.body.deviceId, DEVICE_ID_CONST, 8) == 0)
      return;

    if (isPacketUnique(fullRx))
    {
      updateBLE(fullRx, false);
      if (incomingTTL > 0)
      {
        fullRx.ttl = incomingTTL - 1;
        fullRx.flags |= FLAG_RELAYED;
        enqueueRelay(fullRx);
      }
    }
  }
  else if (incomingTTL > 0)
  {
    relayBlind(incomingTTL, incomingTimestamp, incomingFlags, iv, cipherBuffer);
  }
}

void sendGPSData()
{
  bool emergency = isSOS();
  if (!gps.location.isValid() && !emergency)
    return;

  FullPacket myPacket;
  float vbat = readBatteryVoltage();
  myPacket.ttl = DEFAULT_TTL;
  myPacket.flags = 0;

  time_t now;
  time(&now);
  myPacket.timestamp = (uint32_t)now;

  memset(myPacket.body.deviceId, 0, 8);
  strncpy(myPacket.body.deviceId, DEVICE_ID_CONST, 8);
  memset(myPacket.body.name, 0, 16);
  strncpy(myPacket.body.name, settings_name.c_str(), 16);

  myPacket.body.uid = esp_random();
  myPacket.body.type = emergency ? TYPE_SOS : TYPE_POS;
  myPacket.body.battery = calculateBatteryLevel(vbat);
  myPacket.body.voltage = (uint8_t)(vbat * 10);

  if (gps.location.isValid())
  {
    myPacket.body.lat = (int32_t)(gps.location.lat() * 10000000);
    myPacket.body.lon = (int32_t)(gps.location.lng() * 10000000);
    myPacket.body.altitude = (int16_t)gps.altitude.meters();
    if (gps.time.isValid() && gps.time.age() < 2000)
      myPacket.flags |= FLAG_GPS_VALID;
  }
  else
  {
    myPacket.body.lat = 0;
    myPacket.body.lon = 0;
    myPacket.body.altitude = 0;
  }

  if (sendSafeLoRa(myPacket))
  {
    addHashToHistory(calculatePacketHash(myPacket));
    updateBLE(myPacket, true);
  }
}

uint16_t calculateCRC(uint8_t *data, size_t len)
{
  uint16_t crc = 0;
  for (size_t i = 0; i < len; i++)
  {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t j = 0; j < 8; j++)
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
  }
  return crc;
}

void processGPS(int maxBytes)
{
  int bytesRead = 0;
  while (gpsSerial.available() > 0 && bytesRead < maxBytes)
  {
    gps.encode(gpsSerial.read());
    bytesRead++;
  }
  if (gps.time.isUpdated() && gps.time.isValid() && gps.date.isValid())
    syncWithGPS();
}

void syncWithGPS()
{
  struct tm tm;
  tm.tm_year = gps.date.year() - 1900;
  tm.tm_mon = gps.date.month() - 1;
  tm.tm_mday = gps.date.day();
  tm.tm_hour = gps.time.hour();
  tm.tm_min = gps.time.minute();
  tm.tm_sec = gps.time.second();
  tm.tm_isdst = 0;

  setenv("TZ", "UTC", 1);
  tzset();
  time_t t = mktime(&tm);

  setenv("TZ", "EET-2EEST,M3.5.0/3,M10.5.0/4", 1);
  tzset();

  struct timeval now = {.tv_sec = t, .tv_usec = 0};
  time_t current;
  time(&current);
  if (abs((long)current - (long)t) > 1 || millis() - lastGpsUpdateMs > 10000)
  {
    settimeofday(&now, NULL);
    lastGpsUpdateMs = millis();
  }
}

void syncTimeFromMesh(uint32_t remoteTimestamp, uint8_t remoteFlags)
{
  if (!(remoteFlags & FLAG_GPS_VALID))
    return;
  if (millis() - lastGpsUpdateMs > GPS_TIMEOUT_MS)
  {
    time_t now;
    time(&now);
    if (remoteTimestamp > 1000000000 && abs((long)now - (long)remoteTimestamp) > 2)
    {
      struct timeval tv = {.tv_sec = (time_t)remoteTimestamp, .tv_usec = 0};
      settimeofday(&tv, NULL);
    }
  }
}

void sleepLoRa()
{
  radio.sleep();
  isLoRaAwake = false;
}

void wakeLoRa()
{
  radio.startReceive();
  isLoRaAwake = true;
}

void enqueueRelay(FullPacket &packet)
{
  int nextHead = (queueHead + 1) % QUEUE_SIZE;
  if (nextHead == queueTail)
    queueTail = (queueTail + 1) % QUEUE_SIZE;
  relayQueue[queueHead].packet = packet;

  relayQueue[queueHead].queuedTime = millis();
  relayQueue[queueHead].delayMs = random(100, 1000);

  relayQueue[queueHead].active = true;
  relayQueue[queueHead].retries = 0;
  queueHead = nextHead;
}

void processRelayQueue()
{
  if (queueHead == queueTail)
    return;
  RelayItem *item = &relayQueue[queueTail];

  if (item->active && (millis() - item->queuedTime >= item->delayMs))
  {
    if (sendSafeLoRa(item->packet))
    {
      item->active = false;
      queueTail = (queueTail + 1) % QUEUE_SIZE;
    }
    else
    {
      item->retries++;
      if (item->retries > MAX_RELAY_RETRIES)
      {
        item->active = false;
        queueTail = (queueTail + 1) % QUEUE_SIZE;
      }
      else
      {
        item->queuedTime = millis();
        item->delayMs = random(1000, 3000);
      }
    }
  }
}

void updateBLE(FullPacket &packet, bool isLocal)
{
  if (!bleEnabled || !deviceConnected)
    return;
  docDisplay.clear();
  if (isLocal)
  {
    docDisplay["id"] = LOCAL_DISPLAY_ID;
    docDisplay["name"] = settings_name;
  }
  else
  {
    char idBuf[9];
    strncpy(idBuf, packet.body.deviceId, 8);
    idBuf[8] = '\0';
    docDisplay["id"] = idBuf;
    docDisplay["name"] = packet.body.name;
  }
  docDisplay["t"] = packet.ttl;
  docDisplay["type"] = (packet.body.type == TYPE_SOS) ? "SOS" : "POS";
  docDisplay["bat"] = packet.body.battery;
  docDisplay["v"] = (float)packet.body.voltage / 10.0;
  if (packet.flags & FLAG_GPS_VALID)
    docDisplay["gps_ok"] = 1;
  if (packet.body.lat != 0)
  {
    docDisplay["lat"] = packet.body.lat;
    docDisplay["lon"] = packet.body.lon;
    docDisplay["alt"] = packet.body.altitude;
  }
  char buffer[512];
  serializeJson(docDisplay, buffer, sizeof(buffer));
  pCharacteristic->setValue(buffer);
  pCharacteristic->notify();
}

bool isPacketUnique(FullPacket &packet)
{
  uint32_t msgHash = calculatePacketHash(packet);
  for (int i = 0; i < MESSAGE_HISTORY_SIZE; i++)
    if (messageHistory[i] == msgHash)
      return false;
  addHashToHistory(msgHash);
  return true;
}

void addHashToHistory(uint32_t hash)
{
  messageHistory[historyIndex] = hash;
  historyIndex = (historyIndex + 1) % MESSAGE_HISTORY_SIZE;
}

uint32_t calculatePacketHash(FullPacket &packet)
{
  uint32_t hash = 5381;
  for (int i = 0; i < 8; i++)
    hash = ((hash << 5) + hash) + packet.body.deviceId[i];
  hash = ((hash << 5) + hash) + packet.body.uid;
  return hash;
}

bool isSOS() { return digitalRead(SOS_SWITCH_PIN) == LOW; }

float readBatteryVoltage()
{
  digitalWrite(ADC_CTRL_PIN, HIGH);
  delay(15);
  uint32_t adcSum = 0;
  for (int i = 0; i < ADC_SAMPLES; i++)
  {
    adcSum += analogReadMilliVolts(BATTERY_ADC_PIN);
  }
  digitalWrite(ADC_CTRL_PIN, LOW);

  float adcAverage = (float)adcSum / (float)ADC_SAMPLES;

  // Змінено коефіцієнт з 4.9 на 5.1 (більш точний для плат Heltec)
  float rawVoltage = (adcAverage / 1000.0) * 5.12;

  if (filteredBatteryVoltage == 0.0)
    filteredBatteryVoltage = rawVoltage;
  else
    filteredBatteryVoltage = ADC_FILTER_ALPHA * rawVoltage + (1.0 - ADC_FILTER_ALPHA) * filteredBatteryVoltage;

  return filteredBatteryVoltage;
}

int calculateBatteryLevel(float voltage)
{
  // Жорсткі межі, щоб відсоток не вийшов за 0-100%
  if (voltage >= BATTERY_MAX_VOLTAGE)
    return 100;
  if (voltage <= BATTERY_MIN_VOLTAGE)
    return 0;

  // Чиста математика з плаваючою комою замість цілочисельного map()
  float percentage = ((voltage - BATTERY_MIN_VOLTAGE) / (BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE)) * 100.0;
  return (int)round(percentage);
}