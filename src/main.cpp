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
#include "esp_timer.h" // Бібліотека для безпечного таймера в перериваннях

// +++ ДОДАТКОВІ БІБЛІОТЕКИ ДЛЯ WIFI ТА НАЛАШТУВАНЬ +++
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <DNSServer.h>

// +++ КРИПТОГРАФІЯ +++
#include "mbedtls/aes.h"
#include "mbedtls/sha256.h"

// ==========================================
// ПІНИ ДЛЯ HELTEC WIRELESS TRACKER V1.1 / V1.2
// ==========================================
#define VEXT_PIN 3     // Керування живленням GPS/Sensor
#define ADC_CTRL_PIN 2 // Керування дільником напруги батареї

// PINS LoRa SX1262
#define RADIO_SCLK_PIN 9
#define RADIO_MISO_PIN 11
#define RADIO_MOSI_PIN 10
#define RADIO_CS_PIN 8
#define RADIO_DIO1_PIN 14
#define RADIO_RST_PIN 12
#define RADIO_BUSY_PIN 13

#define LORA_FREQ 433.0

// PINS GPS (UC6580)
#define GPS_RX 33
#define GPS_TX 34

#define BTN 0
#define BATTERY_ADC_PIN 1

// ПЕРЕМИКАЧІ
#define BLE_SWITCH_PIN 46
#define SOS_SWITCH_PIN 47 // ПІН ДЛЯ ТУМБЛЕРА SOS

// ==========================================
// ЗМІННІ НАЛАШТУВАНЬ
// ==========================================
String settings_password = "SuperStrongPassword123";
String settings_name = "Команда 1";

Preferences preferences;
WebServer server(80);
DNSServer dnsServer;
bool isConfigMode = false;

// УНІКАЛЬНИЙ ПІДПИС МЕРЕЖІ
#define MESH_SIGNATURE 0xB5

uint8_t derivedKey[16];
bool isKeyInitialized = false;

// ==========================================
// СТРУКТУРА ПАКЕТУ
// ==========================================
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

const char *DEVICE_ID_CONST = "h3";
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

// --- GLOBALS ---
int loopCounter = 0;
bool isLongListenMode = false;
bool hasSentThisMinute = false;

const float ADC_FILTER_ALPHA = 0.1;
const int ADC_SAMPLES = 20;
const float BATTERY_MIN_VOLTAGE = 3.20;
const float BATTERY_MAX_VOLTAGE = 4.20;
float filteredBatteryVoltage = 0.0;

HardwareSerial gpsSerial(1);
TinyGPSPlus gps;
SX1262 radio = new Module(RADIO_CS_PIN, RADIO_DIO1_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN);

// ==========================================
// ПЕРЕРИВАННЯ
// ==========================================
volatile bool rxFlag = false;
#if defined(ESP8266) || defined(ESP32)
ICACHE_RAM_ATTR
#endif
void setFlag(void)
{
  rxFlag = true;
}

// +++ БЕЗПЕЧНЕ ПЕРЕРИВАННЯ КНОПКИ З АНТИБРЯЗКОТОМ +++
volatile unsigned long isrLastTime = 0;
volatile bool wifiBtnFlag = false;

#if defined(ESP8266) || defined(ESP32)
ICACHE_RAM_ATTR
#endif
void wifiBtnISR(void)
{
  // Отримуємо час у мілісекундах безпечним для переривань методом
  unsigned long currentTime = (unsigned long)(esp_timer_get_time() / 1000ULL);

  // Ігноруємо всі мікро-вібрації кнопки, які стаються швидше ніж за 400мс
  if (currentTime - isrLastTime > 400)
  {
    wifiBtnFlag = true;
    isrLastTime = currentTime;
  }
}

// ==========================================
// ЧЕРГА ТА ІНШІ ЗМІННІ
// ==========================================
struct RelayItem
{
  FullPacket packet;
  unsigned long triggerTime;
  bool active;
  uint8_t retries;
};
#define QUEUE_SIZE 5
#define MAX_RELAY_RETRIES 3
RelayItem relayQueue[QUEUE_SIZE];
int queueHead = 0;
int queueTail = 0;

JsonDocument docDisplay;
unsigned long nextSendTime = 0;
bool isLoRaAwake = true;
unsigned long lastGpsUpdateMs = 0;
const unsigned long GPS_TIMEOUT_MS = 3600000;

#define MESSAGE_HISTORY_SIZE 50
uint32_t messageHistory[MESSAGE_HISTORY_SIZE];
int historyIndex = 0;

// BLE
BLECharacteristic *pCharacteristic;
BLEServer *pServer;
bool deviceConnected = false;
bool oldDeviceConnected = false;
bool bleEnabled = false;
unsigned long lastBleRetryTime = 0;

#define SERVICE_UUID "a066b5b0-c522-4aa9-b148-8f24f37fcba6"
#define CHARACTERISTIC_UUID "d792d09f-1d6e-422b-991a-e2933e7d848b"

// ==========================================
// ПРОТОТИПИ
// ==========================================
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
void processGPS(int maxBytes);
void updateBLE(FullPacket &packet, bool isLocal);
uint16_t calculateCRC(uint8_t *data, size_t len);
void generateKeyFromPassword(const char *password);
bool aesCtrEncryptDecrypt(const uint8_t *input, uint8_t *output, size_t length, uint8_t *iv);

void loadSettings();
void setupConfigMode();
void handleRoot();
void handleSave();

// ==========================================
// BLE CALLBACKS
// ==========================================
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

// ==========================================
// CONFIGURATION LOGIC (WEB SERVER)
// ==========================================
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
  html += "<h2>Налаштування</h2>";
  html += "<form action='/save' method='POST'>";
  html += "<label>Ім'я (до 15 символів):</label>";
  html += "<input type='text' name='devname' value='" + settings_name + "' maxlength='15'>";
  html += "<label>Ключ шифрування:</label>";
  html += "<input type='text' name='pass' value='" + settings_password + "'>";
  html += "<button type='submit'>ЗБЕРЕГТИ І ПЕРЕЗАВАНТАЖИТИ</button>";
  html += "</form></body></html>";
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
      delay(1000);

      ESP.restart();
    }
  }
  else
  {
    server.send(400, "text/plain", "Error: Empty fields");
  }
}

// ==========================================
// SETUP
// ==========================================
void setup()
{
  Serial.begin(115200);

  pinMode(VEXT_PIN, OUTPUT);
  digitalWrite(VEXT_PIN, HIGH);
  pinMode(ADC_CTRL_PIN, OUTPUT);
  digitalWrite(ADC_CTRL_PIN, LOW);

  pinMode(BTN, INPUT_PULLUP);
  pinMode(BLE_SWITCH_PIN, INPUT_PULLUP);
  pinMode(SOS_SWITCH_PIN, INPUT_PULLUP);

  loadSettings();

  // Прив'язуємо переривання та налаштовуємо пробудження
  attachInterrupt(digitalPinToInterrupt(BTN), wifiBtnISR, FALLING);
  gpio_wakeup_enable((gpio_num_t)BTN, GPIO_INTR_LOW_LEVEL);
  gpio_wakeup_enable((gpio_num_t)SOS_SWITCH_PIN, GPIO_INTR_LOW_LEVEL);
  esp_sleep_enable_gpio_wakeup();

  generateKeyFromPassword(settings_password.c_str());
  randomSeed(esp_random());
  setupPowerSaving();

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

  // BLE
  BLEDevice::init(("GAMMA-" + String(DEVICE_ID_CONST)).c_str());
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();

  analogReadResolution(12);

  for (int i = 0; i < QUEUE_SIZE; i++)
    relayQueue[i].active = false;

  wakeLoRa();
  nextSendTime = millis() + random(2000, 10000);
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
  // +++ ЛОГІКА АКТИВАЦІЇ WIFI +++
  if (wifiBtnFlag)
  {
    wifiBtnFlag = false;

    if (!isConfigMode)
    {
      Serial.println("Кнопка натиснута. Перехід у WiFi AP...");

      // 1. Жорстко блокуємо сон, щоб WiFi не відвалився
      esp_pm_config_esp32s3_t pm_config = {.max_freq_mhz = 80, .min_freq_mhz = 80, .light_sleep_enable = false};
      esp_pm_configure(&pm_config);

      // 2. Вбиваємо BLE, щоб розвантажити антену для WiFi
      if (bleEnabled)
      {
        pServer->getAdvertising()->stop();
        BLEDevice::deinit(false);
        bleEnabled = false;
      }

      // Перезапускаємо радіочіп WiFi
      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);
      delay(100);

      isConfigMode = true;
      setupConfigMode();
    }
    else
    {
      // Якщо WiFi ВЖЕ працює, друге натискання кнопки вимикає його
      Serial.println("Вихід з режиму налаштувань. Перезавантаження...");
      delay(100);
      ESP.restart(); // Ідеальний та чистий метод повернення в робочий режим
    }
  }

  if (isConfigMode)
  {
    dnsServer.processNextRequest();
    server.handleClient();
    delay(5);
    return; // Блокуємо решту коду, поки ми в налаштуваннях
  }

  // +++ ЛОГІКА МИТТЄВОГО ВІДПРАВЛЕННЯ SOS +++
  static bool lastSosState = HIGH;
  bool currentSosState = digitalRead(SOS_SWITCH_PIN);

  if (currentSosState == LOW && lastSosState == HIGH)
  {
    Serial.println("!!! SOS АКТИВОВАНО - МИТТЄВА ВІДПРАВКА !!!");
    hasSentThisMinute = false;
    nextSendTime = millis();
  }
  lastSosState = currentSosState;

  processGPS(64);

  bool activeWindow = true;
  bool txAllowed = true;
  struct tm timeinfo;
  static int lastMinute = -1;

  if (getLocalTime(&timeinfo))
  {
    if (timeinfo.tm_min != lastMinute)
    {
      lastMinute = timeinfo.tm_min;
      loopCounter++;
      isLongListenMode = (loopCounter % LONG_LISTEN_INTERVAL_CYCLES == 0);
      if (isLongListenMode)
        Serial.println(">>> LONG LISTEN <<<");

      hasSentThisMinute = false;
      nextSendTime = millis() + random(1000, 15000);
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

    if (rxFlag)
    {
      rxFlag = false;
      receiveLoRaMessage();

      if (millis() >= nextSendTime - 1000)
        nextSendTime += random(500, 1500);
    }
    else
    {
      if (txAllowed)
      {
        processRelayQueue();

        if (millis() >= nextSendTime && !hasSentThisMinute)
        {
          sendGPSData();
          if (!isSOS())
          {
            hasSentThisMinute = true;
          }
          else
          {
            nextSendTime = millis() + 10000; // Повторюємо SOS кожні 10 сек
          }
        }
      }
    }
    delay(1);
  }
  else
  {
    if (isLoRaAwake)
      sleepLoRa();
    processGPS(1024);
    delay(100);
  }

  // BLE Management
  bool switchState = digitalRead(BLE_SWITCH_PIN);
  if (switchState == LOW && bleEnabled)
  {
    pServer->getAdvertising()->stop();
    bleEnabled = false;
    deviceConnected = false;
  }
  else if (switchState == HIGH && !bleEnabled)
  {
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
}

// ==========================================
// RX / TX / HELPERS
// ==========================================

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
  delay(random(50, 150));
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
  Serial.println(">>> Blind Relay");
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
    Serial.println("Packet sent");
  }
}

void setupPowerSaving()
{
  esp_pm_config_esp32s3_t pm_config = {.max_freq_mhz = 80, .min_freq_mhz = 80, .light_sleep_enable = true};
  esp_pm_configure(&pm_config);
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
  time_t t = mktime(&tm);
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
  relayQueue[queueHead].triggerTime = millis() + random(100, 1000);
  relayQueue[queueHead].active = true;
  relayQueue[queueHead].retries = 0;
  queueHead = nextHead;
}

void processRelayQueue()
{
  if (queueHead == queueTail)
    return;
  RelayItem *item = &relayQueue[queueTail];
  if (item->active && millis() >= item->triggerTime)
  {
    if (sendSafeLoRa(item->packet))
    {
      item->active = false;
      queueTail = (queueTail + 1) % QUEUE_SIZE;
      Serial.println("Relayed.");
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
        item->triggerTime = millis() + random(1000, 3000);
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

bool isSOS()
{
  return digitalRead(SOS_SWITCH_PIN) == LOW;
}

float readBatteryVoltage()
{
  digitalWrite(ADC_CTRL_PIN, LOW);
  delayMicroseconds(100);
  uint32_t adcSum = 0;
  for (int i = 0; i < ADC_SAMPLES; i++)
    adcSum += analogRead(BATTERY_ADC_PIN);
  digitalWrite(ADC_CTRL_PIN, HIGH);

  float adcAverage = (float)adcSum / (float)ADC_SAMPLES;
  float rawVoltage = (adcAverage / 4095.0) * 3.3 * 4.9;

  if (filteredBatteryVoltage == 0.0)
    filteredBatteryVoltage = rawVoltage;
  else
    filteredBatteryVoltage = ADC_FILTER_ALPHA * rawVoltage + (1.0 - ADC_FILTER_ALPHA) * filteredBatteryVoltage;
  return filteredBatteryVoltage;
}

int calculateBatteryLevel(float voltage)
{
  float v = constrain(voltage, BATTERY_MIN_VOLTAGE, BATTERY_MAX_VOLTAGE);
  return map(v * 100, BATTERY_MIN_VOLTAGE * 100, BATTERY_MAX_VOLTAGE * 100, 0, 100);
}