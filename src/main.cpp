#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
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

// +++ ДОДАТКОВІ БІБЛІОТЕКИ ДЛЯ WIFI ТА НАЛАШТУВАНЬ +++
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <DNSServer.h>

// +++ КРИПТОГРАФІЯ +++
#include "mbedtls/aes.h"
#include "mbedtls/sha256.h"

// ==========================================
// ЗМІННІ НАЛАШТУВАНЬ
// ==========================================
String settings_password = "SuperStrongPassword123";
String settings_name = "Команда 1";

// Об'єкти для збереження та веб-сервера
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

// Розміри
#define ENCRYPTED_SIZE sizeof(LoRaBody)
#define IV_SIZE 16
#define RADIO_PACKET_SIZE (1 + 1 + 4 + 1 + IV_SIZE + ENCRYPTED_SIZE)

// ==========================================
// ПІНИ ТА НАЛАШТУВАННЯ
// ==========================================
#define LORA_SS 10
#define LORA_RST 9
#define LORA_DIO0 2
#define LORA_FREQ 433E6

#define BTN 6
#define BLE_SWITCH_PIN 8
#define BATTERY_ADC_PIN 1
#define GPS_RX 3
#define GPS_TX 4

const char *DEVICE_ID_CONST = "bug1";
// === ЗМІНА ТУТ: LOCAL_DISPLAY_ID тепер дорівнює DEVICE_ID_CONST ===
#define LOCAL_DISPLAY_ID DEVICE_ID_CONST

const unsigned long SEND_INTERVAL = 5000;
#define DEFAULT_TTL 3
#define baud_rate 9600 // Use 115200 for Foxeer, 9600 for BN180

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

// Battery
const float R1 = 100.0;
const float R2 = 100.0;
const float BATTERY_CALIBRATION_FACTOR = 1.0;
const float ADC_FILTER_ALPHA = 0.1;
const int ADC_SAMPLES = 20;
const float BATTERY_MIN_VOLTAGE = 3.20;
const float BATTERY_MAX_VOLTAGE = 4.20;
float filteredBatteryVoltage = 0.0;

// Hardware Objects
HardwareSerial gpsSerial(1);
TinyGPSPlus gps;

// Relay Queue
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

// JSON & Logic
JsonDocument docDisplay;
unsigned long nextSendTime = 0;
bool isLoRaAwake = true;
unsigned long lastGpsUpdateMs = 0;
const unsigned long GPS_TIMEOUT_MS = 3600000;

// Duplicate Filter
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
void receiveLoRaMessage(int packetSize);
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
void checkDoubleReset();
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
  Serial.println("Loaded Settings Name: " + settings_name);
}

void checkDoubleReset()
{
  preferences.begin("boot_cnt", false);
  bool resetFlag = preferences.getBool("reset_flag", false);

  if (resetFlag)
  {
    isConfigMode = true;
    preferences.putBool("reset_flag", false);
    Serial.println(">>> DOUBLE RESET DETECTED! ENTERING CONFIG MODE <<<");
  }
  else
  {
    preferences.putBool("reset_flag", true);
    Serial.println(">>> Boot flag set. <<<");
  }
  preferences.end();
}

void clearResetFlag()
{
  static unsigned long bootTime = millis();
  static bool flagCleared = false;

  if (!flagCleared && millis() - bootTime > 3000)
  {
    preferences.begin("boot_cnt", false);
    preferences.putBool("reset_flag", false);
    preferences.end();
    flagCleared = true;
    Serial.println(">>> Boot flag cleared. <<<");
  }
}

void setupConfigMode()
{
  WiFi.mode(WIFI_AP);

  // === ЗМІНА ТУТ: Формуємо ім'я таке ж, як у BLE ===
  String apName = "GAMMA-" + String(DEVICE_ID_CONST);
  WiFi.softAP(apName.c_str(), "");

  dnsServer.start(53, "*", WiFi.softAPIP());

  server.on("/", handleRoot);
  server.on("/save", HTTP_POST, handleSave);
  server.onNotFound(handleRoot);
  server.begin();

  Serial.println("AP Started: " + apName);
  Serial.println("Go to 192.168.4.1");
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
  server.send(400, "text/plain", "Error: Empty fields");
}

// ==========================================
// SETUP
// ==========================================
void setup()
{
  pinMode(BTN, INPUT_PULLUP);
  pinMode(BLE_SWITCH_PIN, INPUT_PULLUP);
  Serial.begin(115200);

  checkDoubleReset();
  loadSettings();

  if (isConfigMode)
  {
    setupConfigMode();
    return;
  }

  // === ЗВИЧАЙНИЙ РЕЖИМ ===
  generateKeyFromPassword(settings_password.c_str());
  if (!isKeyInitialized)
  {
    Serial.println("CRITICAL: Key gen failed.");
    while (1)
      delay(100);
  }

  randomSeed(esp_random());
  setupPowerSaving();

  gpsSerial.setRxBufferSize(2048);
  gpsSerial.begin(baud_rate, SERIAL_8N1, GPS_RX, GPS_TX);

  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQ))
  {
    Serial.println("LoRa failed!");
    while (1)
      delay(100);
  }

  // +++ СИНХРОНІЗАЦІЯ ПАРАМЕТРІВ З HELTEC (RadioLib) +++
  LoRa.setSpreadingFactor(10);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setSyncWord(0x12);
  // +++++++++++++++++++++++++++++++++++++++++++++++++++++

  LoRa.setTxPower(20);
  LoRa.enableCrc();

  // BLE ініціалізація з тим самим іменем
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
  analogSetAttenuation(ADC_11db);

  for (int i = 0; i < QUEUE_SIZE; i++)
    relayQueue[i].active = false;

  wakeLoRa();
  nextSendTime = millis() + random(2000, 10000);
}

// ==========================================
// CRYPTO & HELPERS
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
  Serial.println("Key Init OK.");
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
  if (isConfigMode)
  {
    dnsServer.processNextRequest();
    server.handleClient();
    delay(5);
    return;
  }

  clearResetFlag();
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

    int packetSize = LoRa.parsePacket();
    if (packetSize > 0)
    {
      receiveLoRaMessage(packetSize);
      if (millis() >= nextSendTime - 1000)
        nextSendTime = millis() + random(2000, 5000);
    }
    else
    {
      if (txAllowed)
      {
        processRelayQueue();
        if (millis() >= nextSendTime)
        {
          sendGPSData();
          nextSendTime = millis() + SEND_INTERVAL + random(0, 3000);
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
// RX / TX / HELPERS (Code Identical to Previous)
// ==========================================

bool sendSafeLoRa(FullPacket &packet)
{
  packet.body.crc = calculateCRC((uint8_t *)&packet.body, ENCRYPTED_SIZE - sizeof(uint16_t));
  if (!isLoRaAwake)
    wakeLoRa();
  if (LoRa.parsePacket() > 0)
    return false;
  delay(random(10, 60));
  if (LoRa.parsePacket() > 0)
    return false;

  uint8_t iv[IV_SIZE];
  uint8_t cipherBuffer[ENCRYPTED_SIZE];
  for (int i = 0; i < IV_SIZE; i++)
    iv[i] = (uint8_t)esp_random();
  if (!aesCtrEncryptDecrypt((uint8_t *)&packet.body, cipherBuffer, ENCRYPTED_SIZE, iv))
    return false;

  if (LoRa.beginPacket() == 0)
    return false;
  LoRa.write(MESH_SIGNATURE);
  LoRa.write(packet.ttl);
  LoRa.write((uint8_t *)&packet.timestamp, 4);
  LoRa.write(packet.flags);
  LoRa.write(iv, IV_SIZE);
  LoRa.write(cipherBuffer, ENCRYPTED_SIZE);
  LoRa.endPacket();
  LoRa.receive();
  return true;
}

void relayBlind(uint8_t ttl, uint32_t timestamp, uint8_t flags, uint8_t *iv, uint8_t *cipherData)
{
  delay(random(50, 150));
  if (LoRa.parsePacket() == 0 && LoRa.beginPacket())
  {
    LoRa.write(MESH_SIGNATURE);
    LoRa.write(ttl - 1);
    LoRa.write((uint8_t *)&timestamp, 4);
    LoRa.write(flags);
    LoRa.write(iv, IV_SIZE);
    LoRa.write(cipherData, ENCRYPTED_SIZE);
    LoRa.endPacket();
    LoRa.receive();
    Serial.println(">>> Blind Relay");
  }
}

void receiveLoRaMessage(int packetSize)
{
  if (!isLoRaAwake || packetSize < RADIO_PACKET_SIZE)
  {
    if (packetSize > 0)
      while (LoRa.available())
        LoRa.read();
    return;
  }
  uint8_t sig = LoRa.read();
  if (sig != MESH_SIGNATURE)
    return;

  uint8_t incomingTTL = LoRa.read();
  uint32_t incomingTimestamp;
  LoRa.readBytes((uint8_t *)&incomingTimestamp, 4);
  uint8_t incomingFlags = LoRa.read();
  syncTimeFromMesh(incomingTimestamp, incomingFlags);

  uint8_t iv[IV_SIZE];
  uint8_t cipherBuffer[ENCRYPTED_SIZE];
  uint8_t plainBuffer[ENCRYPTED_SIZE];
  LoRa.readBytes(iv, IV_SIZE);
  LoRa.readBytes(cipherBuffer, ENCRYPTED_SIZE);
  while (LoRa.available())
    LoRa.read();

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
  else
    nextSendTime += 1000;
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
  LoRa.sleep();
  isLoRaAwake = false;
}
void wakeLoRa()
{
  LoRa.receive();
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
bool isSOS() { return digitalRead(BTN) == LOW; }
float readBatteryVoltage()
{
  uint32_t adcSum = 0;
  for (int i = 0; i < ADC_SAMPLES; i++)
    adcSum += analogRead(BATTERY_ADC_PIN);
  float adcAverage = (float)adcSum / (float)ADC_SAMPLES;
  float voltageDividerRatio = (R1 + R2) / R2;
  float rawVoltage = (adcAverage / 4095.0 * 3.3) * voltageDividerRatio * BATTERY_CALIBRATION_FACTOR;
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