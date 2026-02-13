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
#include "driver/gpio.h" // Потрібно для налаштування пробудження по піну

// +++ WIFI ТА НАЛАШТУВАННЯ +++
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
#define GPS_RX_PIN 33
#define GPS_TX_PIN 34
#define GPS_BAUD 115200

#define BTN 0
#define BLE_SWITCH_PIN 46
#define BATTERY_ADC_PIN 1

// ==========================================
// НАЛАШТУВАННЯ СНУ
// ==========================================
#define STAY_AWAKE_MS 10000         // Скільки часу не спати після активності (10 сек)
#define SLEEP_DURATION_MS 5000      // Як на довго засинати (5 сек), якщо нічого не відбувається
unsigned long lastActivityTime = 0; // Час останньої події (RX, TX, Button)

// ==========================================
// ЗМІННІ НАЛАШТУВАНЬ
// ==========================================
String settings_password = "SuperStrongPassword123";
String settings_name = "Команда 1";

Preferences preferences;
WebServer server(80);
DNSServer dnsServer;
bool isConfigMode = false;

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
#define TOTAL_PACKET_SIZE (1 + 1 + 4 + 1 + IV_SIZE + ENCRYPTED_SIZE)

const char *DEVICE_ID_CONST = "h1";
#define LOCAL_DISPLAY_ID DEVICE_ID_CONST
const unsigned long SEND_INTERVAL = 15000; // Збільшив інтервал для економії
#define DEFAULT_TTL 3

#define FLAG_GPS_VALID 0x01
#define FLAG_RELAYED 0x02
#define TYPE_POS 0
#define TYPE_SOS 1

// --- GLOBALS ---
float filteredBatteryVoltage = 0.0;

// Hardware Objects
SX1262 radio = new Module(RADIO_CS_PIN, RADIO_DIO1_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN);
HardwareSerial gpsSerial(1);
TinyGPSPlus gps;

// Interrupt Logic
volatile bool operationDone = false;
bool transmitFlag = false; // true if currently transmitting

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
unsigned long lastGpsUpdateMs = 0;

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

#define SERVICE_UUID "a066b5b0-c522-4aa9-b148-8f24f37fcba6"
#define CHARACTERISTIC_UUID "d792d09f-1d6e-422b-991a-e2933e7d848b"

// ASYNC SEND VARIABLES
uint8_t asyncTxBuffer[256];
int asyncTxLen = 0;
bool packetPending = false;
unsigned long packetPendingSince = 0;
unsigned long randomBackoff = 0;

// ==========================================
// ПРОТОТИПИ
// ==========================================
void setFlag(void);
void handleLoRaInterrupt();
void trySendGPSData();
void prepareAndSend(FullPacket &packet, bool isRelay);
void checkSendQueue();
void enterLightSleep();

void receiveLoRaMessage();
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
void refreshActivity();

// ==========================================
// BLE CALLBACKS
// ==========================================
class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    deviceConnected = true;
    Serial.println("BLE Connected -> Sleep Disabled");
    refreshActivity();
  };
  void onDisconnect(BLEServer *pServer)
  {
    deviceConnected = false;
    Serial.println("BLE Disconnected");
    refreshActivity();
  }
};

// ==========================================
// CONFIGURATION LOGIC
// ==========================================
void loadSettings()
{
  preferences.begin("lora_config", false);
  settings_name = preferences.getString("dev_name", "Команда 1");
  settings_password = preferences.getString("aes_pass", "SuperStrongPassword123");
  preferences.end();
}

void checkDoubleReset()
{
  preferences.begin("boot_cnt", false);
  bool resetFlag = preferences.getBool("reset_flag", false);
  if (resetFlag)
  {
    isConfigMode = true;
    preferences.putBool("reset_flag", false);
  }
  else
  {
    preferences.putBool("reset_flag", true);
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
  }
}

void setupConfigMode()
{
  WiFi.mode(WIFI_AP);
  String apName = "GAMMA-" + String(DEVICE_ID_CONST);
  WiFi.softAP(apName.c_str(), "");
  dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
  dnsServer.start(53, "*", WiFi.softAPIP());

  server.onNotFound(handleRoot);
  server.on("/", handleRoot);
  server.on("/save", HTTP_POST, handleSave);
  server.begin();
}

void handleRoot()
{
  String html = "<!DOCTYPE html><html><body><h2>Config</h2><form action='/save' method='POST'><input name='devname' value='" + settings_name + "'><input name='pass' value='" + settings_password + "'><button type='submit'>SAVE</button></form></body></html>";
  server.send(200, "text/html", html);
}

void handleSave()
{
  if (server.hasArg("devname") && server.hasArg("pass"))
  {
    preferences.begin("lora_config", false);
    preferences.putString("dev_name", server.arg("devname"));
    preferences.putString("aes_pass", server.arg("pass"));
    preferences.end();
    server.send(200, "text/html", "Saved. Restarting...");
    delay(1000);
    ESP.restart();
  }
}

// ==========================================
// INTERRUPT FUNCTION
// ==========================================
#if defined(ESP8266) || defined(ESP32)
ICACHE_RAM_ATTR
#endif
void setFlag(void)
{
  operationDone = true;
}

// ==========================================
// SETUP
// ==========================================
void setup()
{
  Serial.begin(115200);

  pinMode(VEXT_PIN, OUTPUT);
  digitalWrite(VEXT_PIN, HIGH); // GPS ON
  pinMode(ADC_CTRL_PIN, OUTPUT);
  digitalWrite(ADC_CTRL_PIN, LOW);

  pinMode(BTN, INPUT_PULLUP);
  pinMode(BLE_SWITCH_PIN, INPUT_PULLUP);

  checkDoubleReset();
  loadSettings();

  if (isConfigMode)
  {
    setupConfigMode();
    return;
  }

  generateKeyFromPassword(settings_password.c_str());

  randomSeed(esp_random());

  // GPS
  gpsSerial.setRxBufferSize(2048);
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  // LORA INIT
  SPI.begin(RADIO_SCLK_PIN, RADIO_MISO_PIN, RADIO_MOSI_PIN, RADIO_CS_PIN);
  int state = radio.begin(LORA_FREQ);
  if (state == RADIOLIB_ERR_NONE)
  {
    Serial.println(F("LoRa init success!"));
  }
  else
  {
    Serial.print(F("LoRa failed, code "));
    Serial.println(state);
    while (true)
      delay(100);
  }

  radio.setOutputPower(22);
  radio.setSpreadingFactor(10);
  radio.setBandwidth(125.0);
  radio.setCodingRate(5);
  radio.setCRC(true);

  // *** ВАЖЛИВО ДЛЯ СНУ ***
  // Налаштовуємо переривання, щоб LoRa могла розбудити ESP32
  radio.setDio1Action(setFlag);

  // BLE INIT
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

  nextSendTime = millis() + random(2000, 5000);
  refreshActivity(); // Ініціалізувати таймер активності

  // ПОЧИНАЄМО СЛУХАТИ
  radio.startReceive();
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
  processGPS(32);

  // 1. ПЕРЕВІРКА КНОПКИ SOS (це активність)
  if (digitalRead(BTN) == LOW)
    refreshActivity();

  // 2. ОБРОБКА ПЕРЕРИВАНЬ LORA
  if (operationDone)
  {
    operationDone = false;
    refreshActivity(); // Якщо щось прийшло або пішло - не спимо
    handleLoRaInterrupt();
  }

  // 3. ОБРОБКА ЧЕРГИ НА ВІДПРАВКУ (CSMA/CA)
  checkSendQueue();

  // 4. ГЕНЕРАЦІЯ GPS (якщо час настав)
  if (millis() >= nextSendTime && !packetPending)
  {
    refreshActivity();
    trySendGPSData();
  }

  // 5. ОБРОБКА RELAY
  if (!packetPending)
  {
    processRelayQueue();
  }

  // === BLE LOGIC ===
  bool switchState = digitalRead(BLE_SWITCH_PIN);
  if (switchState == LOW && bleEnabled)
  {
    pServer->getAdvertising()->stop();
    bleEnabled = false;
    deviceConnected = false;
    refreshActivity();
  }
  else if (switchState == HIGH && !bleEnabled)
  {
    pServer->getAdvertising()->start();
    bleEnabled = true;
    refreshActivity();
  }

  if (bleEnabled)
  {
    if (!deviceConnected && oldDeviceConnected)
    {
      delay(500);
      pServer->startAdvertising();
      oldDeviceConnected = deviceConnected;
    }
    if (deviceConnected && !oldDeviceConnected)
    {
      oldDeviceConnected = deviceConnected;
    }
  }

  // === ЛОГІКА СНУ ===
  // Ми спимо, тільки якщо:
  // 1. Немає активного з'єднання BLE (бо сон порве зв'язок)
  // 2. Немає черги на відправку (packetPending == false)
  // 3. Минуло достатньо часу з останньої активності (STAY_AWAKE_MS)
  if (!deviceConnected && !packetPending && (millis() - lastActivityTime > STAY_AWAKE_MS))
  {
    enterLightSleep();
  }
}

// ==========================================
// SLEEP FUNCTION
// ==========================================
void refreshActivity()
{
  lastActivityTime = millis();
}

void enterLightSleep()
{
  // Налаштовуємо пробудження від таймера (щоб відправити GPS)
  esp_sleep_enable_timer_wakeup(SLEEP_DURATION_MS * 1000ULL); // в мікросекундах

  // Налаштовуємо пробудження від LoRa (Пін DIO1)
  // Для ESP32-S3 (Heltec v3) використовуємо gpio_wakeup logic
  gpio_wakeup_enable((gpio_num_t)RADIO_DIO1_PIN, GPIO_INTR_HIGH_LEVEL);
  esp_sleep_enable_gpio_wakeup();

  Serial.println(">>> Light Sleep...");
  Serial.flush(); // Чекаємо поки все виведеться в консоль

  // Входимо в Light Sleep (RAM зберігається, периферія працює частково)
  esp_light_sleep_start();

  // --- ТУТ ПРОГРАМА ПРОДОВЖУЄТЬСЯ ПІСЛЯ ПРОБУДЖЕННЯ ---
  Serial.println(">>> Woke Up!");

  // Після сну перевіряємо, хто нас розбудив
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

  if (wakeup_reason == ESP_SLEEP_WAKEUP_GPIO)
  {
    // Якщо розбудив LoRa радіомодуль, нам треба перевірити DIO1
    // Але RadioLib interrupt flag міг не спрацювати коректно у сні,
    // тому примусово ставимо прапорець, щоб loop() обробив пакет.
    operationDone = true;
  }

  // Оновлюємо таймер активності, щоб одразу знову не заснути
  refreshActivity();
}

// ==========================================
// HANDLERS & HELPERS
// ==========================================

void handleLoRaInterrupt()
{
  if (transmitFlag)
  {
    transmitFlag = false;
    Serial.println("TX Complete");
    radio.startReceive();
  }
  else
  {
    receiveLoRaMessage();
    radio.startReceive();
  }
}

void prepareAndSend(FullPacket &packet, bool isRelay)
{
  refreshActivity(); // Не спати під час підготовки
  packet.body.crc = calculateCRC((uint8_t *)&packet.body, ENCRYPTED_SIZE - sizeof(uint16_t));

  uint8_t iv[IV_SIZE];
  uint8_t cipherBuffer[ENCRYPTED_SIZE];
  for (int i = 0; i < IV_SIZE; i++)
    iv[i] = (uint8_t)esp_random();

  if (!aesCtrEncryptDecrypt((uint8_t *)&packet.body, cipherBuffer, ENCRYPTED_SIZE, iv))
    return;

  int idx = 0;
  asyncTxBuffer[idx++] = MESH_SIGNATURE;
  asyncTxBuffer[idx++] = packet.ttl;
  memcpy(&asyncTxBuffer[idx], &packet.timestamp, 4);
  idx += 4;
  asyncTxBuffer[idx++] = packet.flags;
  memcpy(&asyncTxBuffer[idx], iv, IV_SIZE);
  idx += IV_SIZE;
  memcpy(&asyncTxBuffer[idx], cipherBuffer, ENCRYPTED_SIZE);
  idx += ENCRYPTED_SIZE;

  asyncTxLen = idx;
  packetPending = true;
  packetPendingSince = millis();
  randomBackoff = random(50, 400);

  if (!isRelay)
    nextSendTime = millis() + SEND_INTERVAL + random(0, 3000);
}

void checkSendQueue()
{
  if (!packetPending)
    return;
  if (transmitFlag)
    return;

  if (millis() - packetPendingSince > randomBackoff)
  {
    refreshActivity(); // Не спати під час відправки
    int state = radio.startTransmit(asyncTxBuffer, asyncTxLen);
    if (state == RADIOLIB_ERR_NONE)
    {
      transmitFlag = true;
      packetPending = false;
      Serial.println("TX Started");
    }
    else
    {
      packetPending = false;
      radio.startReceive();
    }
  }
}

void receiveLoRaMessage()
{
  size_t len = radio.getPacketLength();
  if (len == 0)
    return;

  uint8_t rxBuffer[256];
  int state = radio.readData(rxBuffer, len);

  if (state != RADIOLIB_ERR_NONE)
    return;
  if (len < TOTAL_PACKET_SIZE)
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
  memcpy(iv, &rxBuffer[idx], IV_SIZE);
  idx += IV_SIZE;
  memcpy(cipherBuffer, &rxBuffer[idx], ENCRYPTED_SIZE);
  idx += ENCRYPTED_SIZE;

  uint8_t plainBuffer[ENCRYPTED_SIZE];
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
      Serial.print("RX from: ");
      Serial.println(fullRx.body.name);
      updateBLE(fullRx, false);

      if (incomingTTL > 0)
      {
        fullRx.ttl = incomingTTL - 1;
        fullRx.flags |= FLAG_RELAYED;
        enqueueRelay(fullRx);
      }
    }
  }
}

void trySendGPSData()
{
  bool emergency = isSOS();
  if (!gps.location.isValid() && !emergency)
  {
    nextSendTime = millis() + 5000;
    return;
  }

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

  addHashToHistory(calculatePacketHash(myPacket));
  updateBLE(myPacket, true);
  prepareAndSend(myPacket, false);
}

void enqueueRelay(FullPacket &packet)
{
  refreshActivity();
  int nextHead = (queueHead + 1) % QUEUE_SIZE;
  if (nextHead == queueTail)
    queueTail = (queueTail + 1) % QUEUE_SIZE;

  relayQueue[queueHead].packet = packet;
  relayQueue[queueHead].triggerTime = millis() + random(500, 2000);
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
    refreshActivity();
    prepareAndSend(item->packet, true);
    item->active = false;
    queueTail = (queueTail + 1) % QUEUE_SIZE;
  }
}

void updateBLE(FullPacket &packet, bool isLocal)
{
  if (!bleEnabled)
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
  size_t len = serializeJson(docDisplay, buffer, sizeof(buffer));
  pCharacteristic->setValue((uint8_t *)buffer, len);
  if (deviceConnected)
    pCharacteristic->notify();
}

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
  digitalWrite(ADC_CTRL_PIN, LOW);
  delayMicroseconds(100);
  uint32_t adcSum = 0;
  for (int i = 0; i < 5; i++)
    adcSum += analogRead(BATTERY_ADC_PIN);
  digitalWrite(ADC_CTRL_PIN, HIGH);
  float adcAverage = (float)adcSum / 5.0;
  float rawVoltage = (adcAverage / 4095.0) * 3.3 * 4.9;
  if (filteredBatteryVoltage == 0.0)
    filteredBatteryVoltage = rawVoltage;
  else
    filteredBatteryVoltage = 0.1 * rawVoltage + 0.9 * filteredBatteryVoltage;
  return filteredBatteryVoltage;
}

int calculateBatteryLevel(float voltage)
{
  float v = constrain(voltage, 3.2, 4.2);
  return map(v * 100, 320, 420, 0, 100);
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
  if (millis() - lastGpsUpdateMs > 3600000)
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