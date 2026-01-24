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

// --- НАЛАШТУВАННЯ LORA ---
#define LORA_SS 10
#define LORA_RST 9
#define LORA_DIO0 2
#define LORA_FREQ 433E6

// --- ПІНИ ---
#define BTN 6
#define BLE_SWITCH_PIN 8
#define BATTERY_ADC_PIN 1
#define GPS_RX 3
#define GPS_TX 4

// --- НАЛАШТУВАННЯ ПРИСТРОЮ ---
const char *DEVICE_ID_CONST = "bug";
#define LOCAL_DISPLAY_ID 500001
const unsigned long SEND_INTERVAL = 5000;
#define DEFAULT_TTL 3
#define baud_rate 115200 // Use 115200 for Foxeer, 9600 for BN180

// --- TDMA ---
const int RX_WINDOW_SECONDS = 25;
const int TX_WINDOW_SECONDS = 20;

// --- КОНСТАНТИ БАТАРЕЇ ---
const float R1 = 100.0;
const float R2 = 100.0;

// --- ОБ'ЄКТИ ---
HardwareSerial gpsSerial(1);
TinyGPSPlus gps;

// --- БІНАРНА СТРУКТУРА ПАКЕТУ ---
// Додано поле flags для захисту синхронізації
struct __attribute__((packed)) LoRaPacket
{
  char deviceId[8];   // 8 байт
  uint32_t uid;       // 4 байти
  uint32_t timestamp; // 4 байти
  int32_t lat;        // 4 байти
  int32_t lon;        // 4 байти
  int16_t altitude;   // 2 байти
  uint8_t voltage;    // 1 байт
  uint8_t battery;    // 1 байт
  uint8_t ttl;        // 1 байт
  uint8_t type;       // 1 байт (POS/SOS)
  uint8_t flags;      // 1 байт (Нове поле: Статус GPS і т.д.)
};

// Бітові маски для прапорців
#define FLAG_GPS_VALID 0x01 // Біт 0: Час достовірний (з GPS)
#define FLAG_RELAYED 0x02   // Біт 1: Пакет був ретрансльований (опціонально)

#define TYPE_POS 0
#define TYPE_SOS 1

// --- БУФЕРИ ---
LoRaPacket rxPacket;

struct RelayItem
{
  LoRaPacket packet;
  unsigned long triggerTime;
  bool active;
};

#define QUEUE_SIZE 5
RelayItem relayQueue[QUEUE_SIZE];
int queueHead = 0;
int queueTail = 0;

JsonDocument docDisplay;

// --- ЧАС ---
unsigned long nextSendTime = 0;
bool isLoRaAwake = true;
unsigned long lastGpsUpdateMs = 0;
const unsigned long GPS_TIMEOUT_MS = 3600000;

// --- ІСТОРІЯ ---
#define MESSAGE_HISTORY_SIZE 50
uint32_t messageHistory[MESSAGE_HISTORY_SIZE];
int historyIndex = 0;

// --- BLE ---
BLECharacteristic *pCharacteristic;
BLEServer *pServer;
bool deviceConnected = false;
bool oldDeviceConnected = false;
bool bleEnabled = false;

#define SERVICE_UUID "a066b5b0-c522-4aa9-b148-8f24f37fcba6"
#define CHARACTERISTIC_UUID "d792d09f-1d6e-422b-991a-e2933e7d848b"

// Прототипи
void sendGPSData();
void sendBinaryLoRa(LoRaPacket &packet);
void receiveLoRaMessage(int packetSize);
void enqueueRelay(LoRaPacket &packet);
void processRelayQueue();
bool isPacketUnique(LoRaPacket &packet);
void addHashToHistory(uint32_t hash);
uint32_t calculatePacketHash(LoRaPacket &packet);
void syncTimeFromMesh(uint32_t remoteTimestamp, uint8_t remoteFlags); // Оновлено
void syncWithGPS();
float readBatteryVoltage();
int calculateBatteryLevel(float voltage);
bool isSOS();
void setupPowerSaving();
void sleepLoRa();
void wakeLoRa();
void processGPS(int maxBytes);
void updateBLE(LoRaPacket &packet, bool isLocal);

// BLE Callbacks
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

void setup()
{
  pinMode(BTN, INPUT_PULLUP);
  pinMode(BLE_SWITCH_PIN, INPUT_PULLUP);
  Serial.begin(115200);

  randomSeed(esp_random());
  setupPowerSaving();

  gpsSerial.setRxBufferSize(2048);
  gpsSerial.begin(baud_rate, SERIAL_8N1, GPS_RX, GPS_TX);

  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQ))
  {
    Serial.println("LoRa failed!");
    while (1)
    {
      delay(100);
    }
  }
  LoRa.setTxPower(20);

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
}

void setupPowerSaving()
{
  esp_pm_config_esp32s3_t pm_config = {
      .max_freq_mhz = 80,
      .min_freq_mhz = 80,
      .light_sleep_enable = true};
  esp_pm_configure(&pm_config);
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
  {
    syncWithGPS();
  }
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
  settimeofday(&now, NULL);
  lastGpsUpdateMs = millis();
}

void syncTimeFromMesh(uint32_t remoteTimestamp, uint8_t remoteFlags)
{
  // 1. ПЕРЕВІРКА ПРАПОРЦЯ:
  // Якщо у відправника немає валідного GPS, ми ігноруємо його час.
  // Це зупиняє поширення "вірусу" неправильного часу.
  if (!(remoteFlags & FLAG_GPS_VALID))
  {
    return;
  }

  // 2. СИНХРОНІЗАЦІЯ:
  // Приймаємо час тільки якщо наш власний GPS застарів
  if (millis() - lastGpsUpdateMs > GPS_TIMEOUT_MS)
  {
    time_t now;
    time(&now);
    // Захист від дрейфу та "нульового" часу
    if (remoteTimestamp > 1000000000 && abs((long)now - (long)remoteTimestamp) > 2)
    {
      struct timeval tv = {.tv_sec = (time_t)remoteTimestamp, .tv_usec = 0};
      settimeofday(&tv, NULL);
      // Serial.println("Synced time from valid GPS node");
    }
  }
}

void loop()
{
  processGPS(64);

  bool activeWindow = true;
  bool txAllowed = true;
  struct tm timeinfo;

  if (getLocalTime(&timeinfo))
  {
    activeWindow = (timeinfo.tm_sec < RX_WINDOW_SECONDS);
    txAllowed = (timeinfo.tm_sec < TX_WINDOW_SECONDS);
  }

  if (activeWindow)
  {
    if (!isLoRaAwake)
    {
      wakeLoRa();
      nextSendTime = millis() + random(2000, 15000);
    }

    int packetSize = LoRa.parsePacket();

    if (packetSize > 0)
    {
      receiveLoRaMessage(packetSize);
      if (millis() < nextSendTime)
      {
        nextSendTime += random(500, 1000);
      }
    }
    else
    {
      if (txAllowed)
      {
        processRelayQueue();
        if (millis() >= nextSendTime)
        {
          sendGPSData();
          nextSendTime = millis() + SEND_INTERVAL + random(0, 2000);
        }
      }
    }
    delay(5);
  }
  else
  {
    if (isLoRaAwake)
      sleepLoRa();
    processGPS(1024);
    delay(100);
  }

  // BLE Logic
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
    delay(500);
    if (bleEnabled)
      pServer->startAdvertising();
    oldDeviceConnected = deviceConnected;
  }
  if (deviceConnected && !oldDeviceConnected)
    oldDeviceConnected = deviceConnected;
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

// --- RELAY LOGIC ---

void enqueueRelay(LoRaPacket &packet)
{
  int nextHead = (queueHead + 1) % QUEUE_SIZE;
  if (nextHead == queueTail)
  {
    queueTail = (queueTail + 1) % QUEUE_SIZE;
  }
  relayQueue[queueHead].packet = packet;
  relayQueue[queueHead].triggerTime = millis() + random(50, 400);
  relayQueue[queueHead].active = true;
  queueHead = nextHead;
}

void processRelayQueue()
{
  if (queueHead == queueTail)
    return;
  RelayItem *item = &relayQueue[queueTail];

  if (item->active && millis() >= item->triggerTime)
  {
    sendBinaryLoRa(item->packet);
    item->active = false;
    queueTail = (queueTail + 1) % QUEUE_SIZE;
    Serial.println("Relayed binary packet.");
  }
}

void sendBinaryLoRa(LoRaPacket &packet)
{
  if (!isLoRaAwake)
    return;
  LoRa.beginPacket();
  LoRa.write((uint8_t *)&packet, sizeof(LoRaPacket));
  LoRa.endPacket();
  LoRa.receive();
}

void receiveLoRaMessage(int packetSize)
{
  if (!isLoRaAwake || packetSize == 0)
    return;

  if (packetSize != sizeof(LoRaPacket))
  {
    while (LoRa.available())
      LoRa.read();
    return;
  }

  LoRa.readBytes((uint8_t *)&rxPacket, sizeof(LoRaPacket));

  // 1. БЕЗПЕЧНА СИНХРОНІЗАЦІЯ
  // Передаємо час І статус прапорців
  syncTimeFromMesh(rxPacket.timestamp, rxPacket.flags);

  if (strncmp(rxPacket.deviceId, DEVICE_ID_CONST, 8) == 0)
    return;

  if (isPacketUnique(rxPacket))
  {
    if (rxPacket.ttl > 0)
    {
      rxPacket.ttl--;
      // При ретрансляції ми зберігаємо оригінальні прапорці!
      // Якщо оригінал мав GPS, цей прапорець залишається.
      // Це дозволяє нодам далі в мережі синхронізуватись від ретранслятора.
      enqueueRelay(rxPacket);
    }
    updateBLE(rxPacket, false);
  }
}

void sendGPSData()
{
  LoRaPacket myPacket;
  float vbat = readBatteryVoltage();

  memset(myPacket.deviceId, 0, 8);
  strncpy(myPacket.deviceId, DEVICE_ID_CONST, 8);

  myPacket.uid = esp_random();
  myPacket.ttl = DEFAULT_TTL;
  myPacket.flags = 0; // Скидаємо прапорці

  time_t now;
  time(&now);
  myPacket.timestamp = (uint32_t)now;

  myPacket.type = isSOS() ? TYPE_SOS : TYPE_POS;
  myPacket.battery = calculateBatteryLevel(vbat);
  myPacket.voltage = (uint8_t)(vbat * 10);

  if (gps.location.isValid())
  {
    myPacket.lat = (int32_t)(gps.location.lat() * 10000000);
    myPacket.lon = (int32_t)(gps.location.lng() * 10000000);
    myPacket.altitude = (int16_t)gps.altitude.meters();

    // ВСТАНОВЛЮЄМО ПРАПОРЕЦЬ ДОВІРИ
    // Тільки якщо ми маємо валідний GPS сигнал прямо зараз
    if (gps.time.isValid() && gps.time.age() < 2000)
    {
      myPacket.flags |= FLAG_GPS_VALID;
    }
  }
  else
  {
    myPacket.lat = 0;
    myPacket.lon = 0;
    myPacket.altitude = 0;
  }

  addHashToHistory(calculatePacketHash(myPacket));
  sendBinaryLoRa(myPacket);
  updateBLE(myPacket, true);
}

void updateBLE(LoRaPacket &packet, bool isLocal)
{
  if (!bleEnabled || !deviceConnected)
    return;

  docDisplay.clear();

  if (isLocal)
  {
    docDisplay["id"] = LOCAL_DISPLAY_ID;
  }
  else
  {
    char idBuf[9];
    strncpy(idBuf, packet.deviceId, 8);
    idBuf[8] = '\0';
    docDisplay["id"] = idBuf;
  }

  docDisplay["t"] = packet.ttl;
  docDisplay["type"] = (packet.type == TYPE_SOS) ? "SOS" : "POS";
  docDisplay["bat"] = packet.battery;
  docDisplay["v"] = (float)packet.voltage / 10.0;

  // Додаємо статус GPS для налагодження (опціонально)
  if (packet.flags & FLAG_GPS_VALID)
  {
    docDisplay["gps_ok"] = 1;
  }

  if (packet.lat != 0)
  {
    docDisplay["lat"] = packet.lat;
    docDisplay["lon"] = packet.lon;
    docDisplay["alt"] = packet.altitude;
  }

  char buffer[512];
  serializeJson(docDisplay, buffer, sizeof(buffer));
  pCharacteristic->setValue(buffer);
  pCharacteristic->notify();
}

bool isPacketUnique(LoRaPacket &packet)
{
  uint32_t msgHash = calculatePacketHash(packet);
  for (int i = 0; i < MESSAGE_HISTORY_SIZE; i++)
  {
    if (messageHistory[i] == msgHash)
      return false;
  }
  addHashToHistory(msgHash);
  return true;
}

void addHashToHistory(uint32_t hash)
{
  messageHistory[historyIndex] = hash;
  historyIndex = (historyIndex + 1) % MESSAGE_HISTORY_SIZE;
}

uint32_t calculatePacketHash(LoRaPacket &packet)
{
  uint32_t hash = 5381;
  for (int i = 0; i < 8; i++)
  {
    hash = ((hash << 5) + hash) + packet.deviceId[i];
  }
  hash = ((hash << 5) + hash) + packet.uid;
  return hash;
}

bool isSOS() { return digitalRead(BTN) == LOW; }

float readBatteryVoltage()
{
  int adc = analogRead(BATTERY_ADC_PIN);
  return (adc / 4095.0 * 3.3) * (R1 + R2) / R2;
}

int calculateBatteryLevel(float voltage)
{
  if (voltage >= 4.00)
    return 100;
  if (voltage >= 3.80)
    return 75;
  if (voltage >= 3.60)
    return 50;
  if (voltage >= 3.40)
    return 20;
  return 1;
}