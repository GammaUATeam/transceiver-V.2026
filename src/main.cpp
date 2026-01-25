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
const char *DEVICE_NAME = "Команда 1";

#define LOCAL_DISPLAY_ID 500001
const unsigned long SEND_INTERVAL = 5000;
#define DEFAULT_TTL 3
#define baud_rate 9600 // Use 115200 for Foxeer, 9600 for BN180

// --- TDMA & SYNCHRONIZATION ---
const int RX_WINDOW_SECONDS = 25;
const int TX_WINDOW_SECONDS = 20;
const int LONG_LISTEN_INTERVAL_CYCLES = 200;
int loopCounter = 0;
bool isLongListenMode = false;

// --- КОНСТАНТИ БАТАРЕЇ ---
const float R1 = 100.0;
const float R2 = 100.0;
const float BATTERY_CALIBRATION_FACTOR = 1.0;

const float ADC_FILTER_ALPHA = 0.1;
const int ADC_SAMPLES = 20;
const float BATTERY_MIN_VOLTAGE = 3.20;
const float BATTERY_MAX_VOLTAGE = 4.20;
float filteredBatteryVoltage = 0.0;

// --- ОБ'ЄКТИ ---
HardwareSerial gpsSerial(1);
TinyGPSPlus gps;

// --- СТРУКТУРА ПАКЕТУ ---
struct __attribute__((packed)) LoRaPacket
{
  char deviceId[8];
  char name[16];
  uint32_t uid;
  uint32_t timestamp;
  int32_t lat;
  int32_t lon;
  int16_t altitude;
  uint8_t voltage;
  uint8_t battery;
  uint8_t ttl;
  uint8_t type;
  uint8_t flags;
  uint16_t crc;
};

#define FLAG_GPS_VALID 0x01
#define FLAG_RELAYED 0x02
#define TYPE_POS 0
#define TYPE_SOS 1

// --- БУФЕРИ ---
LoRaPacket rxPacket;

struct RelayItem
{
  LoRaPacket packet;
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
unsigned long lastBleRetryTime = 0; // Для неблокуючого таймера

#define SERVICE_UUID "a066b5b0-c522-4aa9-b148-8f24f37fcba6"
#define CHARACTERISTIC_UUID "d792d09f-1d6e-422b-991a-e2933e7d848b"

// --- ПРОТОТИПИ ---
void sendGPSData();
bool sendSafeLoRa(LoRaPacket &packet);
void receiveLoRaMessage(int packetSize);
void enqueueRelay(LoRaPacket &packet);
void processRelayQueue();
bool isPacketUnique(LoRaPacket &packet);
void addHashToHistory(uint32_t hash);
uint32_t calculatePacketHash(LoRaPacket &packet);
void syncTimeFromMesh(uint32_t remoteTimestamp, uint8_t remoteFlags);
void syncWithGPS();
float readBatteryVoltage();
int calculateBatteryLevel(float voltage);
bool isSOS();
void setupPowerSaving();
void sleepLoRa();
void wakeLoRa();
void processGPS(int maxBytes);
void updateBLE(LoRaPacket &packet, bool isLocal);
uint16_t calculateCRC(uint8_t *data, size_t len);

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
      delay(100);
  }
  LoRa.setTxPower(20);
  LoRa.enableCrc();

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

void setupPowerSaving()
{
  esp_pm_config_esp32s3_t pm_config = {
      .max_freq_mhz = 80,
      .min_freq_mhz = 80,
      .light_sleep_enable = true};
  esp_pm_configure(&pm_config);
}

uint16_t calculateCRC(uint8_t *data, size_t len)
{
  uint16_t crc = 0;
  for (size_t i = 0; i < len; i++)
  {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t j = 0; j < 8; j++)
    {
      if (crc & 0x8000)
        crc = (crc << 1) ^ 0x1021;
      else
        crc <<= 1;
    }
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
      Serial.println("Synced time from valid GPS node");
    }
  }
}

void loop()
{
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
        Serial.println(">>> LONG LISTEN MODE <<<");
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
      {
        nextSendTime = millis() + random(2000, 5000);
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
          nextSendTime = millis() + SEND_INTERVAL + random(0, 3000);
        }
      }
    }
    // Зменшили з 5 до 1 мс для швидшої реакції
    delay(1);
  }
  else
  {
    if (isLoRaAwake)
      sleepLoRa();
    processGPS(1024);
    delay(100);
  }

  // BLE Management (ВИПРАВЛЕНО БЛОКУВАННЯ)
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

  // Тут було delay(500), тепер неблокуючий код
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
  // Цей блок має виконуватись відразу при підключенні
  if (deviceConnected && !oldDeviceConnected)
  {
    oldDeviceConnected = deviceConnected;
    lastBleRetryTime = 0;
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

void enqueueRelay(LoRaPacket &packet)
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
      Serial.println("Relay deferred: Busy");
      item->retries++;
      if (item->retries > MAX_RELAY_RETRIES)
      {
        item->active = false;
        queueTail = (queueTail + 1) % QUEUE_SIZE;
      }
      else
      {
        item->triggerTime = millis() + random(1000, 3000);
      }
    }
  }
}

bool sendSafeLoRa(LoRaPacket &packet)
{
  packet.crc = calculateCRC((uint8_t *)&packet, sizeof(LoRaPacket) - sizeof(uint16_t));

  if (!isLoRaAwake)
    wakeLoRa();

  int existingPacketSize = LoRa.parsePacket();
  if (existingPacketSize > 0)
  {
    receiveLoRaMessage(existingPacketSize);
    return false;
  }

  unsigned long jitter = random(10, 60);
  delay(jitter); // Це блокування потрібне для CSMA

  if (LoRa.parsePacket() > 0)
    return false;

  if (LoRa.beginPacket() == 0)
    return false;

  LoRa.write((uint8_t *)&packet, sizeof(LoRaPacket));
  LoRa.endPacket();

  LoRa.receive();
  return true;
}

void receiveLoRaMessage(int packetSize)
{
  if (!isLoRaAwake || packetSize == 0)
    return;

  if (packetSize != sizeof(LoRaPacket))
  {
    while (LoRa.available())
      LoRa.read();
    Serial.println("Error: Wrong packet size");
    return;
  }

  LoRa.readBytes((uint8_t *)&rxPacket, sizeof(LoRaPacket));

  uint16_t receivedCRC = rxPacket.crc;
  uint16_t calculatedCRC = calculateCRC((uint8_t *)&rxPacket, sizeof(LoRaPacket) - sizeof(uint16_t));

  if (receivedCRC != calculatedCRC)
  {
    Serial.println("Error: CRC Mismatch! Packet Corrupted.");
    return;
  }

  syncTimeFromMesh(rxPacket.timestamp, rxPacket.flags);

  if (strncmp(rxPacket.deviceId, DEVICE_ID_CONST, 8) == 0)
    return;

  if (isPacketUnique(rxPacket))
  {
    updateBLE(rxPacket, false);

    if (rxPacket.ttl > 0)
    {
      rxPacket.ttl--;
      rxPacket.flags |= FLAG_RELAYED;
      enqueueRelay(rxPacket);
    }
  }
}

void sendGPSData()
{
  LoRaPacket myPacket;
  float vbat = readBatteryVoltage(); // Тут є затримка 40мс, але це допустимо

  memset(myPacket.deviceId, 0, 8);
  strncpy(myPacket.deviceId, DEVICE_ID_CONST, 8);

  memset(myPacket.name, 0, 16);
  strncpy(myPacket.name, DEVICE_NAME, 16);
  myPacket.name[15] = '\0';

  myPacket.uid = esp_random();
  myPacket.ttl = DEFAULT_TTL;
  myPacket.flags = 0;

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

  if (sendSafeLoRa(myPacket))
  {
    addHashToHistory(calculatePacketHash(myPacket));
    updateBLE(myPacket, true);
  }
  else
  {
    Serial.println("GPS Send skipped: Busy");
    nextSendTime += 1000;
  }
}

void updateBLE(LoRaPacket &packet, bool isLocal)
{
  if (!bleEnabled || !deviceConnected)
    return;

  docDisplay.clear();

  if (isLocal)
  {
    docDisplay["id"] = LOCAL_DISPLAY_ID;
    docDisplay["name"] = DEVICE_NAME;
  }
  else
  {
    char idBuf[9];
    strncpy(idBuf, packet.deviceId, 8);
    idBuf[8] = '\0';
    docDisplay["id"] = idBuf;
    docDisplay["name"] = packet.name;
  }

  docDisplay["t"] = packet.ttl;
  docDisplay["type"] = (packet.type == TYPE_SOS) ? "SOS" : "POS";
  docDisplay["bat"] = packet.battery;
  docDisplay["v"] = (float)packet.voltage / 10.0;
  if (packet.flags & FLAG_GPS_VALID)
    docDisplay["gps_ok"] = 1;

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
    hash = ((hash << 5) + hash) + packet.deviceId[i];
  hash = ((hash << 5) + hash) + packet.uid;
  return hash;
}

bool isSOS() { return digitalRead(BTN) == LOW; }

float readBatteryVoltage()
{
  uint32_t adcSum = 0;
  for (int i = 0; i < ADC_SAMPLES; i++)
  {
    adcSum += analogRead(BATTERY_ADC_PIN);
  }
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
  int percent = map(v * 100, BATTERY_MIN_VOLTAGE * 100, BATTERY_MAX_VOLTAGE * 100, 0, 100);
  return percent;
}