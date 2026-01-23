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

// --- НАЛАШТУВАННЯ LORA ---
#define LORA_SS 10
#define LORA_RST 9
#define LORA_DIO0 2
#define LORA_FREQ 433E6

// --- ПІНИ ---
#define BTN 6
#define BLE_SWITCH_PIN 8
#define BATTERY_ADC_PIN 1 // GPIO1
#define GPS_RX 3
#define GPS_TX 4

// --- НАЛАШТУВАННЯ ПРИСТРОЮ ---
const char *DEVICE_ID = "fox";  // Ваш ID (string)
#define LOCAL_DISPLAY_ID 500001 // ID для локального дисплею (int)
const unsigned long SEND_INTERVAL = 5000;
#define TTL 3            // Time To Live
#define baud_rate 115200 // Use 115200 for Foxeer, 9600 for BN180

// --- КОНСТАНТИ БАТАРЕЇ ---
const float R1 = 100.0;
const float R2 = 100.0;

// --- ОБ'ЄКТИ ---
HardwareSerial gpsSerial(1);
TinyGPSPlus gps;
unsigned long lastSendTime = 0;

// --- ІСТОРІЯ ПОВІДОМЛЕНЬ (для дедуплікації) ---
#define MESSAGE_HISTORY_SIZE 100
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

// Прототипи функцій
void sendGPSData();
void sendLoRaMessage(String message);
void receiveLoRaMessage();
void forwardLoRaMessage(String message);
bool isMessageUnique(String message);
void addMessageToHistory(String message);
uint32_t calculateHash(String &msg);
float readBatteryVoltage();
int calculateBatteryLevel(float voltage);
bool isSOS();
void smartDelay(unsigned long duration); // НОВА ФУНКЦІЯ

// --- BLE CALLBACKS ---
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
  gpsSerial.begin(baud_rate, SERIAL_8N1, GPS_RX, GPS_TX);

  // Ініціалізація LoRa
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQ))
  {
    Serial.println("LoRa failed!");
    while (1)
      ;
  }
  Serial.println("LoRa initialized");

  // Ініціалізація BLE
  BLEDevice::init(("GAMMA-" + String(DEVICE_ID)).c_str());
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();

  bleEnabled = false;

  // ADC
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  // Power mgmt
  esp_sleep_enable_timer_wakeup(100000);
  setCpuFrequencyMhz(80);
}

// --- НОВА ФУНКЦІЯ: РОЗУМНА ЗАТРИМКА ---
// Вона замінює звичайний delay(), дозволяючи читати GPS під час очікування
void smartDelay(unsigned long duration)
{
  unsigned long start = millis();
  while (millis() - start < duration)
  {
    // 1. Обслуговуємо GPS
    while (gpsSerial.available() > 0)
    {
      gps.encode(gpsSerial.read());
    }
    // 2. Дозволяємо системним процесам (Watchdog, BLE стек) працювати
    yield();
  }
}

void loop()
{
  // 1. Керування BLE через перемикач
  bool switchState = digitalRead(BLE_SWITCH_PIN);
  if (switchState == LOW && bleEnabled)
  {
    pServer->getAdvertising()->stop();
    bleEnabled = false;
    deviceConnected = false;
    oldDeviceConnected = false;
    Serial.println("BLE advertising stopped");
  }
  else if (switchState == HIGH && !bleEnabled)
  {
    pServer->getAdvertising()->start();
    bleEnabled = true;
    Serial.println("BLE advertising started");
  }

  // 2. Прийом LoRa (ОНОВЛЕНО)
  receiveLoRaMessage();

  // 3. Читання GPS (також є всередині smartDelay, але залишаємо і тут)
  while (gpsSerial.available() > 0)
  {
    gps.encode(gpsSerial.read());
  }

  // 4. Відправка даних (Таймер)
  if (millis() - lastSendTime > SEND_INTERVAL)
  {
    sendGPSData();
    lastSendTime = millis();
  }

  // 5. Відновлення BLE з'єднання
  if (!deviceConnected && oldDeviceConnected)
  {
    smartDelay(500); // ВИКОРИСТОВУЄМО smartDelay
    if (bleEnabled)
      pServer->startAdvertising();
    oldDeviceConnected = deviceConnected;
  }
  if (deviceConnected && !oldDeviceConnected)
  {
    oldDeviceConnected = deviceConnected;
  }

  // Коротка пауза для стабільності, теж через smartDelay
  smartDelay(10);
}

// --- ЛОГІКА СЕНСОРІВ ---

bool isSOS() { return digitalRead(BTN) == LOW; }

float readBatteryVoltage()
{
  int adc = analogRead(BATTERY_ADC_PIN);
  float v_adc = adc / 4095.0 * 3.3;
  float v_batt = v_adc * (R1 + R2) / R2;
  return v_batt;
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

// --- ВІДПРАВКА ДАНИХ ---

void sendGPSData()
{
  StaticJsonDocument<256> jsonToSend;
  StaticJsonDocument<256> jsonToDisplay;

  float vbat = readBatteryVoltage();
  int batt = calculateBatteryLevel(vbat);

  jsonToSend["id"] = DEVICE_ID;
  jsonToSend["t"] = TTL;

  jsonToDisplay["id"] = LOCAL_DISPLAY_ID;
  jsonToDisplay["t"] = TTL;

  if (isSOS())
  {
    jsonToSend["type"] = "SOS";
    jsonToDisplay["type"] = "SOS";
  }
  else
  {
    jsonToSend["type"] = "POS";
    jsonToDisplay["type"] = "POS";

    jsonToSend["bat"] = batt;
    jsonToDisplay["bat"] = batt;

    jsonToSend["v"] = (int)(vbat * 100) / 100.0;
    jsonToDisplay["v"] = (int)(vbat * 100) / 100.0;

    // Перевіряємо валідність GPS і вік даних (менше 5 сек)
    if (gps.location.isValid() && gps.location.age() < 5000)
    {
      int32_t lat = (int32_t)(gps.location.lat() * 10000000);
      int32_t lon = (int32_t)(gps.location.lng() * 10000000);
      int alt = (int)gps.altitude.meters();

      jsonToSend["lat"] = lat;
      jsonToDisplay["lat"] = lat;

      jsonToSend["lon"] = lon;
      jsonToDisplay["lon"] = lon;

      jsonToSend["alt"] = alt;
      jsonToDisplay["alt"] = alt;
    }
  }

  String toSend, toDisplay;
  serializeJson(jsonToSend, toSend);
  serializeJson(jsonToDisplay, toDisplay);

  if (toSend.length() > 255)
  {
    Serial.println("ERROR: Message too long for LoRa!");
  }
  else
  {
    sendLoRaMessage(toSend);
  }

  Serial.println(toDisplay);

  if (bleEnabled && deviceConnected)
  {
    pCharacteristic->setValue(toDisplay.c_str());
    pCharacteristic->notify();
  }
}

void sendLoRaMessage(String message)
{
  LoRa.beginPacket();
  LoRa.print(message);
  LoRa.endPacket();
}

// --- ПРИЙОМ ДАНИХ (ПРОФЕСІЙНИЙ ВАРІАНТ) ---

void receiveLoRaMessage()
{
  int packetSize = LoRa.parsePacket();

  if (packetSize)
  {
    char incoming[256];
    int index = 0;

    while (LoRa.available() && index < 255)
    {
      incoming[index] = (char)LoRa.read();
      index++;
    }
    incoming[index] = '\0';

    StaticJsonDocument<512> receivedJson;
    DeserializationError error = deserializeJson(receivedJson, incoming);

    if (error)
    {
      Serial.print(F("JSON Error: "));
      Serial.println(error.f_str());
      return;
    }

    const char *idStr = receivedJson["id"];
    if (idStr != nullptr && strcmp(idStr, DEVICE_ID) == 0)
    {
      return;
    }

    String msgString = String(incoming);

    if (isMessageUnique(msgString))
    {
      addMessageToHistory(msgString);

      int currentTTL = receivedJson["t"] | 0;

      if (currentTTL > 0)
      {
        receivedJson["t"] = currentTTL - 1;

        String messageToForward;
        serializeJson(receivedJson, messageToForward);

        if (messageToForward.length() <= 255)
        {
          // !!! ТУТ ЗМІНА: ВИКОРИСТОВУЄМО smartDelay ЗАМІСТЬ delay
          smartDelay(random(200, 1000));

          forwardLoRaMessage(messageToForward);
          Serial.print("FWD TTL: ");
          Serial.println(currentTTL - 1);
        }
      }
      else
      {
        Serial.println("TTL expired");
      }

      Serial.println(incoming);

      if (bleEnabled && deviceConnected)
      {
        pCharacteristic->setValue((uint8_t *)incoming, strlen(incoming));
        pCharacteristic->notify();
      }
    }
  }
}

void forwardLoRaMessage(String message)
{
  LoRa.beginPacket();
  LoRa.print(message);
  LoRa.endPacket();
}

// --- ХЕШУВАННЯ ---

bool isMessageUnique(String message)
{
  uint32_t msgHash = calculateHash(message);
  for (int i = 0; i < MESSAGE_HISTORY_SIZE; i++)
  {
    if (messageHistory[i] == msgHash)
      return false;
  }
  return true;
}

void addMessageToHistory(String message)
{
  messageHistory[historyIndex] = calculateHash(message);
  historyIndex = (historyIndex + 1) % MESSAGE_HISTORY_SIZE;
}

uint32_t calculateHash(String &msg)
{
  StaticJsonDocument<512> jsonDoc;
  DeserializationError error = deserializeJson(jsonDoc, msg);

  if (error)
  {
    uint32_t hash = 5381;
    for (int i = 0; i < msg.length(); i++)
      hash = ((hash << 5) + hash) + msg.charAt(i);
    return hash;
  }

  jsonDoc.remove("t");

  String msgWithoutTTL;
  serializeJson(jsonDoc, msgWithoutTTL);

  uint32_t hash = 5381;
  for (int i = 0; i < msgWithoutTTL.length(); i++)
  {
    hash = ((hash << 5) + hash) + msgWithoutTTL.charAt(i);
  }
  return hash;
}