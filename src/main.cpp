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

// LoRa Pin Definitions
#define LORA_SS 10
#define LORA_RST 9
#define LORA_DIO0 2
#define LORA_FREQ 433E6

// Button and Switch Pins
#define BTN 6
#define BLE_SWITCH_PIN 8

// Device and Display IDs (DEVICE_ID is now a string)
const char *DEVICE_ID = "tiny-lion"; // Changed to string type
#define LOCAL_DISPLAY_ID 500001      // Remains an integer for local display purposes

// GPS Pin Definitions
#define baud_rate 115200 // Use 115200 for Foxeer, 9600 for BN180
#define GPS_RX 3
#define GPS_TX 4

// Battery ADC Pin and Voltage Divider Resistors
#define BATTERY_ADC_PIN 1 // GPIO1
const float R1 = 100.0;   // kΩ
const float R2 = 100.0;   // kΩ

// Data Sending Interval
const unsigned long SEND_INTERVAL = 5000; // 5 seconds

// GPS and LoRa Objects
HardwareSerial gpsSerial(1);
TinyGPSPlus gps;
unsigned long lastSendTime = 0;

// Message History for Duplicate Detection
#define MESSAGE_HISTORY_SIZE 100
uint32_t messageHistory[MESSAGE_HISTORY_SIZE];
int historyIndex = 0;

// BLE Variables
BLECharacteristic *pCharacteristic;
BLEServer *pServer;
bool deviceConnected = false;
bool oldDeviceConnected = false;
bool bleEnabled = false;

// BLE Service and Characteristic UUIDs
#define SERVICE_UUID "a066b5b0-c522-4aa9-b148-8f24f37fcba6"
#define CHARACTERISTIC_UUID "d792d09f-1d6e-422b-991a-e2933e7d848b"

/**
 * @brief Callback class for BLE server events.
 */
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

/**
 * @brief Setup function: Initializes serial communication, LoRa, BLE, and ADC.
 */
void setup()
{
  // Configure pin modes
  pinMode(BTN, INPUT_PULLUP);
  pinMode(BLE_SWITCH_PIN, INPUT_PULLUP);
  Serial.begin(115200); // Initialize serial monitor

  // Initialize GPS serial communication
  gpsSerial.begin(baud_rate, SERIAL_8N1, GPS_RX, GPS_TX);

  // Initialize LoRa module
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQ))
  {
    Serial.println("LoRa failed!");
    while (1)
      ; // Halt if LoRa initialization fails
  }
  Serial.println("LoRa initialized");

  // Initialize BLE device with a name including DEVICE_ID
  BLEDevice::init(("GAMMA-GPS-" + String(DEVICE_ID)).c_str()); // DEVICE_ID is now a const char*, String() converts it
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create BLE service and characteristic
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_NOTIFY);
  pCharacteristic->addDescriptor(new BLE2902()); // Add client characteristic configuration descriptor
  pService->start();                             // Start the BLE service

  bleEnabled = false; // BLE is initially disabled until switch is HIGH

  // Configure ADC for battery voltage reading
  analogReadResolution(12);       // Set ADC resolution to 12 bits (0-4095)
  analogSetAttenuation(ADC_11db); // Set attenuation for ~3.6V max input

  // Configure ESP32 deep sleep (not actively used in this loop, but set up)
  esp_sleep_enable_timer_wakeup(100000); // Enable wakeup by timer (100ms)
  setCpuFrequencyMhz(80);                // Set CPU frequency to 80MHz
}

/**
 * @brief Main loop function: Handles BLE switch, LoRa reception, GPS data, and sending.
 */
void loop()
{
  // Check BLE switch state to enable/disable advertising
  bool switchState = digitalRead(BLE_SWITCH_PIN);
  if (switchState == LOW && bleEnabled)
  {
    pServer->getAdvertising()->stop();
    bleEnabled = false;
    deviceConnected = false;
    oldDeviceConnected = false;
    Serial.println("BLE advertising stopped by switch");
  }
  else if (switchState == HIGH && !bleEnabled)
  {
    pServer->getAdvertising()->start();
    bleEnabled = true;
    Serial.println("BLE advertising started by switch");
  }

  // Process incoming LoRa messages
  receiveLoRaMessage();

  // Process incoming GPS data
  while (gpsSerial.available() > 0)
  {
    gps.encode(gpsSerial.read());
  }

  // Send GPS data at regular intervals
  if (millis() - lastSendTime > SEND_INTERVAL)
  {
    sendGPSData();
    lastSendTime = millis();
  }

  // Handle BLE connection state changes (restart advertising if disconnected)
  if (!deviceConnected && oldDeviceConnected)
  {
    delay(500); // Small delay before restarting advertising
    if (bleEnabled)
    {
      pServer->startAdvertising();
      Serial.println("BLE advertising restarted");
    }
    oldDeviceConnected = deviceConnected;
  }

  if (deviceConnected && !oldDeviceConnected)
  {
    oldDeviceConnected = deviceConnected;
  }

  delay(10); // Small delay to prevent watchdog timer issues
}

/**
 * @brief Checks if the SOS button is pressed.
 * @return True if SOS button is pressed (LOW), false otherwise.
 */
bool isSOS()
{
  return digitalRead(BTN) == LOW;
}

/**
 * @brief Reads the battery voltage using ADC and voltage divider.
 * @return Calculated battery voltage in Volts.
 */
float readBatteryVoltage()
{
  int adc = analogRead(BATTERY_ADC_PIN);
  float v_adc = adc / 4095.0 * 3.3;      // Convert ADC reading to voltage at ADC pin (assuming 3.3V reference)
  float v_batt = v_adc * (R1 + R2) / R2; // Calculate battery voltage using voltage divider formula
  return v_batt;
}

/**
 * @brief Calculates battery level percentage based on voltage.
 * @param voltage Current battery voltage.
 * @return Battery level percentage (0-100).
 */
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
  return 1; // Very low battery
}

/**
 * @brief Prepares and sends GPS data via LoRa and BLE.
 */
void sendGPSData()
{
  unsigned long now = millis() / 1000; // Current time in seconds

  StaticJsonDocument<512> jsonToSend;    // JSON document for LoRa transmission
  StaticJsonDocument<512> jsonToDisplay; // JSON document for local display/BLE

  // Set 'from' ID (DEVICE_ID is now a string) and 'rx_time'
  jsonToSend["from"] = DEVICE_ID;           // Use string DEVICE_ID
  jsonToSend["rx_time"] = 0;                // поміняти на now
  jsonToDisplay["from"] = LOCAL_DISPLAY_ID; // LOCAL_DISPLAY_ID remains integer
  jsonToDisplay["rx_time"] = 0;             // поміняти на now

  // Create nested 'packet' objects
  JsonObject pktSend = jsonToSend.createNestedObject("packet");
  JsonObject pktDisp = jsonToDisplay.createNestedObject("packet");

  // Create nested 'decoded' objects
  JsonObject decodedSend = pktSend.createNestedObject("decoded");
  JsonObject decodedDisp = pktDisp.createNestedObject("decoded");

  // Create nested 'raw' objects
  JsonObject rawSend = pktSend.createNestedObject("raw");
  JsonObject rawDisp = pktDisp.createNestedObject("raw");

  // Check for SOS button press
  if (isSOS())
  {
    decodedSend["portnum"] = "TEXT_MESSAGE_APP";
    decodedSend["payload"] = "SOS";
    decodedSend["want_response"] = false;
    decodedDisp.set(decodedSend); // Copy SOS data to display JSON
  }
  else
  {
    decodedSend["portnum"] = "POSITION_APP";
    decodedDisp["portnum"] = "POSITION_APP";

    JsonObject payloadSend = decodedSend.createNestedObject("payload");
    JsonObject payloadDisp = decodedDisp.createNestedObject("payload");

    // If GPS location is valid, populate payload
    if (gps.location.isValid())
    {
      int32_t lat = (int32_t)(gps.location.lat() * 10000000);
      int32_t lon = (int32_t)(gps.location.lng() * 10000000);
      int alt = gps.altitude.isValid() ? gps.altitude.meters() : 0;

      float vbat = readBatteryVoltage();
      int batt = calculateBatteryLevel(vbat);

      payloadSend["latitude_i"] = lat;
      payloadSend["longitude_i"] = lon;
      payloadSend["altitude"] = alt;
      payloadSend["battery_level"] = batt;
      payloadSend["voltage"] = vbat;
      payloadSend["time"] = now - 4; // Adjust time if needed
      payloadSend["position_flags"] = 3;

      payloadDisp.set(payloadSend); // Copy GPS data to display JSON
    }
  }

  // Add raw signal data (example values)
  rawSend["rx_rssi"] = -76;
  rawSend["rx_snr"] = 9.4;
  rawDisp["rx_rssi"] = -76;
  rawDisp["rx_snr"] = 9.4;

  // Serialize JSON documents to strings
  String toSend, toDisplay;
  serializeJson(jsonToSend, toSend);
  serializeJson(jsonToDisplay, toDisplay);

  // Send LoRa message
  sendLoRaMessage(toSend);
  Serial.println(toDisplay); // Print local display JSON to serial monitor

  // Send BLE notification if enabled and connected
  if (bleEnabled && deviceConnected)
  {
    pCharacteristic->setValue(toDisplay.c_str());
    pCharacteristic->notify();
    Serial.println("BLE notify sent");
  }
}

/**
 * @brief Sends a message via LoRa.
 * @param message The string message to send.
 */
void sendLoRaMessage(String message)
{
  LoRa.beginPacket();
  LoRa.print(message);
  LoRa.endPacket();
}

/**
 * @brief Receives and processes incoming LoRa messages.
 * Checks for duplicates and forwards unique messages.
 */
void receiveLoRaMessage()
{
  if (LoRa.parsePacket())
  {
    String incomingMessage = "";
    while (LoRa.available())
    {
      incomingMessage += (char)LoRa.read();
    }

    // Parse the incoming message as JSON
    StaticJsonDocument<512> receivedJson;
    DeserializationError error = deserializeJson(receivedJson, incomingMessage);

    if (error)
    {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
      return; // Exit if JSON parsing fails
    }

    // Get the "from" field as a string (const char*)
    const char *fromIdStr = receivedJson["from"];

    // Compare with DEVICE_ID (which is now a const char*)
    // Use strcmp for string comparison and check for nullptr
    if (fromIdStr != nullptr && strcmp(fromIdStr, DEVICE_ID) == 0)
    {
      Serial.println("Received message from self, ignoring.");
      return; // Message from self, ignore
    }

    // Check if the message is unique before processing
    if (isMessageUnique(incomingMessage))
    {
      addMessageToHistory(incomingMessage); // Add to history to prevent duplicates
      forwardLoRaMessage(incomingMessage);  // Forward the message via LoRa
      Serial.println(incomingMessage);      // Print to serial monitor

      // Send BLE notification if enabled and connected
      if (bleEnabled && deviceConnected)
      {
        pCharacteristic->setValue(incomingMessage.c_str());
        pCharacteristic->notify();
      }
    }
  }
}

/**
 * @brief Forwards a received LoRa message.
 * @param message The string message to forward.
 */
void forwardLoRaMessage(String message)
{
  LoRa.beginPacket();
  LoRa.print(message);
  LoRa.endPacket();
}

/**
 * @brief Checks if a message is unique in the history.
 * @param message The message to check.
 * @return True if the message is unique, false otherwise.
 */
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

/**
 * @brief Adds a message to the history.
 * @param message The message to add.
 */
void addMessageToHistory(String message)
{
  messageHistory[historyIndex] = calculateHash(message);
  historyIndex = (historyIndex + 1) % MESSAGE_HISTORY_SIZE;
}

// Функція хешування (DJB2 - швидка і проста)
uint32_t calculateHash(String &msg)
{
  uint32_t hash = 5381;
  for (int i = 0; i < msg.length(); i++)
  {
    hash = ((hash << 5) + hash) + msg.charAt(i);
  }
  return hash;
}