#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// Pin definitions
const int RELAY_PIN = 14;  // Relay connected to pin 14 for irrigation control
const int LED_PIN = 12;    // LED connected to pin 12 for status indication

// WiFi credentials
const char* WIFI_SSID = "Wokwi-GUEST";
const char* WIFI_PASSWORD = "";

// MQTT configuration
const char* MQTT_SERVER = "test.mosquitto.org";
const int MQTT_PORT = 1883;
const char* MQTT_CLIENT_ID = "agro-papin-001";

// MQTT Topics
const char* TOPIC_STATUS = "agropapin/devices/agro-papin-001/status";
const char* TOPIC_COMMANDS = "agropapin/devices/agro-papin-001/commands";
const char* TOPIC_TELEMETRY = "agropapin/devices/agro-papin-001/telemetry";

// Device configuration
const char* DEVICE_ID = "agro-papin-001";

// State variables
bool relayState = false;
bool ledState = false;
unsigned long lastStatusUpdate = 0;
const unsigned long STATUS_INTERVAL = 10000; // Send status every 10 seconds

// MQTT and WiFi clients
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

/// @brief Arduino setup function - called once at startup
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== AgroPapin Irrigation Controller Starting ===");
  
  // Initialize pins
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  
  // Set initial states to OFF
  digitalWrite(RELAY_PIN, LOW);
  digitalWrite(LED_PIN, LOW);
  relayState = false;
  ledState = false;
  Serial.println("Actuators initialized (OFF state)");
  
  // Connect to WiFi
  connectWiFi();
  
  // Configure MQTT
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setCallback(onMqttMessage);
  
  // Connect to MQTT broker
  connectMQTT();
  
  Serial.println("=== Setup Complete ===\n");
}

/// @brief Arduino main loop function
void loop() {
  // Ensure MQTT connection is maintained
  if (!mqttClient.connected()) {
    connectMQTT();
  }
  
  // Process MQTT messages
  mqttClient.loop();
  
  // Send periodic status updates
  unsigned long currentTime = millis();
  if (currentTime - lastStatusUpdate >= STATUS_INTERVAL) {
    sendStatusUpdate();
    lastStatusUpdate = currentTime;
  }
  
  delay(100);
}

/// @brief Establishes WiFi connection
void connectWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("\nWiFi connected successfully!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Signal strength (RSSI): ");
  Serial.print(WiFi.RSSI());
  Serial.println(" dBm");
}

/// @brief Connects to MQTT broker
void connectMQTT() {
  Serial.print("Connecting to MQTT broker...");
  
  while (!mqttClient.connected()) {
    if (mqttClient.connect(MQTT_CLIENT_ID)) {
      Serial.println(" Connected!");
      
      // Subscribe to command topic
      mqttClient.subscribe(TOPIC_COMMANDS);
      Serial.printf("Subscribed to: %s\n", TOPIC_COMMANDS);
      
      // Send initial connection message
      sendConnectionStatus(true);
      
    } else {
      Serial.print("Failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" Retrying in 5 seconds...");
      delay(5000);
    }
  }
}

/// @brief Handles incoming MQTT messages
/// @param topic The topic that received the message
/// @param payload The message payload
/// @param length The length of the payload
void onMqttMessage(char* topic, byte* payload, unsigned int length) {
  // Convert payload to string
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  Serial.printf("MQTT Message received on topic: %s\n", topic);
  Serial.printf("Message: %s\n", message.c_str());
  
  // Process command messages
  if (String(topic) == TOPIC_COMMANDS) {
    processCommand(message);
  }
}

/// @brief Processes incoming commands
/// @param command JSON command string
void processCommand(const String& command) {
  DynamicJsonDocument doc(512);
  DeserializationError error = deserializeJson(doc, command);
  
  if (error) {
    Serial.print("Failed to parse command JSON: ");
    Serial.println(error.c_str());
    return;
  }
  
  // Extract command data
  String action = doc["action"];
  float soilMoisture = doc["soil_moisture"] | 0.0;
  
  Serial.printf("Command - Action: %s | Soil Moisture: %.1f%%\n", action.c_str(), soilMoisture);
  
  // Control actuators based on command
  bool shouldActivate = (action == "irrigate");
  controlRelay(shouldActivate);
  controlLed(shouldActivate);
  
  Serial.println(shouldActivate ? "IRRIGATION ACTIVATED" : "IRRIGATION DEACTIVATED");
  
  // Send telemetry update
  sendTelemetryUpdate(soilMoisture);
}

/// @brief Controls the relay actuator
/// @param state True to activate relay (turn ON), false to deactivate (turn OFF)
void controlRelay(bool state) {
  digitalWrite(RELAY_PIN, state ? HIGH : LOW);
  relayState = state;
  Serial.printf("Relay %s\n", state ? "ON" : "OFF");
}

/// @brief Controls the LED actuator
/// @param state True to turn LED ON, false to turn LED OFF
void controlLed(bool state) {
  digitalWrite(LED_PIN, state ? HIGH : LOW);
  ledState = state;
  Serial.printf("LED %s\n", state ? "ON" : "OFF");
}

/// @brief Returns current relay state
/// @return True if relay is ON, false if OFF
bool getRelayState() {
  return relayState;
}

/// @brief Returns current LED state
/// @return True if LED is ON, false if OFF
bool getLedState() {
  return ledState;
}

/// @brief Sends periodic status update to MQTT
void sendStatusUpdate() {
  DynamicJsonDocument doc(256);
  doc["device_id"] = DEVICE_ID;
  doc["timestamp"] = millis();
  doc["relay_state"] = relayState;
  doc["led_state"] = ledState;
  doc["wifi_rssi"] = WiFi.RSSI();
  doc["uptime"] = millis() / 1000;
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  if (mqttClient.publish(TOPIC_STATUS, jsonString.c_str())) {
    Serial.println("📡 Status update sent via MQTT");
    Serial.println(jsonString);
  } else {
    Serial.println("❌ Failed to send status update");
  }
}

/// @brief Sends telemetry data to MQTT
/// @param soilMoisture Current soil moisture reading
void sendTelemetryUpdate(float soilMoisture) {
  DynamicJsonDocument doc(256);
  doc["device_id"] = DEVICE_ID;
  doc["timestamp"] = millis();
  doc["soil_moisture"] = soilMoisture;
  doc["relay_state"] = relayState;
  doc["led_state"] = ledState;
  doc["irrigation_active"] = relayState;
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  if (mqttClient.publish(TOPIC_TELEMETRY, jsonString.c_str())) {
    Serial.println("📊 Telemetry data sent via MQTT");
    Serial.println(jsonString);
  } else {
    Serial.println("❌ Failed to send telemetry data");
  }
}

/// @brief Sends connection status to MQTT
/// @param connected Connection status (true = connected, false = disconnected)
void sendConnectionStatus(bool connected) {
  DynamicJsonDocument doc(256);
  doc["device_id"] = DEVICE_ID;
  doc["timestamp"] = millis();
  doc["status"] = connected ? "online" : "offline";
  doc["ip_address"] = WiFi.localIP().toString();
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  if (mqttClient.publish(TOPIC_STATUS, jsonString.c_str(), true)) { // Retained message
    Serial.printf("Connection status sent: %s\n", connected ? "ONLINE" : "OFFLINE");
  } else {
    Serial.println("Failed to send connection status");
  }
}