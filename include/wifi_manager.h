#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

// ===================== WIFI & MQTT INTEGRATION (TEMPLATE) =====================
// Uncomment and configure when ready to add networking capabilities

/*
// Recommended libraries to add to platformio.ini:
// lib_deps = 
//     knolleary/PubSubClient@^2.8

#include <WiFi.h>
#include <PubSubClient.h>
#include "hvac_types.h"
#include "hvac_config.h"

// ===================== WIFI CONFIGURATION =====================
// TODO: Move to secrets file or use WiFiManager for setup
static const char* WIFI_SSID = "your_ssid_here";
static const char* WIFI_PASSWORD = "your_password_here";

// Connection settings
static constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 10000;
static constexpr uint32_t WIFI_RECONNECT_INTERVAL_MS = 30000;

// ===================== MQTT CONFIGURATION =====================
static const char* MQTT_BROKER = "192.168.1.100";  // or "mqtt.yourdomain.com"
static constexpr uint16_t MQTT_PORT = 1883;
static const char* MQTT_CLIENT_ID = "esp32_hvac_monitor";
static const char* MQTT_USERNAME = "hvac_user";  // Optional
static const char* MQTT_PASSWORD = "hvac_pass";  // Optional

// MQTT Topics
static const char* MQTT_TOPIC_STATE = "homeassistant/sensor/hvac/state";
static const char* MQTT_TOPIC_TEMPS = "homeassistant/sensor/hvac/temperatures";
static const char* MQTT_TOPIC_PRESSURES = "homeassistant/sensor/hvac/pressures";
static const char* MQTT_TOPIC_DIAGNOSTICS = "homeassistant/sensor/hvac/diagnostics";
static const char* MQTT_TOPIC_AVAILABILITY = "homeassistant/sensor/hvac/availability";

// Home Assistant Discovery Topics (auto-configuration)
static const char* MQTT_DISCOVERY_PREFIX = "homeassistant";

// ===================== GLOBALS =====================
static WiFiClient wifiClient;
static PubSubClient mqttClient(wifiClient);
static uint32_t lastWifiCheck = 0;
static uint32_t lastMqttPublish = 0;

// ===================== WIFI FUNCTIONS =====================

inline void connectWifi() {
  if (WiFi.status() == WL_CONNECTED) return;

  Serial.println("Connecting to WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  uint32_t startAttempt = millis();
  while (WiFi.status() != WL_CONNECTED && 
         millis() - startAttempt < WIFI_CONNECT_TIMEOUT_MS) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected!");
    Serial.printf("IP address: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("Signal strength: %d dBm\n", WiFi.RSSI());
  } else {
    Serial.println("\nWiFi connection failed");
  }
}

inline void checkWifiConnection() {
  uint32_t now = millis();
  if (now - lastWifiCheck < WIFI_RECONNECT_INTERVAL_MS) return;
  
  lastWifiCheck = now;
  
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected, reconnecting...");
    connectWifi();
  }
}

// ===================== MQTT FUNCTIONS =====================

inline void connectMqtt() {
  if (mqttClient.connected()) return;
  if (WiFi.status() != WL_CONNECTED) return;

  Serial.println("Connecting to MQTT broker...");
  
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  
  bool connected = false;
  if (strlen(MQTT_USERNAME) > 0) {
    connected = mqttClient.connect(MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD,
                                    MQTT_TOPIC_AVAILABILITY, 0, true, "offline");
  } else {
    connected = mqttClient.connect(MQTT_CLIENT_ID, 
                                    MQTT_TOPIC_AVAILABILITY, 0, true, "offline");
  }

  if (connected) {
    Serial.println("MQTT connected!");
    mqttClient.publish(MQTT_TOPIC_AVAILABILITY, "online", true);
    
    // TODO: Publish Home Assistant discovery messages here
    publishHomeAssistantDiscovery();
  } else {
    Serial.printf("MQTT connection failed, rc=%d\n", mqttClient.state());
  }
}

inline void publishHomeAssistantDiscovery() {
  // Example discovery message for supply air temperature
  const char* discoveryMsg = R"({
    "name": "HVAC Supply Air Temp",
    "device_class": "temperature",
    "state_topic": "homeassistant/sensor/hvac/temperatures",
    "unit_of_measurement": "°F",
    "value_template": "{{ value_json.supply_air }}",
    "unique_id": "hvac_supply_air_temp",
    "device": {
      "identifiers": ["esp32_hvac_001"],
      "name": "HVAC Monitor",
      "model": "ESP32 DevKit",
      "manufacturer": "Custom"
    }
  })";

  mqttClient.publish(
    "homeassistant/sensor/hvac_supply_temp/config", 
    discoveryMsg, 
    true  // retained
  );

  // TODO: Add discovery messages for all other sensors
}

inline void checkMqttConnection() {
  if (!mqttClient.connected()) {
    connectMqtt();
  } else {
    mqttClient.loop();  // Process incoming messages
  }
}

// ===================== DATA PUBLISHING =====================

inline String createTemperaturesJson(const HvacTemperatures& temps) {
  char buffer[256];
  snprintf(buffer, sizeof(buffer),
    R"({"high_pressure":%.1f,"low_pressure":%.1f,"supply_air":%.1f,"return_air":%.1f,"delta_t":%.1f})",
    celsiusToFahrenheit(temps.highPressureLineTempC),
    celsiusToFahrenheit(temps.lowPressureLineTempC),
    celsiusToFahrenheit(temps.supplyAirTempC),
    celsiusToFahrenheit(temps.returnAirTempC),
    deltaCelsiusToDeltaFahrenheit(temps.deltaTempC)
  );
  return String(buffer);
}

inline String createPressuresJson(const HvacPressures& pressures) {
  char buffer[128];
  snprintf(buffer, sizeof(buffer),
    R"({"low_side":%.1f,"high_side":%.1f,"delta":%.1f})",
    pressures.lowSidePressurePsi,
    pressures.highSidePressurePsi,
    pressures.highSidePressurePsi - pressures.lowSidePressurePsi
  );
  return String(buffer);
}

inline String createStateJson(const HvacState& state) {
  char buffer[256];
  snprintf(buffer, sizeof(buffer),
    R"({"system":"%s","mode":"%s","diagnostic":"%s"})",
    (state.systemState == HvacSystemState::Running) ? "RUNNING" : "OFF",
    hvacModeToString(state.mode),
    diagToString(state.diagnostic)
  );
  return String(buffer);
}

inline String createDiagnosticsJson(const FaultReport& faults) {
  String json = "{\"fault_count\":" + String(faults.count) + ",\"faults\":[";
  
  for (uint8_t i = 0; i < faults.count; i++) {
    if (i > 0) json += ",";
    json += "\"" + String(faultCodeToString(faults.codes[i])) + "\"";
  }
  
  json += "]}";
  return json;
}

inline void publishSensorData(
  const HvacTemperatures& temps,
  const HvacPressures& pressures,
  const HvacState& state,
  const FaultReport& faults
) {
  uint32_t now = millis();
  if (now - lastMqttPublish < NETWORK_SEND_INTERVAL_MS) return;
  
  if (!mqttClient.connected()) return;

  lastMqttPublish = now;

  // Publish each data type to its own topic
  mqttClient.publish(MQTT_TOPIC_TEMPS, createTemperaturesJson(temps).c_str());
  mqttClient.publish(MQTT_TOPIC_PRESSURES, createPressuresJson(pressures).c_str());
  mqttClient.publish(MQTT_TOPIC_STATE, createStateJson(state).c_str());
  mqttClient.publish(MQTT_TOPIC_DIAGNOSTICS, createDiagnosticsJson(faults).c_str());

  Serial.println("Published sensor data to MQTT");
}

// ===================== INITIALIZATION =====================

inline void initWifiAndMqtt() {
  connectWifi();
  
  if (WiFi.status() == WL_CONNECTED) {
    connectMqtt();
  }
}

// ===================== MAIN LOOP INTEGRATION =====================

inline void updateNetworking(
  const HvacTemperatures& temps,
  const HvacPressures& pressures,
  const HvacState& state,
  const FaultReport& faults
) {
  checkWifiConnection();
  checkMqttConnection();
  publishSensorData(temps, pressures, state, faults);
}

*/

#endif // WIFI_MANAGER_H
