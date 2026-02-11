# WiFi & MQTT Integration Guide

## Quick Start to Add Networking

Your code is now ready for WiFi and MQTT integration. Follow these steps when you're ready to connect to Home Assistant.

---

## Step 1: Update platformio.ini

Add the MQTT library dependency:

```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
lib_deps = 
    knolleary/PubSubClient@^2.8
```

---

## Step 2: Uncomment WiFi Manager Code

In `include/wifi_manager.h`, uncomment all the code (remove the `/*` and `*/` at the top and bottom).

Then configure your settings:

```cpp
// WiFi credentials
static const char* WIFI_SSID = "YourNetwork";
static const char* WIFI_PASSWORD = "YourPassword";

// MQTT broker (adjust IP or hostname)
static const char* MQTT_BROKER = "192.168.1.100";  // Your Home Assistant IP
static constexpr uint16_t MQTT_PORT = 1883;
static const char* MQTT_CLIENT_ID = "esp32_hvac_monitor";

// If your MQTT broker requires authentication:
static const char* MQTT_USERNAME = "your_mqtt_user";
static const char* MQTT_PASSWORD = "your_mqtt_pass";
```

---

## Step 3: Update main.cpp

Add the WiFi header at the top with the other includes:

```cpp
#include "hvac_diagnostics.h"
#include "wifi_manager.h"  // <-- ADD THIS LINE
```

In `setup()`, add WiFi initialization:

```cpp
void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println("\n=== HVAC ESP32 Monitoring System ===");
  Serial.println("Modular architecture - Ready for WiFi integration");

  initADS1115();
  initMax31855Bus();
  
  initWifiAndMqtt();  // <-- ADD THIS LINE

  Serial.println("Initialization complete.\n");
}
```

In `loop()`, add networking updates before the delay:

```cpp
  printFaultReportLines(faultReport);
  Serial.println();
  Serial.println("=== END MONITORING ===");
  Serial.println();

  // Publish data to MQTT
  updateNetworking(hvacTemps, hvacPressures, hvacState, faultReport);  // <-- ADD THIS

  delay(SENSOR_READ_INTERVAL_MS);
}
```

---

## Step 4: Configure Home Assistant

### Option A: Using MQTT Auto-Discovery

The code includes Home Assistant MQTT discovery. Your sensors will automatically appear in Home Assistant once connected.

### Option B: Manual Configuration

Add to your Home Assistant `configuration.yaml`:

```yaml
mqtt:
  sensor:
    # Temperature Sensors
    - name: "HVAC Supply Air Temp"
      state_topic: "homeassistant/sensor/hvac/temperatures"
      unit_of_measurement: "°F"
      value_template: "{{ value_json.supply_air }}"
      device_class: temperature
      
    - name: "HVAC Return Air Temp"
      state_topic: "homeassistant/sensor/hvac/temperatures"
      unit_of_measurement: "°F"
      value_template: "{{ value_json.return_air }}"
      device_class: temperature
      
    - name: "HVAC Delta-T"
      state_topic: "homeassistant/sensor/hvac/temperatures"
      unit_of_measurement: "°F"
      value_template: "{{ value_json.delta_t }}"
      
    # Pressure Sensors
    - name: "HVAC Low Side Pressure"
      state_topic: "homeassistant/sensor/hvac/pressures"
      unit_of_measurement: "PSI"
      value_template: "{{ value_json.low_side }}"
      device_class: pressure
      
    - name: "HVAC High Side Pressure"
      state_topic: "homeassistant/sensor/hvac/pressures"
      unit_of_measurement: "PSI"
      value_template: "{{ value_json.high_side }}"
      device_class: pressure
      
    # System State
    - name: "HVAC System State"
      state_topic: "homeassistant/sensor/hvac/state"
      value_template: "{{ value_json.system }}"
      
    - name: "HVAC Operating Mode"
      state_topic: "homeassistant/sensor/hvac/state"
      value_template: "{{ value_json.mode }}"
      
    - name: "HVAC Diagnostic State"
      state_topic: "homeassistant/sensor/hvac/state"
      value_template: "{{ value_json.diagnostic }}"
      
    # Diagnostics
    - name: "HVAC Fault Count"
      state_topic: "homeassistant/sensor/hvac/diagnostics"
      value_template: "{{ value_json.fault_count }}"
      
  binary_sensor:
    - name: "HVAC Monitor Online"
      state_topic: "homeassistant/sensor/hvac/availability"
      payload_on: "online"
      payload_off: "offline"
      device_class: connectivity
```

---

## Step 5: Tailscale Integration (Optional)

If you want secure remote access via Tailscale:

1. **Install Tailscale on your network**:
   - On the machine running your MQTT broker
   - On any device you want to access Home Assistant from

2. **Update MQTT_BROKER** to use Tailscale IP:
   ```cpp
   static const char* MQTT_BROKER = "100.x.x.x";  // Your Tailscale IP
   ```

3. **ESP32 connects via local WiFi**, but MQTT broker is on Tailscale network:
   - The ESP32 doesn't need Tailscale client
   - It connects to the broker's Tailscale IP on your local network
   - All traffic between broker and Home Assistant is encrypted via Tailscale

---

## Step 6: Data Flow to TrueNAS

### Using InfluxDB on TrueNAS

1. **Install InfluxDB on TrueNAS** (via TrueNAS SCALE apps or manual Docker)

2. **Configure Home Assistant** to send data to InfluxDB:
   ```yaml
   influxdb:
     host: truenas.local  # or Tailscale IP
     port: 8086
     database: homeassistant
     username: !secret influxdb_user
     password: !secret influxdb_pass
     include:
       entities:
         - sensor.hvac_supply_air_temp
         - sensor.hvac_return_air_temp
         - sensor.hvac_low_side_pressure
         - sensor.hvac_high_side_pressure
         # ... add all HVAC sensors
   ```

3. **Use Grafana** (on TrueNAS) for advanced visualization

---

## Step 7: Testing

1. **Upload to ESP32**:
   ```bash
   pio run --target upload
   pio device monitor
   ```

2. **Watch Serial Monitor** for:
   - WiFi connection status
   - MQTT connection status
   - "Published sensor data to MQTT" messages every 15 seconds

3. **Check Home Assistant**:
   - Developer Tools → MQTT
   - Listen to topic: `homeassistant/sensor/hvac/#`
   - Verify data is arriving

4. **Verify Entities**:
   - Settings → Devices & Services → MQTT
   - Should see HVAC sensors auto-discovered

---

## Troubleshooting

### WiFi Won't Connect
- Check SSID and password
- Verify ESP32 is in range
- Try 2.4 GHz network (ESP32 doesn't support 5 GHz)

### MQTT Won't Connect
- Check broker IP address
- Verify port 1883 is open
- Test with `mosquitto_pub -h BROKER_IP -t test/topic -m "hello"`
- Check username/password if authentication is enabled

### Data Not Appearing in Home Assistant
- Check MQTT integration is configured
- Verify topics match in both ESP32 and HA config
- Use MQTT Explorer to inspect raw messages
- Check Home Assistant logs for errors

### High Network Traffic
- Increase `NETWORK_SEND_INTERVAL_MS` in `hvac_config.h`
- Consider publishing only on significant changes
- Reduce JSON message size by removing unnecessary fields

---

## Advanced: ESPHome Alternative

If you prefer ESPHome over Arduino framework:

1. **Create ESPHome YAML** with custom components
2. **Import your HVAC logic** as custom C++ components
3. **Benefits**:
   - Automatic OTA updates
   - Built-in Home Assistant integration
   - Web interface for configuration
   - No need to manually configure MQTT

Would you like help converting to ESPHome? Let me know!

---

## Next Steps

Now that networking is set up:
- [ ] Add alerting for critical faults
- [ ] Create Home Assistant dashboard
- [ ] Set up historical data retention on TrueNAS
- [ ] Add remote control capabilities (future)
- [ ] Implement OTA updates
- [ ] Add web interface on ESP32 (optional)

---

## Security Recommendations

1. **Use Tailscale** for encrypted remote access
2. **Enable MQTT authentication** (username/password)
3. **Use TLS/SSL** for MQTT (port 8883) if exposing to internet
4. **Keep ESP32 firmware updated**
5. **Isolate IoT devices** on separate VLAN if possible
6. **Use secrets file** instead of hardcoded credentials

---

Questions? Issues? Come back and I'll help you integrate!
