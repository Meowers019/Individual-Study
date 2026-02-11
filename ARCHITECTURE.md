# ESP32 HVAC Monitoring System - Architecture

## Project Overview
Modular ESP32-based HVAC monitoring system with comprehensive diagnostics for heat pumps and air conditioning units. Designed for integration with Home Assistant via WiFi networking.

## System Architecture

### Hardware
- **ESP32 DevKit** - Main controller
- **MAX31855 (x4)** - K-type thermocouple interface (SPI)
  - High pressure line (liquid line)
  - Low pressure line (suction line)
  - Supply air duct
  - Return air duct
- **ADS1115** - 16-bit ADC for pressure sensors (I2C)
  - Channel A0: TDH33 (0-1000 PSI, 0-5V)
  - Channel A1: TD1000 (0-600 PSI, 1-5V)
- **Voltage Divider** - 10kΩ/20kΩ for sensor voltage scaling

### Software Architecture (Modular Design)

#### Core Headers (include/)

##### `hvac_types.h`
- Core data structures and enums
- `HvacMode`, `DiagnosticState`, `HvacSystemState`, `FaultCode`
- Sensor reading structures: `Max31855Reading`, `HvacTemperatures`, `HvacPressures`
- Fault reporting: `FaultReport` with multi-fault tracking
- String conversion helpers for debugging

##### `hvac_config.h`
- Hardware pin definitions (I2C, SPI, chip selects)
- Sensor calibration constants
- Voltage divider specifications
- Threshold values for diagnostics:
  - Mode detection (cooling/heating/off)
  - Running state detection
  - Fault detection limits
- Timing constants (sensor read intervals, future network intervals)

##### `max31855_driver.h`
- MAX31855 thermocouple amplifier driver
- SPI communication with proper timing
- Raw frame reading and decoding
- Cold junction compensation
- Multi-sample filtering to reduce noise
- Temperature conversion (°C ↔ °F)
- Fault detection: open circuit, short to GND/VCC

##### `ads1115_driver.h`
- ADS1115 16-bit ADC driver
- I2C communication at 400 kHz
- Single-ended channel reading (0-3)
- ±4.096V PGA range configuration
- Voltage-to-pressure conversion with calibration
- Sensor-specific readers for A0/A1
- Deadband filtering for noise suppression

##### `r22_tables.h`
- R-22 refrigerant pressure/temperature lookup table
- PSI → Saturation temperature conversion
- Superheat calculation (suction temp - sat temp)
- Subcooling calculation (sat temp - liquid temp)
- Extensible for other refrigerants (R-410A, R-134a, etc.)

##### `hvac_sensors.h`
- High-level sensor management
- Orchestrates readings from all sensors
- HVAC mode detection (cooling/heating/off)
- Running state detection (deltaT and deltaPsi methods)
- Sensor data updates with timestamps
- State machine logic with confirmation counts

##### `hvac_diagnostics.h`
- Comprehensive fault detection system
- 16+ fault codes covering:
  - Sensor validity (missing data)
  - Pressure issues (too high/low, not equalizing when off)
  - Temperature issues (freezing suction, overheated liquid)
  - Delta-T problems (too low/high/zero)
  - Superheat/subcooling out of range
  - Pattern recognition (low charge, restriction)
- Severity classification (Normal/WeakPerformance/SensorFault)
- Multi-fault reporting with deduplification

##### `main.cpp`
- Simplified main program (~214 lines vs. original 1228 lines!)
- Global state management
- `setup()`: Initialize I2C, SPI, sensors
- `loop()`: 
  - Read all sensors
  - Update state machines
  - Calculate derived values
  - Evaluate diagnostics
  - Print comprehensive status report
- Currently outputs to Serial, prepared for network integration

### File Structure
```
ESP_Server/
├── platformio.ini          # PlatformIO configuration
├── include/                # Header files
│   ├── hvac_types.h       # Data structures & enums
│   ├── hvac_config.h      # Configuration & constants
│   ├── max31855_driver.h  # Thermocouple driver
│   ├── ads1115_driver.h   # ADC driver
│   ├── r22_tables.h       # Refrigerant P/T tables
│   ├── hvac_sensors.h     # Sensor management
│   └── hvac_diagnostics.h # Fault detection
├── src/
│   ├── main.cpp           # Main program (NEW - modular)
│   └── main_old.cpp       # Original monolithic version (backup)
└── lib/                   # External libraries (none required currently)
```

## Monitoring Capabilities

### Real-Time Measurements
- **Temperatures** (all in °F for display, stored as °C)
  - High/low pressure line temps
  - Supply/return air temps
  - Delta-T (return - supply)
- **Pressures** (PSI)
  - Low side (suction) pressure
  - High side (discharge) pressure
  - Delta-P (high - low)
- **Derived Values**
  - Saturation temperatures (from P/T chart)
  - Superheat (suction line)
  - Subcooling (liquid line)

### HVAC State Detection
- **System State**: OFF / RUNNING
  - Detected via delta-T OR delta-PSI thresholds
- **Operating Mode**: OFF / COOLING / HEATING
  - Uses temperature differentials and duct temps
  - Confirmation count prevents rapid mode switching
- **Diagnostic State**: NORMAL / WEAK_PERFORMANCE / SENSOR_FAULT

### Fault Detection
- Missing/invalid sensor data
- Static pressure not equalizing when off (restriction indicator)
- High/low side pressures out of range
- Suction line freezing
- Liquid line overheating
- Delta-T too low, too high, or near zero
- Superheat too low/high
- Subcooling too low/high
- Low refrigerant charge pattern (low subcool + high superheat)
- Restriction pattern (high subcool + high superheat)

## Next Steps: WiFi & Home Assistant Integration

### Option 1: MQTT (Recommended for flexibility)
```cpp
// Future additions to platformio.ini
lib_deps = 
    knolleary/PubSubClient@^2.8

// New file: include/mqtt_client.h
- Connect to WiFi (with Tailscale VPN support)
- Connect to MQTT broker
- Publish sensor data as JSON
- Subscribe to commands (future: remote control)
```

### Option 2: ESPHome Native
- Migrate to ESPHome framework
- Define custom components for HVAC logic
- Automatic Home Assistant discovery
- OTA updates via ESPHome dashboard

### Option 3: HTTP REST API
- Simple HTTP POST to Home Assistant webhook
- Less efficient but easiest to implement
- No MQTT broker required

### Recommended Data Flow
```
ESP32 → WiFi → Tailscale VPN → MQTT Broker → Home Assistant → TrueNAS
         (encrypted)            (Mosquitto)   (sensor entities)  (database)
```

### Home Assistant Integration Points
1. **Sensor Entities**: Temperatures, pressures, SH/SC
2. **Binary Sensors**: System running, mode states, faults
3. **Diagnostic Sensors**: Fault codes, diagnostic state
4. **Attributes**: Timestamps, raw readings, calibration info
5. **Automations**: Alerts on faults, performance tracking
6. **History**: Long-term trend analysis stored in TrueNAS

## Configuration & Calibration

### Temperature Offsets
Adjust in `hvac_config.h`:
```cpp
static constexpr float SUPPLY_TEMP_OFFSET_C = 1.0f * 5.0f / 9.0f;  // 1°F
static constexpr float RETURN_TEMP_OFFSET_C = 1.0f * 5.0f / 9.0f;  // 1°F
```

### Pressure Sensor Zero Calibration
Update after measuring with system off (0 PSI):
```cpp
static constexpr float A0_ZERO_CAL_V = 0.151f;  // Update from live reading
static constexpr float A1_ZERO_CAL_V = 1.025f;  // Update from live reading
```

### Fault Thresholds
All thresholds in `hvac_config.h` - adjust based on your specific system:
- Pressure limits (cooling: 40-500 PSI typical)
- Delta-T limits (10-25°F typical cooling)
- Superheat (5-25°F typical)
- Subcooling (5-20°F typical)

## Building & Uploading

### PlatformIO
```bash
# Build
pio run

# Upload to ESP32
pio run --target upload

# Monitor serial output
pio device monitor
```

### VS Code with PlatformIO Extension
- Build: Ctrl+Alt+B
- Upload: Ctrl+Alt+U
- Serial Monitor: Ctrl+Alt+S

## Serial Output Format
Every 2 seconds (configurable), outputs:
```
=== BEGIN MONITORING ===
SYS=RUNNING  MODE=COOLING  DIAG=NORMAL

---- MAX31855 Raw/Decoded ----
[Raw frame data with CJ and TC temps]

---- HVAC Temps (Thermocouple TC) ----
[Line and duct temperatures]

---- Pressures ----
[Low/high side PSI, delta-PSI]

---- Saturation Temps (R-22) ----
[Calculated sat temps from pressure]

---- Superheat / Subcooling ----
[SH and SC values]

FAULTS: NONE
(or list of active faults if any)

=== END MONITORING ===
```

## Code Review Notes (From Original Analysis)

### Improvements Made
✅ Separated into logical modules
✅ Removed commented-out code
✅ Consistent naming conventions
✅ Proper header guards
✅ Inline functions for performance

### Still Recommended
- [ ] Add watchdog timer for reliability
- [ ] I2C retry logic on ADS1115 failures
- [ ] Move R-22 table to PROGMEM to save RAM
- [ ] WiFi connectivity with reconnection logic
- [ ] Data logging to SD card or network
- [ ] OTA updates for remote deployment
- [ ] JSON API for structured data export

## License & Attribution
Developed for HVAC monitoring and diagnostics.
Modular architecture designed for extensibility.
Ready for community contributions and feature additions.

## Contact & Support
For issues, feature requests, or questions about integration with Home Assistant, 
open an issue in the project repository.
