# HVAC Test Mode - Usage Guide

## Overview
The test mode allows you to simulate sensor readings without hardware, perfect for:
- Developing and testing diagnostic logic
- Testing WiFi/MQTT integration
- Validating fault detection algorithms
- Creating demonstrations
- Unit testing your code changes

## Quick Start

### Enable Test Mode
In `main.cpp`, uncomment ONE line in the `setup()` function:

```cpp
void setup() {
  // ... other code ...
  
  // Enable test mode with a specific scenario:
  enableTestMode(TestScenario::Normal_Cooling);
  
  // ... rest of setup ...
}
```

### Available Test Scenarios

| Scenario | Description | Expected Faults |
|----------|-------------|----------------|
| `Normal_Cooling` | Healthy system cooling | None |
| `Normal_Heating` | Healthy system heating | None |
| `System_Off` | Unit completely off | None |
| `Low_Refrigerant_Charge` | Classic low charge symptoms | Low subcool, high superheat, low charge pattern |
| `Dirty_Condenser` | High head pressure | High head pressure fault |
| `Dirty_Evaporator` | Low suction, low superheat | Low suction pressure, low superheat |
| `Restriction_TXV` | TXV restriction pattern | High subcool, high superheat, restriction pattern |
| `Suction_Line_Freezing` | Evaporator freezing | Suction line freezing, low suction |
| `Static_Not_Equalizing` | System off, restriction | Static not equalizing fault |
| `Weak_Airflow` | Poor airflow, low delta-T | Delta-T too low |
| `Multiple_Faults` | Several issues at once | Multiple fault codes |
| `All_Sensors_Fault` | Hardware failure simulation | Missing temp, missing pressure |
| `Custom` | Your own test values | Depends on custom values |

## Usage Examples

### Example 1: Test Normal Cooling
```cpp
void setup() {
  Serial.begin(115200);
  delay(200);

  enableTestMode(TestScenario::Normal_Cooling);
  
  // Hardware init skipped automatically in test mode
  if (!TEST_MODE_ENABLED) {
    initADS1115();
    initMax31855Bus();
  }
  
  Serial.println("Initialization complete.\n");
}
```

**Expected Output:**
```
*** TEST MODE ENABLED ***
Scenario: Normal Cooling - Healthy system
***************************

SYS=RUNNING  MODE=COOLING  DIAG=NORMAL
FAULTS: NONE
```

### Example 2: Test Low Refrigerant Pattern
```cpp
enableTestMode(TestScenario::Low_Refrigerant_Charge);
```

**Expected Output:**
```
SYS=RUNNING  MODE=COOLING  DIAG=WEAK_PERFORMANCE
FAULTS (3):
 - FAULT_SUBCOOL_LOW
 - FAULT_SUPERHEAT_HIGH
 - FAULT_LOW_CHARGE_PATTERN
```

### Example 3: Test All Sensor Failure
```cpp
enableTestMode(TestScenario::All_Sensors_Fault);
```

**Expected Output:**
```
SYS=OFF  MODE=OFF  DIAG=SENSOR_FAULT
FAULTS (4):
 - FAULT_MISSING_TEMP
 - FAULT_MISSING_PRESSURE
 - FAULT_MISSING_SATTEMP
 - FAULT_MISSING_SHSC
```

### Example 4: List All Scenarios
```cpp
void setup() {
  Serial.begin(115200);
  delay(200);
  
  printTestScenarioList();  // Show all available scenarios
  enableTestMode(TestScenario::Normal_Cooling);
}
```

**Output:**
```
=== Available Test Scenarios ===
 0: Normal Cooling - Healthy system
 1: Normal Heating - Healthy system
 2: System Off - Pressures equalized
 3: Low Refrigerant Charge - Low subcool, high superheat
 4: Dirty Condenser - High head pressure
 ... etc
================================
```

## Advanced Usage

### Switching Scenarios at Runtime
```cpp
void loop() {
  // Run diagnostics...
  
  // After 10 seconds, switch to a different scenario
  if (millis() > 10000) {
    setTestScenario(TestScenario::Dirty_Condenser);
  }
}
```

### Cycling Through All Scenarios
```cpp
void loop() {
  static uint32_t lastCycle = 0;
  
  // Cycle scenarios every 5 seconds
  if (millis() - lastCycle > 5000) {
    cycleTestScenarios();
    lastCycle = millis();
  }
  
  // ... rest of loop ...
}
```

### Print Current Test Data
```cpp
void setup() {
  enableTestMode(TestScenario::Low_Refrigerant_Charge);
  printCurrentTestData();  // Show detailed test values
}
```

**Output:**
```
=== Current Test Data ===
Scenario: Low Refrigerant Charge - Low subcool, high superheat

Temperatures:
  High Pressure Line: 32.2°C (90.0°F)
  Low Pressure Line:  7.2°C (45.0°F)
  Supply Air:         15.6°C (60.1°F)
  Return Air:         23.9°C (75.0°F)

Pressures:
  Low Side:  50.0 PSI
  High Side: 200.0 PSI
  Delta:     150.0 PSI
=========================
```

### Disable Test Mode
```cpp
void setup() {
  // Start in test mode
  enableTestMode(TestScenario::Normal_Cooling);
  
  // After testing, switch to live mode
  delay(10000);  // 10 seconds of testing
  disableTestMode();
  
  // Now read real sensors
  initADS1115();
  initMax31855Bus();
}
```

## Creating Custom Test Scenarios

### Option 1: Edit the Custom Scenario
In `hvac_test_data.h`, find the `Custom` scenario at the end of `TEST_SCENARIOS[]`:

```cpp
// Custom: Set your own values below
{
  .highPressureLineTempC = 35.0f,  // Change these!
  .lowPressureLineTempC = 8.0f,
  .supplyAirTempC = 13.0f,
  .returnAirTempC = 24.0f,
  .lowSidePressurePsi = 65.0f,
  .highSidePressurePsi = 280.0f,
  .description = "Custom Test Values"
}
```

Then use:
```cpp
enableTestMode(TestScenario::Custom);
```

### Option 2: Add New Scenarios
Add your own scenarios to the `TEST_SCENARIOS[]` array and update the `TestScenario` enum:

```cpp
// In hvac_test_data.h
enum class TestScenario : uint8_t {
  // ... existing scenarios ...
  My_New_Scenario,  // Add your enum value
  Custom
};

// In TEST_SCENARIOS array
{
  .highPressureLineTempC = 40.0f,
  .lowPressureLineTempC = 5.0f,
  .supplyAirTempC = 10.0f,
  .returnAirTempC = 25.0f,
  .lowSidePressurePsi = 45.0f,
  .highSidePressurePsi = 350.0f,
  .description = "My Custom Scenario Description"
},
```

## Testing WiFi Integration

Test mode is perfect for developing WiFi/MQTT features:

```cpp
void loop() {
  // Get test data
  if (TEST_MODE_ENABLED) {
    applyTestDataToHvacTemps(hvacTemps);
    applyTestDataToHvacPressures(hvacPressures);
  }
  
  // Update diagnostics (works with test data!)
  updateHvacRunStateFromDeltaTOrDeltaPsi(hvacTemps, hvacPressures, hvacState);
  updateHvacModeFromTemperatures(hvacTemps, hvacState);
  
  // Test your MQTT publishing
  if (WiFi.status() == WL_CONNECTED) {
    publishSensorData(hvacTemps, hvacPressures, hvacState, faultReport);
  }
  
  delay(5000);
}
```

## Tips & Best Practices

1. **Use test mode for WiFi development** - Don't need hardware connected
2. **Test all fault scenarios** - Ensure your thresholds are correct
3. **Verify mode detection** - Test cooling/heating/off transitions
4. **Create realistic scenarios** - Use actual field measurements
5. **Document your tests** - Note which faults you expect
6. **Cycle scenarios automatically** - Test continuous monitoring
7. **Print test data** - Verify what values are being used

## Troubleshooting

### Test mode not working?
- Check `TEST_MODE_ENABLED` is `true`
- Verify you called `enableTestMode()` in `setup()`
- Look for "TEST MODE ENABLED" message on serial

### Wrong fault codes?
- Use `printCurrentTestData()` to see actual values
- Check threshold values in `hvac_config.h`
- Verify your scenario data is correct

### Still reading hardware?
- Ensure hardware init is skipped: `if (!TEST_MODE_ENABLED)`
- Test mode must be enabled BEFORE hardware init

## Next Steps

Once you've tested your diagnostic logic:
1. Test WiFi connectivity with simulated data
2. Verify MQTT publishing works
3. Test Home Assistant integration
4. Switch to live sensors: `disableTestMode()`
5. Compare live vs. test behavior

Happy testing! 🧪
