#ifndef HVAC_TEST_DATA_H
#define HVAC_TEST_DATA_H

#include "hvac_types.h"
#include "hvac_config.h"

// ===================== TEST MODE CONTROL =====================
// Set to true to use simulated data instead of reading from sensors
static bool TEST_MODE_ENABLED = false;

// ===================== TEST SCENARIOS =====================
enum class TestScenario : uint8_t {
  Normal_Cooling,           // System running normally in cooling mode
  Normal_Heating,           // System running normally in heating mode
  System_Off,               // System completely off
  Low_Refrigerant_Charge,   // Low charge pattern (low subcool + high superheat)
  Dirty_Condenser,          // High head pressure
  Dirty_Evaporator,         // Low suction pressure, low superheat
  Restriction_TXV,          // Restriction pattern (high subcool + high superheat)
  Suction_Line_Freezing,    // Evaporator freezing up
  Static_Not_Equalizing,    // System off but pressures not equalizing
  Weak_Airflow,             // Low delta-T
  Multiple_Faults,          // Several issues at once
  All_Sensors_Fault,        // All sensors reading NAN (hardware failure)
  
  // Add more scenarios as needed
  Custom                    // Use custom values set below
};

// Select which scenario to test
static TestScenario ACTIVE_TEST_SCENARIO = TestScenario::Normal_Cooling;

// ===================== TEST DATA STRUCTURES =====================
struct TestSensorData {
  // Temperatures in Celsius (will be used directly)
  float highPressureLineTempC;
  float lowPressureLineTempC;
  float supplyAirTempC;
  float returnAirTempC;
  
  // Pressures in PSI
  float lowSidePressurePsi;
  float highSidePressurePsi;
  
  // Description for logging
  const char* description;
};

// ===================== PREDEFINED TEST SCENARIOS =====================

static const TestSensorData TEST_SCENARIOS[] = {
  // Normal_Cooling: Healthy system in cooling mode
  {
    .highPressureLineTempC = 37.8f,   // 100°F liquid line
    .lowPressureLineTempC = 10.0f,    // 50°F suction line
    .supplyAirTempC = 12.8f,          // 55°F supply air
    .returnAirTempC = 23.9f,          // 75°F return air (20°F delta)
    .lowSidePressurePsi = 69.0f,      // ~40°F evap temp
    .highSidePressurePsi = 278.0f,    // ~120°F condensing temp
    .description = "Normal Cooling - Healthy system"
  },
  
  // Normal_Heating: Healthy system in heating mode
  {
    .highPressureLineTempC = 32.2f,   // 90°F liquid line
    .lowPressureLineTempC = 4.4f,     // 40°F suction line
    .supplyAirTempC = 43.3f,          // 110°F supply air
    .returnAirTempC = 21.1f,          // 70°F return air (-40°F delta)
    .lowSidePressurePsi = 55.0f,      // Low side
    .highSidePressurePsi = 260.0f,    // High side
    .description = "Normal Heating - Healthy system"
  },
  
  // System_Off: Unit completely off
  {
    .highPressureLineTempC = 21.1f,   // 70°F ambient
    .lowPressureLineTempC = 21.1f,    // 70°F ambient
    .supplyAirTempC = 21.1f,          // 70°F
    .returnAirTempC = 21.1f,          // 70°F (0°F delta)
    .lowSidePressurePsi = 120.0f,     // Equalized
    .highSidePressurePsi = 125.0f,    // Equalized (5 PSI delta is normal)
    .description = "System Off - Pressures equalized"
  },
  
  // Low_Refrigerant_Charge: Classic low charge symptoms
  {
    .highPressureLineTempC = 32.2f,   // 90°F liquid line
    .lowPressureLineTempC = 7.2f,     // 45°F suction line
    .supplyAirTempC = 15.6f,          // 60°F supply (poor cooling)
    .returnAirTempC = 23.9f,          // 75°F return (low delta-T)
    .lowSidePressurePsi = 50.0f,      // Low suction pressure
    .highSidePressurePsi = 200.0f,    // Low head pressure
    .description = "Low Refrigerant Charge - Low subcool, high superheat"
  },
  
  // Dirty_Condenser: High head pressure
  {
    .highPressureLineTempC = 48.9f,   // 120°F liquid line (hot!)
    .lowPressureLineTempC = 10.0f,    // 50°F suction line
    .supplyAirTempC = 15.6f,          // 60°F supply
    .returnAirTempC = 23.9f,          // 75°F return
    .lowSidePressurePsi = 69.0f,      // Normal suction
    .highSidePressurePsi = 450.0f,    // High head pressure!
    .description = "Dirty Condenser - High head pressure"
  },
  
  // Dirty_Evaporator: Low suction pressure, low superheat
  {
    .highPressureLineTempC = 37.8f,   // 100°F liquid line
    .lowPressureLineTempC = 1.7f,     // 35°F suction line
    .supplyAirTempC = 15.6f,          // 60°F supply
    .returnAirTempC = 23.9f,          // 75°F return
    .lowSidePressurePsi = 35.0f,      // Low suction pressure
    .highSidePressurePsi = 278.0f,    // Normal head
    .description = "Dirty Evaporator - Low suction, low superheat"
  },
  
  // Restriction_TXV: Restriction pattern
  {
    .highPressureLineTempC = 32.2f,   // 90°F liquid line (cool)
    .lowPressureLineTempC = 1.7f,     // 35°F suction line
    .supplyAirTempC = 15.6f,          // 60°F supply
    .returnAirTempC = 23.9f,          // 75°F return
    .lowSidePressurePsi = 35.0f,      // Low suction
    .highSidePressurePsi = 278.0f,    // Normal head
    .description = "TXV Restriction - High subcool, high superheat"
  },
  
  // Suction_Line_Freezing: Evaporator freezing
  {
    .highPressureLineTempC = 37.8f,   // 100°F liquid line
    .lowPressureLineTempC = -1.1f,    // 30°F suction line (freezing!)
    .supplyAirTempC = 10.0f,          // 50°F supply
    .returnAirTempC = 23.9f,          // 75°F return
    .lowSidePressurePsi = 30.0f,      // Very low suction
    .highSidePressurePsi = 278.0f,    // Normal head
    .description = "Suction Line Freezing - Very low evap temp"
  },
  
  // Static_Not_Equalizing: System off but restriction present
  {
    .highPressureLineTempC = 21.1f,   // 70°F ambient
    .lowPressureLineTempC = 21.1f,    // 70°F ambient
    .supplyAirTempC = 21.1f,          // 70°F
    .returnAirTempC = 21.1f,          // 70°F
    .lowSidePressurePsi = 80.0f,      // Not equalized
    .highSidePressurePsi = 180.0f,    // Big delta (100 PSI - bad!)
    .description = "Static Not Equalizing - Possible restriction"
  },
  
  // Weak_Airflow: Low delta-T, system running but poor airflow
  {
    .highPressureLineTempC = 37.8f,   // 100°F liquid line
    .lowPressureLineTempC = 10.0f,    // 50°F suction line
    .supplyAirTempC = 18.3f,          // 65°F supply (warm)
    .returnAirTempC = 21.1f,          // 70°F return (only 5°F delta!)
    .lowSidePressurePsi = 69.0f,      // Normal suction
    .highSidePressurePsi = 278.0f,    // Normal head
    .description = "Weak Airflow - Low delta-T despite running"
  },
  
  // Multiple_Faults: Several issues at once
  {
    .highPressureLineTempC = 48.9f,   // 120°F liquid line (hot!)
    .lowPressureLineTempC = -1.1f,    // 30°F suction line (freezing!)
    .supplyAirTempC = 10.0f,          // 50°F supply
    .returnAirTempC = 15.6f,          // 60°F return (low delta)
    .lowSidePressurePsi = 28.0f,      // Very low suction
    .highSidePressurePsi = 480.0f,    // Very high head
    .description = "Multiple Faults - Severe issues"
  },
  
  // All_Sensors_Fault: Hardware failure simulation
  {
    .highPressureLineTempC = NAN,
    .lowPressureLineTempC = NAN,
    .supplyAirTempC = NAN,
    .returnAirTempC = NAN,
    .lowSidePressurePsi = NAN,
    .highSidePressurePsi = NAN,
    .description = "All Sensors Fault - Hardware failure"
  },
  
  // Custom: Set your own values below
  {
    .highPressureLineTempC = 35.0f,
    .lowPressureLineTempC = 8.0f,
    .supplyAirTempC = 13.0f,
    .returnAirTempC = 24.0f,
    .lowSidePressurePsi = 65.0f,
    .highSidePressurePsi = 280.0f,
    .description = "Custom Test Values"
  }
};

// ===================== TEST DATA GETTERS =====================

inline const TestSensorData& getActiveTestData() {
  return TEST_SCENARIOS[(uint8_t)ACTIVE_TEST_SCENARIO];
}

inline void applyTestDataToHvacTemps(HvacTemperatures& temps) {
  if (!TEST_MODE_ENABLED) return;
  
  const TestSensorData& testData = getActiveTestData();
  
  temps.highPressureLineTempC = testData.highPressureLineTempC;
  temps.lowPressureLineTempC = testData.lowPressureLineTempC;
  temps.supplyAirTempC = testData.supplyAirTempC;
  temps.returnAirTempC = testData.returnAirTempC;
  
  // Compute delta-T
  if (!isnan(temps.returnAirTempC) && !isnan(temps.supplyAirTempC)) {
    temps.deltaTempC = temps.returnAirTempC - temps.supplyAirTempC;
  } else {
    temps.deltaTempC = NAN;
  }
  
  temps.updatedAtMs = millis();
}

inline void applyTestDataToHvacPressures(HvacPressures& pressures) {
  if (!TEST_MODE_ENABLED) return;
  
  const TestSensorData& testData = getActiveTestData();
  
  pressures.lowSidePressurePsi = testData.lowSidePressurePsi;
  pressures.highSidePressurePsi = testData.highSidePressurePsi;
  pressures.updatedAtMs = millis();
}

// ===================== TEST MODE CONTROL FUNCTIONS =====================

inline void enableTestMode(TestScenario scenario = TestScenario::Normal_Cooling) {
  TEST_MODE_ENABLED = true;
  ACTIVE_TEST_SCENARIO = scenario;
  
  Serial.println("\n*** TEST MODE ENABLED ***");
  Serial.printf("Scenario: %s\n", getActiveTestData().description);
  Serial.println("***************************\n");
}

inline void disableTestMode() {
  TEST_MODE_ENABLED = false;
  
  Serial.println("\n*** TEST MODE DISABLED ***");
  Serial.println("Using live sensor readings");
  Serial.println("**************************\n");
}

inline void setTestScenario(TestScenario scenario) {
  ACTIVE_TEST_SCENARIO = scenario;
  
  Serial.println("\n*** TEST SCENARIO CHANGED ***");
  Serial.printf("New Scenario: %s\n", getActiveTestData().description);
  Serial.println("*****************************\n");
}

inline void cycleTestScenarios() {
  uint8_t nextScenario = ((uint8_t)ACTIVE_TEST_SCENARIO + 1) % 
                         (sizeof(TEST_SCENARIOS) / sizeof(TEST_SCENARIOS[0]));
  setTestScenario((TestScenario)nextScenario);
}

inline void printTestScenarioList() {
  Serial.println("\n=== Available Test Scenarios ===");
  const size_t scenarioCount = sizeof(TEST_SCENARIOS) / sizeof(TEST_SCENARIOS[0]);
  
  for (size_t i = 0; i < scenarioCount; i++) {
    Serial.printf("%2zu: %s\n", i, TEST_SCENARIOS[i].description);
  }
  Serial.println("================================\n");
}

inline void printCurrentTestData() {
  if (!TEST_MODE_ENABLED) {
    Serial.println("Test mode is disabled");
    return;
  }
  
  const TestSensorData& data = getActiveTestData();
  
  Serial.println("\n=== Current Test Data ===");
  Serial.printf("Scenario: %s\n\n", data.description);
  
  Serial.println("Temperatures:");
  Serial.printf("  High Pressure Line: %.1f°C (%.1f°F)\n", 
                data.highPressureLineTempC,
                data.highPressureLineTempC * 9.0f / 5.0f + 32.0f);
  Serial.printf("  Low Pressure Line:  %.1f°C (%.1f°F)\n",
                data.lowPressureLineTempC,
                data.lowPressureLineTempC * 9.0f / 5.0f + 32.0f);
  Serial.printf("  Supply Air:         %.1f°C (%.1f°F)\n",
                data.supplyAirTempC,
                data.supplyAirTempC * 9.0f / 5.0f + 32.0f);
  Serial.printf("  Return Air:         %.1f°C (%.1f°F)\n",
                data.returnAirTempC,
                data.returnAirTempC * 9.0f / 5.0f + 32.0f);
  
  Serial.println("\nPressures:");
  Serial.printf("  Low Side:  %.1f PSI\n", data.lowSidePressurePsi);
  Serial.printf("  High Side: %.1f PSI\n", data.highSidePressurePsi);
  Serial.printf("  Delta:     %.1f PSI\n", 
                data.highSidePressurePsi - data.lowSidePressurePsi);
  
  Serial.println("=========================\n");
}

#endif // HVAC_TEST_DATA_H
