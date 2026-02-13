// ===================== ESP32 HVAC MONITORING SYSTEM =====================
// Modular architecture for HVAC diagnostics and monitoring
// Prepared for WiFi/ESPHome/Home Assistant integration
// Testing Testing

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <math.h>

// HVAC System Headers
#include "hvac_types.h"
#include "hvac_config.h"
#include "max31855_driver.h"
#include "ads1115_driver.h"
#include "r22_tables.h"
#include "hvac_sensors.h"
#include "hvac_diagnostics.h"
#include "hvac_test_data.h"

// ===================== GLOBAL STATE =====================
static HvacState hvacState = {
  HvacSystemState::Off,
  HvacMode::Off,
  HvacSubstate::None,
  DiagnosticState::Unknown,
  0
};

static HvacTemperatures hvacTemps = {
  NAN,  // highPressureLineTempC
  NAN,  // lowPressureLineTempC
  NAN,  // supplyAirTempC
  NAN,  // returnAirTempC
  NAN,  // deltaTempC
  0     // updatedAtMs
};

static HvacPressures hvacPressures = {
  NAN,  // lowSidePressurePsi
  NAN,  // highSidePressurePsi
  0     // updatedAtMs
};

static HvacSaturationTemps hvacSatTemps = {
  NAN,  // lowSideSatTempF
  NAN,  // highSideSatTempF
  0     // updatedAtMs
};

static HvacShSc hvacShSc = {
  NAN,  // superheatF
  NAN,  // subcoolingF
  0     // updatedAtMs
};

static FaultReport faultReport;

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println("\n=== HVAC ESP32 Monitoring System ===");
  Serial.println("Modular architecture - Ready for WiFi integration");

  // ===================== TEST MODE SETUP =====================
  // Uncomment ONE of these lines to enable test mode with a specific scenario:
  
  // enableTestMode(TestScenario::Normal_Cooling);
  // enableTestMode(TestScenario::Normal_Heating);
  // enableTestMode(TestScenario::System_Off);
  // enableTestMode(TestScenario::Low_Refrigerant_Charge);
  // enableTestMode(TestScenario::Dirty_Condenser);
  // enableTestMode(TestScenario::Dirty_Evaporator);
  // enableTestMode(TestScenario::Restriction_TXV);
  // enableTestMode(TestScenario::Suction_Line_Freezing);
  // enableTestMode(TestScenario::Static_Not_Equalizing);
  // enableTestMode(TestScenario::Weak_Airflow);
  // enableTestMode(TestScenario::Multiple_Faults);
  // enableTestMode(TestScenario::All_Sensors_Fault);
  // enableTestMode(TestScenario::Custom);
  
  // printTestScenarioList();  // Show all available scenarios
  
  // ===================== HARDWARE INIT =====================
  if (!TEST_MODE_ENABLED) {
    initADS1115();
    initMax31855Bus();
  } else {
    Serial.println("Skipping hardware initialization (TEST MODE)");
  }

  Serial.println("Initialization complete.\n");
}

// ===================== LOOP =====================
void loop() {
  // Declare sensor readings (used for live mode only)
  Max31855Reading sensorHighPressure = {};
  Max31855Reading sensorLowPressure = {};
  Max31855Reading sensorSupplyAir = {};
  Max31855Reading sensorReturnAir = {};

  // ---------- READ THERMOCOUPLES (or use test data) ----------
  if (TEST_MODE_ENABLED) {
    // Use simulated test data
    applyTestDataToHvacTemps(hvacTemps);
    applyTestDataToHvacPressures(hvacPressures);
  } else {
    // Read actual hardware sensors
    sensorHighPressure = readMax31855Filtered(MAX31855_CS_HIGH_PRESSURE);
    sensorLowPressure  = readMax31855Filtered(MAX31855_CS_LOW_PRESSURE);
    sensorSupplyAir    = readMax31855Filtered(MAX31855_CS_SUPPLY_AIR);
    sensorReturnAir    = readMax31855Filtered(MAX31855_CS_RETURN_AIR);

    // ---------- UPDATE TEMPERATURE STATE ----------
    hvacTemps.highPressureLineTempC = sensorHighPressure.thermocoupleTempC;
    hvacTemps.lowPressureLineTempC  = sensorLowPressure.thermocoupleTempC;

    // Apply calibration offsets to supply/return air
    if (!isnan(sensorSupplyAir.thermocoupleTempC)) {
      hvacTemps.supplyAirTempC = sensorSupplyAir.thermocoupleTempC + SUPPLY_TEMP_OFFSET_C;
    } else {
      hvacTemps.supplyAirTempC = NAN;
    }

    if (!isnan(sensorReturnAir.thermocoupleTempC)) {
      hvacTemps.returnAirTempC = sensorReturnAir.thermocoupleTempC + RETURN_TEMP_OFFSET_C;
    } else {
      hvacTemps.returnAirTempC = NAN;
    }

    // Compute delta-T (Return - Supply)
    if (!isnan(hvacTemps.returnAirTempC) && !isnan(hvacTemps.supplyAirTempC)) {
      hvacTemps.deltaTempC = hvacTemps.returnAirTempC - hvacTemps.supplyAirTempC;
    } else {
      hvacTemps.deltaTempC = NAN;
    }
    hvacTemps.updatedAtMs = millis();

    // ---------- UPDATE PRESSURES ----------
    updateHvacPressures(hvacPressures);
  }

  // ---------- HVAC STATE ANALYSIS ----------
  hvacState.diagnostic = DiagnosticState::Unknown;
  updateHvacRunStateFromDeltaTOrDeltaPsi(hvacTemps, hvacPressures, hvacState);

  // ---------- DETERMINE HVAC MODE ----------
  updateHvacModeFromTemperatures(hvacTemps, hvacState);

  // ---------- UPDATE SATURATION TEMPS & SH/SC ----------
  updateHvacSaturationTempsFromPressures(hvacPressures, hvacSatTemps);
  updateHvacSuperheatSubcool(hvacTemps, hvacSatTemps, hvacShSc);

  // ---------- EVALUATE FAULTS ----------
  faultReport = evaluateFaultsHeatPumpAll(
    hvacTemps, hvacPressures, hvacSatTemps, hvacShSc,
    hvacState.systemState, hvacState.mode
  );

  hvacState.diagnostic = faultReport.diag;

  // ===================== SERIAL OUTPUT =====================
  Serial.println("=== BEGIN MONITORING ===");
  if (TEST_MODE_ENABLED) {
    Serial.printf("*** TEST MODE: %s ***\n", getActiveTestData().description);
  }
  Serial.println();
  Serial.printf("SYS=%s  MODE=%s  DIAG=%s\n",
    (hvacState.systemState == HvacSystemState::Running) ? "RUNNING" : "OFF",
    hvacModeToString(hvacState.mode),
    diagToString(hvacState.diagnostic)
  );
  Serial.println();

  // Skip MAX31855 raw readings in test mode
  if (!TEST_MODE_ENABLED) {
    Serial.println("---- MAX31855 Raw/Decoded ----");
    printMax31855Reading("HighPressure", sensorHighPressure);
    printMax31855Reading("LowPressure ", sensorLowPressure);
    printMax31855Reading("SupplyAir   ", sensorSupplyAir);
    printMax31855Reading("ReturnAir   ", sensorReturnAir);
    Serial.println();
  }

  Serial.println("---- HVAC Temps (Thermocouple TC) ----");
  printTemperatureOrFaultF("High Pressure Line", hvacTemps.highPressureLineTempC);
  printTemperatureOrFaultF("Low Pressure Line ", hvacTemps.lowPressureLineTempC);
  printTemperatureOrFaultF("Supply Air        ", hvacTemps.supplyAirTempC);
  printTemperatureOrFaultF("Return Air        ", hvacTemps.returnAirTempC);

  if (!isnan(hvacTemps.deltaTempC)) {
    float deltaTempF = deltaCelsiusToDeltaFahrenheit(hvacTemps.deltaTempC);
    Serial.printf("DeltaT (Return-Supply): %.2f F\n", deltaTempF);
  } else {
    Serial.println("DeltaT (Return-Supply): FAULT");
  }
  Serial.println();

  // Skip ADC voltage details in test mode
  if (!TEST_MODE_ENABLED) {
    float adcVoltageA0 = NAN;
    float adcVoltageA1 = NAN;
    bool a0Ok = readAds1115ChannelVoltageAtAdcPin(0, adcVoltageA0);
    bool a1Ok = readAds1115ChannelVoltageAtAdcPin(1, adcVoltageA1);
    float sensorVoltageA0 = convertAdcPinVoltageToSensorVoltage(adcVoltageA0);
    float sensorVoltageA1 = convertAdcPinVoltageToSensorVoltage(adcVoltageA1);

    Serial.println("---- ADS1115 (Voltage) ----");
    if (a0Ok) {
      Serial.printf("A0 TDH33 (after divider): %.3f V | sensor: %.3f V\n", 
                    adcVoltageA0, sensorVoltageA0);
    } else {
      Serial.println("A0 TDH33 read failed");
    }

    if (a1Ok) {
      Serial.printf("A1 TD1000 (after divider): %.3f V | sensor: %.3f V\n", 
                    adcVoltageA1, sensorVoltageA1);
    } else {
      Serial.println("A1 TD1000 read failed");
    }
    Serial.println();
  }

  Serial.println("---- Pressures ----");
  printPressureOrFault("Low Side (A0 0-5V, 0-1000psi)", hvacPressures.lowSidePressurePsi);
  printPressureOrFault("High Side (A1 1-5V, 0-600psi)", hvacPressures.highSidePressurePsi);

  float deltaPsi = computeDeltaPsi(hvacPressures);
  if (isnan(deltaPsi)) {
    Serial.println("DeltaPsi (High-Low): FAULT");
  } else {
    Serial.printf("DeltaPsi (High-Low): %.1f PSI\n", deltaPsi);
  }
  Serial.println();

  Serial.println("---- Saturation Temps (R-22) ----");
  printSatTempOrFault("Low Side Sat Temp",  hvacSatTemps.lowSideSatTempF);
  printSatTempOrFault("High Side Sat Temp", hvacSatTemps.highSideSatTempF);
  Serial.println();

  Serial.println("---- Superheat / Subcooling ----");
  printShScOrFault("Superheat (Suction - SatLow)", hvacShSc.superheatF);
  printShScOrFault("Subcool  (SatHigh - Liquid)", hvacShSc.subcoolingF);
  Serial.println();

  Serial.printf("MODE=%s  DIAG=%s\n", 
                hvacModeToString(hvacState.mode), 
                diagToString(hvacState.diagnostic));
  printFaultReportLines(faultReport);
  Serial.println();

  Serial.println("=== END MONITORING ===");
  Serial.println();

  delay(SENSOR_READ_INTERVAL_MS);
}
