// ===================== ESP32 HVAC MONITORING SYSTEM =====================
// Modular architecture for HVAC diagnostics and monitoring

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <math.h>

#include "hvac_types.h"
#include "hvac_config.h"
#include "max31855_driver.h"
#include "ads1115_driver.h"

// Refrigerant selection: REFRIG_R22 or REFRIG_R410A
#define REFRIGERANT_TYPE REFRIG_R410A

#include "type_refrig_tables.h"
#include "hvac_sensors.h"
#include "hvac_diagnostics.h"
#include "hvac_test_data.h"
#include "network_setup.h"

// ===================== GLOBAL STATE =====================
HvacState hvacState = {
  HvacSystemState::Off,
  HvacMode::Off,
  HvacSubstate::None,
  DiagnosticState::Unknown,
  0
};

HvacTemperatures hvacTemps = {
  NAN,  // highPressureLineTempC
  NAN,  // lowPressureLineTempC
  NAN,  // supplyAirTempC
  NAN,  // returnAirTempC
  NAN,  // deltaTempC
  0     // updatedAtMs
};

HvacPressures hvacPressures = {
  NAN,  // lowSidePressurePsi
  NAN,  // highSidePressurePsi
  0     // updatedAtMs
};

HvacSaturationTemps hvacSatTemps = {
  NAN,  // lowSideSatTempF
  NAN,  // highSideSatTempF
  0     // updatedAtMs
};

HvacShSc hvacShSc = {
  NAN,  // superheatF
  NAN,  // subcoolingF
  0     // updatedAtMs
};

FaultReport faultReport;

// Thermocouple sensor readings (live mode only, kept global for serial reporting)
static Max31855Reading sensorHighPressure;
static Max31855Reading sensorLowPressure;
static Max31855Reading sensorSupplyAir;
static Max31855Reading sensorReturnAir;

// ===================== SENSOR READING =====================
static void readSensors() {
  if (TEST_MODE_ENABLED) {
    applyTestDataToHvacTemps(hvacTemps);
    applyTestDataToHvacPressures(hvacPressures);
    return;
  }

  // Read thermocouples
  sensorHighPressure = readMax31855Filtered(MAX31855_CS_HIGH_PRESSURE);
  sensorLowPressure  = readMax31855Filtered(MAX31855_CS_LOW_PRESSURE);
  sensorSupplyAir    = readMax31855Filtered(MAX31855_CS_SUPPLY_AIR);
  sensorReturnAir    = readMax31855Filtered(MAX31855_CS_RETURN_AIR);

  // Update temperature state
  hvacTemps.highPressureLineTempC = sensorHighPressure.thermocoupleTempC;
  hvacTemps.lowPressureLineTempC  = sensorLowPressure.thermocoupleTempC;

  // Apply calibration offsets to supply/return air
  hvacTemps.supplyAirTempC = isnan(sensorSupplyAir.thermocoupleTempC) ? NAN
    : sensorSupplyAir.thermocoupleTempC + SUPPLY_TEMP_OFFSET_C;

  hvacTemps.returnAirTempC = isnan(sensorReturnAir.thermocoupleTempC) ? NAN
    : sensorReturnAir.thermocoupleTempC + RETURN_TEMP_OFFSET_C;

  // Compute delta-T (Return - Supply)
  hvacTemps.deltaTempC = (!isnan(hvacTemps.returnAirTempC) && !isnan(hvacTemps.supplyAirTempC))
    ? hvacTemps.returnAirTempC - hvacTemps.supplyAirTempC
    : NAN;

  hvacTemps.updatedAtMs = millis();

  // Update pressures
  updateHvacPressures(hvacPressures);
}

// ===================== DIAGNOSTICS =====================
static void runDiagnostics() {
  hvacState.diagnostic = DiagnosticState::Unknown;
  updateHvacRunStateFromDeltaTOrDeltaPsi(hvacTemps, hvacPressures, hvacState);
  updateHvacModeFromTemperatures(hvacTemps, hvacState);
  updateHvacSaturationTempsFromPressures(hvacPressures, hvacSatTemps);
  updateHvacSuperheatSubcool(hvacTemps, hvacSatTemps, hvacShSc);

  faultReport = evaluateFaultsHeatPumpAll(
    hvacTemps, hvacPressures, hvacSatTemps, hvacShSc,
    hvacState.systemState, hvacState.mode
  );

  // Fault report may overwrite diagnostic (SensorFault takes precedence)
  if (hvacState.diagnostic != DiagnosticState::SensorFault) {
    hvacState.diagnostic = faultReport.diag;
  }
}

// ===================== SERIAL REPORT =====================
static void printMonitoringReport() {
  Serial.println("=== BEGIN MONITORING ===");
  if (TEST_MODE_ENABLED) {
    Serial.printf("*** TEST MODE: %s ***\n", getActiveTestData().description);
  }
  Serial.println();

  Serial.printf("SYS=%s  MODE=%s  DIAG=%s\n",
    hvacState.systemState == HvacSystemState::Running ? "RUNNING" : "OFF",
    hvacModeToString(hvacState.mode),
    diagToString(hvacState.diagnostic)
  );
  Serial.println();

  // Raw thermocouple data (live mode only)
  if (!TEST_MODE_ENABLED) {
    Serial.println("---- MAX31855 Raw/Decoded ----");
    printMax31855Reading("HighPressure", sensorHighPressure);
    printMax31855Reading("LowPressure ", sensorLowPressure);
    printMax31855Reading("SupplyAir   ", sensorSupplyAir);
    printMax31855Reading("ReturnAir   ", sensorReturnAir);
    Serial.println();
  }

  Serial.println("---- Temperatures ----");
  printTemperatureOrFaultF("High Pressure Line", hvacTemps.highPressureLineTempC);
  printTemperatureOrFaultF("Low Pressure Line ", hvacTemps.lowPressureLineTempC);
  printTemperatureOrFaultF("Supply Air        ", hvacTemps.supplyAirTempC);
  printTemperatureOrFaultF("Return Air        ", hvacTemps.returnAirTempC);

  if (!isnan(hvacTemps.deltaTempC)) {
    Serial.printf("DeltaT (Return-Supply): %.2f F\n",
      fabsf(deltaCelsiusToDeltaFahrenheit(hvacTemps.deltaTempC)));
  } else {
    Serial.println("DeltaT (Return-Supply): FAULT");
  }
  Serial.println();

  // ADC voltage details (live mode only)
  if (!TEST_MODE_ENABLED) {
    float adcVoltageA0 = NAN;
    float adcVoltageA1 = NAN;
    bool a0Ok = readAds1115ChannelVoltageAtAdcPin(0, adcVoltageA0);
    bool a1Ok = readAds1115ChannelVoltageAtAdcPin(1, adcVoltageA1);
    float sensorVoltageA0 = convertAdcPinVoltageToSensorVoltage(adcVoltageA0);
    float sensorVoltageA1 = convertAdcPinVoltageToSensorVoltage(adcVoltageA1);

    Serial.println("---- ADS1115 Voltages ----");
    if (a0Ok) {
      Serial.printf("A0 TDH33:  %.3f V (ADC)  %.3f V (sensor)\n", adcVoltageA0, sensorVoltageA0);
    } else {
      Serial.println("A0 TDH33: read failed");
    }
    if (a1Ok) {
      Serial.printf("A1 TD1000: %.3f V (ADC)  %.3f V (sensor)\n", adcVoltageA1, sensorVoltageA1);
    } else {
      Serial.println("A1 TD1000: read failed");
    }
    Serial.println();
  }

  Serial.println("---- Pressures ----");
  printPressureOrFault("High Side (0-1000 PSI)", hvacPressures.highSidePressurePsi);
  printPressureOrFault("Low Side  (0-600 PSI) ", hvacPressures.lowSidePressurePsi);

  float deltaPsi = computeDeltaPsi(hvacPressures);
  if (isnan(deltaPsi)) {
    Serial.println("DeltaPsi (High-Low): FAULT");
  } else {
    Serial.printf("DeltaPsi (High-Low): %.1f PSI\n", deltaPsi);
  }
  Serial.println();

  Serial.println("---- Saturation Temps (" REFRIGERANT_NAME ") ----");
  printSatTempOrFault("Low Side Sat Temp",  hvacSatTemps.lowSideSatTempF);
  printSatTempOrFault("High Side Sat Temp", hvacSatTemps.highSideSatTempF);
  Serial.println();

  Serial.println("---- Superheat / Subcooling ----");
  printShScOrFault("Superheat (Suction - SatLow)", hvacShSc.superheatF);
  printShScOrFault("Subcool   (SatHigh - Liquid)", hvacShSc.subcoolingF);
  Serial.println();

  Serial.printf("MODE=%s  DIAG=%s\n",
    hvacModeToString(hvacState.mode),
    diagToString(hvacState.diagnostic)
  );
  printFaultReportLines(faultReport);
  Serial.println();

  Serial.println("=== END MONITORING ===");
  Serial.println();
}

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println("=== HVAC ESP32 Monitoring System ===");

  wifiProvision();     // Captive portal WiFi provisioning
  startLocalServer();  // Start local HTTP server

  // To enable test mode, uncomment ONE of the lines below:
  // 
  // enableTestMode(TestScenario::Normal_Cooling);
  // enableTestMode(TestScenario::Low_Refrigerant_Charge);
  // printTestScenarioList();

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
  readSensors();
  runDiagnostics();
  printMonitoringReport();

  handleLocalServer();
  sendToAWS();

  delay(SENSOR_READ_INTERVAL_MS);
}
