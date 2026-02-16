#ifndef MAIN_H
#define MAIN_H

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
    0};

static HvacTemperatures hvacTemps = {
    NAN, // highPressureLineTempC
    NAN, // lowPressureLineTempC
    NAN, // supplyAirTempC
    NAN, // returnAirTempC
    NAN, // deltaTempC
    0    // updatedAtMs
};

static HvacPressures hvacPressures = {
    NAN, // lowSidePressurePsi
    NAN, // highSidePressurePsi
    0    // updatedAtMs
};

static HvacSaturationTemps hvacSatTemps = {
    NAN, // lowSideSatTempF
    NAN, // highSideSatTempF
    0    // updatedAtMs
};

static HvacShSc hvacShSc = {
    NAN, // superheatF
    NAN, // subcoolingF
    0    // updatedAtMs
};

static FaultReport faultReport;

#endif