#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <math.h>

// HVAC System Headers
#include "ads1115_driver.h"
#include "hvac_config.h"
#include "hvac_diagnostics.h"
#include "hvac_sensors.h"
#include "hvac_test_data.h"
#include "hvac_types.h"
#include "max31855_driver.h"
#include "r22_tables.h"

#define SERIAL_BAUD_RATE 115200

// ===================== GLOBAL STATE =====================
static HvacState hvacState{};
static HvacTemperatures hvacTemps{};
static HvacPressures hvacPressures{};
static HvacSaturationTemps hvacSatTemps{};
static HvacShSc hvacShSc{};

static FaultReport faultReport;

#endif