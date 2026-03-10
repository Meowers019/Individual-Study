#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <math.h>

#include "ads1115_driver.h"
#include "hvac_config.h"
#include "hvac_diagnostics.h"
#include "hvac_sensors.h"
#include "hvac_test_data.h"
#include "hvac_types.h"
#include "max31855_driver.h"
#include "r22_tables.h"

constexpr int SERIAL_BAUD_RATE = 115200

#endif