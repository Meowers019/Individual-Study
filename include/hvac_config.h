#ifndef HVAC_CONFIG_H
#define HVAC_CONFIG_H

#include <stdint.h>

// ===================== PINOUT =====================
// I2C for ADS1115
static constexpr uint8_t I2C_SDA_PIN = 21;
static constexpr uint8_t I2C_SCL_PIN = 22;
static constexpr uint8_t ADS1115_I2C_ADDRESS = 0x48;  // ADDR = GND

// SPI for MAX31855 (read-only)
static constexpr uint8_t SPI_CLOCK_PIN    = 18;
static constexpr uint8_t SPI_MISO_DO_PIN  = 19;

// MAX31855 Chip Select pins
static constexpr uint8_t MAX31855_CS_HIGH_PRESSURE = 16;  // Good cable
static constexpr uint8_t MAX31855_CS_LOW_PRESSURE  = 17;  // Good cable
static constexpr uint8_t MAX31855_CS_SUPPLY_AIR    = 26;  // Needs replacement
static constexpr uint8_t MAX31855_CS_RETURN_AIR    = 27;  // Needs replacement

// ===================== TEMPERATURE CALIBRATION =====================
// Offsets for shitty cables that need replacement
// NOTE: These are in Celsius. 1°F = 0.556°C
static constexpr float SUPPLY_TEMP_OFFSET_C = 1.0f * 5.0f / 9.0f;  // ~0.556°C (1°F)
static constexpr float RETURN_TEMP_OFFSET_C = 1.0f * 5.0f / 9.0f;  // ~0.556°C (1°F)

// ===================== VOLTAGE DIVIDER =====================
// Sensor -> 10kΩ -> ADC -> 20kΩ -> GND
// Vadc = Vsensor × (20k/(10k+20k)) = Vsensor × (2/3)
// Therefore: Vsensor = Vadc × 1.5
static constexpr float VOLTAGE_DIVIDER_GAIN_TO_SENSOR = 3.0f / 2.0f;

// ===================== PRESSURE SENSOR SPECS =====================
// A0: TDH33 (0-1000 PSI, 0-5V)
static constexpr float PRESSURE_SENSOR_A0_FULL_SCALE_PSI = 1000.0f;
static constexpr float PRESSURE_SENSOR_A0_VOLTAGE_AT_ZERO_PSI = 0.0f;
static constexpr float PRESSURE_SENSOR_A0_VOLTAGE_AT_FULL_PSI = 5.0f;

// A1: TD1000 (0-600 PSI, 1-5V)
static constexpr float PRESSURE_SENSOR_A1_FULL_SCALE_PSI = 600.0f;
static constexpr float PRESSURE_SENSOR_A1_VOLTAGE_AT_ZERO_PSI = 1.0f;
static constexpr float PRESSURE_SENSOR_A1_VOLTAGE_AT_FULL_PSI = 5.0f;

// Zero calibration voltages (measure when pressure = 0 PSI)
static constexpr float A0_ZERO_CAL_V = 0.151f;  // Update from live readings
static constexpr float A1_ZERO_CAL_V = 1.025f;  // Update from live readings

// Noise suppression
static constexpr float PSI_DEADBAND = 0.5f;

// ===================== HVAC MODE DETECTION THRESHOLDS =====================
// Temperature-based (in Celsius)
static constexpr float DELTA_COOL_ON_C      = +3.0f;  // Return - Supply ≥ +3°C
static constexpr float DELTA_HEAT_ON_C      = -3.0f;  // Return - Supply ≤ -3°C
static constexpr float DELTA_OFF_BAND_C     =  1.0f;  // |delta| ≤ 1°C = likely off

static constexpr float SUPPLY_COOL_MAX_C    = 18.0f;  // Supply ≤ 18°C supports cooling
static constexpr float SUPPLY_HEAT_MIN_C    = 30.0f;  // Supply ≥ 30°C supports heating

static constexpr int   MODE_CONFIRM_COUNT   = 3;      // Consecutive confirmations needed

// ===================== RUNNING DETECTION THRESHOLDS =====================
static constexpr float RUNNING_DELTA_T_C_THRESHOLD  = 0.6f;   // ~1.1°F
static constexpr float RUNNING_DELTA_PSI_THRESHOLD  = 25.0f;  // PSI

// ===================== FAULT DETECTION THRESHOLDS =====================
// Pressure limits (cooling mode)
static constexpr float PSI_HIGH_MAX_COOL = 500.0f;
static constexpr float PSI_LOW_MIN_COOL  = 40.0f;

// Temperature limits (Fahrenheit)
static constexpr float SUCTION_FREEZE_F  = 32.0f;
static constexpr float LIQUID_TOO_HOT_F  = 130.0f;

// Delta-T limits (Fahrenheit)
static constexpr float DELTAT_LOW_F      = 10.0f;
static constexpr float DELTAT_HIGH_F     = 25.0f;
static constexpr float DELTAT_ZERO_F     = 2.0f;

// Superheat/Subcooling limits (Fahrenheit)
static constexpr float SUBCOOL_LOW_F     = 5.0f;
static constexpr float SUBCOOL_HIGH_F    = 20.0f;
static constexpr float SUPERHEAT_LOW_F   = 2.0f;
static constexpr float SUPERHEAT_HIGH_F  = 25.0f;

// OFF state diagnostics
static constexpr float STATIC_EQUALIZE_DELTA_PSI_MAX = 25.0f;

// ===================== TIMING =====================
static constexpr uint32_t SENSOR_READ_INTERVAL_MS = 2000;  // Current loop delay
static constexpr uint32_t NETWORK_SEND_INTERVAL_MS = 15000; // 15 sec for WiFi (future)

// ===================== MAX31855 SPI =====================
static constexpr uint32_t MAX31855_SPI_CLOCK_HZ = 1000000;  // 1 MHz
static constexpr int MAX31855_FILTER_SAMPLE_COUNT = 5;      // Multi-sample filtering

// ===================== ADS1115 ADC =====================
static constexpr float ADS1115_PGA_VOLTAGE = 4.096f;  // ±4.096V range
static constexpr uint8_t ADS1115_CONVERSION_DELAY_MS = 10;  // @128 SPS

#endif // HVAC_CONFIG_H
