#ifndef ADS1115_DRIVER_H
#define ADS1115_DRIVER_H

#include <Arduino.h>
#include <Wire.h>
#include "hvac_config.h"

// ===================== ADS1115 ADC DRIVER =====================

// Register addresses
static constexpr uint16_t ADS1115_REG_CONVERSION = 0x00;
static constexpr uint16_t ADS1115_REG_CONFIG     = 0x01;

// Configuration register bits
static constexpr uint16_t ADS1115_PGA_4_096V     = 0x0200;  // ±4.096V range
static constexpr uint16_t ADS1115_MODE_SINGLESHOT = 0x0100; // Single-shot mode
static constexpr uint16_t ADS1115_DR_128SPS      = 0x0080;  // 128 samples/sec
static constexpr uint16_t ADS1115_COMP_DISABLE   = 0x0003;  // Comparator disabled

// ===================== I2C HELPERS =====================
inline bool isI2CDevicePresent(TwoWire &bus, uint8_t address) {
  bus.beginTransmission(address);
  return (bus.endTransmission() == 0);
}

inline void initADS1115() {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);  // 400 kHz I2C fast mode

  if (isI2CDevicePresent(Wire, ADS1115_I2C_ADDRESS)) {
    Serial.println("ADS1115 detected");
  } else {
    Serial.println("WARNING: ADS1115 not responding");
  }
}

// ===================== REGISTER ACCESS =====================
inline bool ads1115WriteRegister16(uint8_t i2cAddress, uint16_t reg, uint16_t value) {
  Wire.beginTransmission(i2cAddress);
  Wire.write((uint8_t)reg);
  Wire.write((uint8_t)(value >> 8));   // MSB
  Wire.write((uint8_t)(value & 0xFF)); // LSB
  return (Wire.endTransmission() == 0);
}

inline bool ads1115ReadRegister16(uint8_t i2cAddress, uint16_t reg, uint16_t &outValue) {
  Wire.beginTransmission(i2cAddress);
  Wire.write((uint8_t)reg);
  if (Wire.endTransmission(false) != 0) return false;

  if (Wire.requestFrom((int)i2cAddress, 2) != 2) return false;

  uint8_t msb = Wire.read();
  uint8_t lsb = Wire.read();
  outValue = ((uint16_t)msb << 8) | lsb;
  return true;
}

// ===================== CONVERSION =====================
inline float ads1115CountsToVolts(int16_t counts, float fullScaleVolts) {
  // ADS1115 is 16-bit signed. Full scale is ±fullScaleVolts
  // LSB size = fullScaleVolts / 32768
  return (float)counts * (fullScaleVolts / 32768.0f);
}

// ===================== CHANNEL READING =====================
// Read single-ended channel (0-3) in single-shot mode
inline bool readAds1115SingleEndedCounts(uint8_t channel, int16_t &outCounts) {
  if (channel > 3) return false;

  // MUX bits for single-ended:
  // AIN0-GND = 0x4000, AIN1-GND = 0x5000, AIN2-GND = 0x6000, AIN3-GND = 0x7000
  uint16_t muxBits = 0x4000 + (channel * 0x1000);

  // Build config: start conversion, single-shot, ±4.096V, 128 SPS
  uint16_t config =
    0x8000 |                  // OS = 1 (start conversion)
    muxBits |                 // MUX = single-ended channel
    ADS1115_PGA_4_096V |      // PGA = ±4.096V
    ADS1115_MODE_SINGLESHOT | // Single-shot mode
    ADS1115_DR_128SPS |       // Data rate
    ADS1115_COMP_DISABLE;     // Comparator disabled

  if (!ads1115WriteRegister16(ADS1115_I2C_ADDRESS, ADS1115_REG_CONFIG, config)) {
    return false;
  }

  // Wait for conversion at 128 SPS (~7.8ms)
  delay(ADS1115_CONVERSION_DELAY_MS);

  uint16_t rawConversion = 0;
  if (!ads1115ReadRegister16(ADS1115_I2C_ADDRESS, ADS1115_REG_CONVERSION, rawConversion)) {
    return false;
  }

  outCounts = (int16_t)rawConversion;
  return true;
}

// Read channel voltage at ADC pin (after voltage divider)
inline bool readAds1115ChannelVoltageAtAdcPin(uint8_t channel, float &outVoltageAtAdcPin) {
  int16_t counts = 0;
  if (!readAds1115SingleEndedCounts(channel, counts)) {
    outVoltageAtAdcPin = NAN;
    return false;
  }

  // For PGA ±4.096V
  outVoltageAtAdcPin = ads1115CountsToVolts(counts, ADS1115_PGA_VOLTAGE);
  return true;
}

// Convert ADC-pin voltage to sensor output voltage (undo divider)
inline float convertAdcPinVoltageToSensorVoltage(float adcPinVoltage) {
  if (isnan(adcPinVoltage)) return NAN;
  return adcPinVoltage * VOLTAGE_DIVIDER_GAIN_TO_SENSOR;
}

// ===================== PRESSURE CONVERSION =====================
inline float clampFloat(float value, float minValue, float maxValue) {
  if (value < minValue) return minValue;
  if (value > maxValue) return maxValue;
  return value;
}

inline float convertSensorVoltageToPsiCalibrated(
  float sensorVoltageV,
  float voltageAtFullScalePsi,
  float fullScalePsi,
  float zeroCalVoltageV
) {
  if (isnan(sensorVoltageV)) return NAN;

  const float spanV = voltageAtFullScalePsi - zeroCalVoltageV;
  if (spanV <= 0.0f || fullScalePsi <= 0.0f) return NAN;

  // Clamp to valid range
  float v = clampFloat(sensorVoltageV, zeroCalVoltageV, voltageAtFullScalePsi);

  float fraction = (v - zeroCalVoltageV) / spanV;
  float psi = fraction * fullScalePsi;

  // Suppress noise near zero
  if (psi < 0.0f) psi = 0.0f;
  return psi;
}

inline float applyPsiDeadband(float psi, float deadbandPsi = PSI_DEADBAND) {
  if (isnan(psi)) return NAN;
  return (psi < deadbandPsi) ? 0.0f : psi;
}

// ===================== SENSOR-SPECIFIC READERS =====================
// A0: TDH33 (0-1000 PSI, 0-5V)
inline bool readPressureSensorA0Psi(float &outPressurePsi) {
  float adcPinVoltageV = NAN;

  if (!readAds1115ChannelVoltageAtAdcPin(0, adcPinVoltageV)) {
    outPressurePsi = NAN;
    return false;
  }

  float sensorVoltageV = convertAdcPinVoltageToSensorVoltage(adcPinVoltageV);

  outPressurePsi = convertSensorVoltageToPsiCalibrated(
    sensorVoltageV,
    PRESSURE_SENSOR_A0_VOLTAGE_AT_FULL_PSI,
    PRESSURE_SENSOR_A0_FULL_SCALE_PSI,
    A0_ZERO_CAL_V
  );

  outPressurePsi = applyPsiDeadband(outPressurePsi);
  return true;
}

// A1: TD1000 (0-600 PSI, 1-5V)
inline bool readPressureSensorA1Psi(float &outPressurePsi) {
  float adcPinVoltageV = NAN;

  if (!readAds1115ChannelVoltageAtAdcPin(1, adcPinVoltageV)) {
    outPressurePsi = NAN;
    return false;
  }

  float sensorVoltageV = convertAdcPinVoltageToSensorVoltage(adcPinVoltageV);

  outPressurePsi = convertSensorVoltageToPsiCalibrated(
    sensorVoltageV,
    PRESSURE_SENSOR_A1_VOLTAGE_AT_FULL_PSI,
    PRESSURE_SENSOR_A1_FULL_SCALE_PSI,
    A1_ZERO_CAL_V
  );

  outPressurePsi = applyPsiDeadband(outPressurePsi);
  return true;
}

// ===================== PRINTING =====================
inline void printPressureOrFault(const char* label, float pressurePsi) {
  if (isnan(pressurePsi)) {
    Serial.printf("%s: FAULT\n", label);
  } else {
    Serial.printf("%s: %.1f PSI\n", label, pressurePsi);
  }
}

#endif // ADS1115_DRIVER_H
