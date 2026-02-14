#ifndef ADS1115_DRIVER_H
#define ADS1115_DRIVER_H

#include <Arduino.h>
#include <Wire.h>
#include "hvac_config.h"
#include <Adafruit_ADS1X15.h>

extern Adafruit_ADS1115 ads;

// ===================== I2C HELPERS =====================
inline void initADS1115();

// ===================== REGISTER ACCESS =====================
inline bool ads1115WriteRegister16(uint8_t i2cAddress, uint16_t reg, uint16_t value);
inline bool ads1115ReadRegister16(uint8_t i2cAddress, uint16_t reg, uint16_t &outValue);

// ===================== CONVERSION =====================
inline float ads1115CountsToVolts(int16_t counts, float fullScaleVolts);

// ===================== CHANNEL READING =====================
// Read single-ended channel (0-3) in single-shot mode
inline bool readAds1115SingleEndedCounts(uint8_t channel, int16_t &outCounts);

// Read channel voltage at ADC pin (after voltage divider)
inline bool readAds1115ChannelVoltageAtAdcPin(uint8_t channel, float &outVoltageAtAdcPin);

// Convert ADC-pin voltage to sensor output voltage (undo divider)
inline float convertAdcPinVoltageToSensorVoltage(float adcPinVoltage);

// ===================== PRESSURE CONVERSION =====================
inline float clampFloat(float value, float minValue, float maxValue);

inline float convertSensorVoltageToPsiCalibrated(
    float sensorVoltageV,
    float voltageAtFullScalePsi,
    float fullScalePsi,
    float zeroCalVoltageV);

inline float applyPsiDeadband(float psi, float deadbandPsi = PSI_DEADBAND);

// ===================== SENSOR-SPECIFIC READERS =====================
// A0: TDH33 (0-1000 PSI, 0-5V)
inline bool readPressureSensorA0Psi(float &outPressurePsi);

// A1: TD1000 (0-600 PSI, 1-5V)
inline bool readPressureSensorA1Psi(float &outPressurePsi);

// ===================== PRINTING =====================
inline void printPressureOrFault(const char *label, float pressurePsi);

#endif
