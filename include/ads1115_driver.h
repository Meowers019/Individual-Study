#pragma once
#include <Adafruit_ADS1X15.h>
#include <Arduino.h>

#include "hvac_config.h"
#include "utils.h"

// ===================== I2C HELPERS =====================
class ADS1115 {
private:
  Adafruit_ADS1115 ads_;

  ADS1115() {}

public:
  static ADS1115 &instance() {
    static ADS1115 instance;
    return instance;
  }

  void begin() {
    if (!ads_.begin()) {
      Serial.println("Failed to initialize ADS.");
      while (1)
        ;
    }
    ads_.setGain(GAIN_ONE);

    Serial.println("ADS1115 initialized");

    // Mux
    ads_.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1, false);
  }

  bool readRaw(uint8_t channel, float &outVolts) {
    if (!ads_.conversionComplete()) {
      outVolts = NAN;
      return false;
    }
    int16_t results = ads_.getLastConversionResults();
    outVolts = ads_.computeVolts(results);
    return true;
  }

  bool readVolts(uint8_t ch, float &outVolts) {
    bool s = readRaw(ch, outVolts);
    if (!s)
      return false;
    outVolts = ads_.computeVolts(outVolts);
    return true;
  }

  float rawToVolt(int16_t raw) { return ads_.computeVolts(raw); }
};

// ===================== PRESSURE CONVERSION =====================
inline float convertSensorVoltageToPsiCalibrated(float sensorVoltageV,
                                                 Psi_Sensor s) {
  if (isnan(sensorVoltageV))
    return NAN;

  const float spanV = s.voltFull - s.voltZero;
  if (spanV <= 0.0f || s.fullScale <= 0.0f)
    return NAN;

  // Clamp to valid range
  float v = clampFloat(sensorVoltageV, s.voltZero, s.voltFull);

  float fraction = (v - s.voltZero) / spanV;
  float psi = fraction * s.fullScale;

  // Suppress noise near zero
  if (psi < 0.0f)
    psi = 0.0f;
  return psi;
}

inline float applyPsiDeadband(float psi, float deadbandPsi = PSI_DEADBAND) {
  if (isnan(psi))
    return NAN;
  return (psi < deadbandPsi) ? 0.0f : psi;
}

// ===================== SENSOR-SPECIFIC READERS =====================
// A0: TDH33 (0-1000 PSI, 0-5V)
inline bool readPressureSensorPsi(Psi_Sensor s, float &outPressurePsi) {
  float adcPinVoltageV = NAN;

  ADS1115 &ads = ADS1115::instance();

  if (!ads.readRaw(s.channel, adcPinVoltageV)) {
    outPressurePsi = NAN;
    return false;
  }

  const float sensorVoltageV = ads.rawToVolt(adcPinVoltageV);

  outPressurePsi = convertSensorVoltageToPsiCalibrated(sensorVoltageV, s);

  outPressurePsi = applyPsiDeadband(outPressurePsi);
  return true;
}

inline bool readPressureSensorA0Psi(float &outPressurePsi) {
  return readPressureSensorPsi(psiA0, outPressurePsi);
}

inline bool readPressureSensorA1Psi(float &outPressurePsi) {
  return readPressureSensorPsi(psiA1, outPressurePsi);
}

// ===================== PRINTING =====================
inline void printPressureOrFault(const char *label, float pressurePsi) {
  if (isnan(pressurePsi)) {
    Serial.printf("%s: FAULT\n", label);
  } else {
    Serial.printf("%s: %.1f PSI\n", label, pressurePsi);
  }
}
