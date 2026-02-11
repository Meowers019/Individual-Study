#ifndef HVAC_SENSORS_H
#define HVAC_SENSORS_H

#include "hvac_types.h"
#include "hvac_config.h"
#include "max31855_driver.h"
#include "ads1115_driver.h"
#include "r22_tables.h"

// ===================== HVAC SENSOR MANAGEMENT =====================

// ===================== PRESSURE UPDATES =====================
inline void updateHvacPressures(HvacPressures &hvacPressures) {
  hvacPressures.lowSidePressurePsi  = NAN;
  hvacPressures.highSidePressurePsi = NAN;

  bool a0Ok = readPressureSensorA0Psi(hvacPressures.lowSidePressurePsi);
  bool a1Ok = readPressureSensorA1Psi(hvacPressures.highSidePressurePsi);

  if (a0Ok || a1Ok) {
    hvacPressures.updatedAtMs = millis();
  }
}

// ===================== SATURATION TEMPERATURE UPDATES =====================
inline void updateHvacSaturationTempsFromPressures(
  const HvacPressures &hvacPressures,
  HvacSaturationTemps &hvacSatTemps
) {
  hvacSatTemps.lowSideSatTempF  = r22PsiToSaturationTempF(hvacPressures.lowSidePressurePsi);
  hvacSatTemps.highSideSatTempF = r22PsiToSaturationTempF(hvacPressures.highSidePressurePsi);
  hvacSatTemps.updatedAtMs = millis();
}

// ===================== SUPERHEAT / SUBCOOLING UPDATES =====================
inline void updateHvacSuperheatSubcool(
  const HvacTemperatures &hvacTemps,
  const HvacSaturationTemps &hvacSatTemps,
  HvacShSc &hvacShSc
) {
  // Convert line temps to Fahrenheit
  const float suctionLineTempF = celsiusToFahrenheit(hvacTemps.lowPressureLineTempC);
  const float liquidLineTempF  = celsiusToFahrenheit(hvacTemps.highPressureLineTempC);

  hvacShSc.superheatF  = computeSuperheatF(suctionLineTempF, hvacSatTemps.lowSideSatTempF);
  hvacShSc.subcoolingF = computeSubcoolingF(hvacSatTemps.highSideSatTempF, liquidLineTempF);

  hvacShSc.updatedAtMs = millis();
}

// ===================== MODE DETECTION =====================
inline float absf_safe(float v) { 
  return (v < 0.0f) ? -v : v; 
}

inline bool okf(float v) { 
  return !isnan(v); 
}

inline bool isValidTemp(float t) { 
  return !isnan(t); 
}

inline void updateHvacModeFromTemperatures(
  const HvacTemperatures &temps, 
  HvacState &state
) {
  static int coolingConfirmCount = 0;
  static int heatingConfirmCount = 0;
  static int offConfirmCount     = 0;

  // If system isn't running, mode is OFF
  if (state.systemState != HvacSystemState::Running) {
    state.mode = HvacMode::Off;
    return;
  }

  // Validate temperature data
  if (!isValidTemp(temps.supplyAirTempC) || 
      !isValidTemp(temps.returnAirTempC) || 
      isnan(temps.deltaTempC)) {
    state.diagnostic = DiagnosticState::SensorFault;
    return;
  }

  // Detect mode based on deltas and duct temps
  const bool looksOffByDelta     = (fabsf(temps.deltaTempC) <= DELTA_OFF_BAND_C);

  const bool looksCoolingByDelta = (temps.deltaTempC >= DELTA_COOL_ON_C);
  const bool looksCoolingByDuct  = (temps.supplyAirTempC <= SUPPLY_COOL_MAX_C);

  const bool looksHeatingByDelta = (temps.deltaTempC <= DELTA_HEAT_ON_C);
  const bool looksHeatingByDuct  = (temps.supplyAirTempC >= SUPPLY_HEAT_MIN_C);

  const bool coolingEvidence = looksCoolingByDelta && looksCoolingByDuct;
  const bool heatingEvidence = looksHeatingByDelta && looksHeatingByDuct;
  const bool offEvidence     = looksOffByDelta;

  // Update counters
  coolingConfirmCount = coolingEvidence ? (coolingConfirmCount + 1) : 0;
  heatingConfirmCount = heatingEvidence ? (heatingConfirmCount + 1) : 0;
  offConfirmCount     = offEvidence     ? (offConfirmCount + 1)     : 0;

  // Decide mode with persistence (prevents flip-flop)
  HvacMode previousMode = state.mode;
  HvacMode newMode = previousMode;

  if (coolingConfirmCount >= MODE_CONFIRM_COUNT) {
    newMode = HvacMode::Cooling;
  } else if (heatingConfirmCount >= MODE_CONFIRM_COUNT) {
    newMode = HvacMode::Heating;
  } else if (offConfirmCount >= MODE_CONFIRM_COUNT) {
    newMode = HvacMode::Off;
  }

  if (newMode != previousMode) {
    state.mode = newMode;
    state.lastModeChangeMs = millis();
  }

  // Basic diagnostic
  state.diagnostic = DiagnosticState::Normal;
}

// ===================== RUNNING STATE DETECTION =====================
inline void updateHvacRunStateFromDeltaTOrDeltaPsi(
  const HvacTemperatures &t,
  const HvacPressures &p,
  HvacState &s
) {
  const bool deltaTOk = okf(t.deltaTempC);

  // deltaPsi = High - Low
  const float deltaPsi = (okf(p.highSidePressurePsi) && okf(p.lowSidePressurePsi))
    ? (p.highSidePressurePsi - p.lowSidePressurePsi)
    : NAN;

  const bool deltaPsiOk = okf(deltaPsi);

  // If neither metric exists, don't guess
  if (!deltaTOk && !deltaPsiOk) {
    s.diagnostic = DiagnosticState::SensorFault;
    return;
  }

  const bool runningByDeltaT   = deltaTOk   && (absf_safe(t.deltaTempC) >= RUNNING_DELTA_T_C_THRESHOLD);
  const bool runningByDeltaPsi = deltaPsiOk && (absf_safe(deltaPsi)     >= RUNNING_DELTA_PSI_THRESHOLD);

  const HvacSystemState newSys = (runningByDeltaT || runningByDeltaPsi)
    ? HvacSystemState::Running
    : HvacSystemState::Off;

  // Transition handling
  if (newSys != s.systemState) {
    s.systemState = newSys;
    s.lastModeChangeMs = millis();
  }

  // If system is OFF, force mode OFF
  if (s.systemState == HvacSystemState::Off) {
    s.mode = HvacMode::Off;
    s.substate = HvacSubstate::None;
  }

  // Don't overwrite diagnostic if already set to SensorFault
  if (s.diagnostic == DiagnosticState::Unknown) {
    s.diagnostic = DiagnosticState::Normal;
  }
}

// ===================== DELTA PSI HELPER =====================
inline float computeDeltaPsi(const HvacPressures &p) {
  if (isnan(p.highSidePressurePsi) || isnan(p.lowSidePressurePsi)) return NAN;
  return p.highSidePressurePsi - p.lowSidePressurePsi;
}

#endif // HVAC_SENSORS_H
