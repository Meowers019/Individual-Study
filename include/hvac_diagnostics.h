#ifndef HVAC_DIAGNOSTICS_H
#define HVAC_DIAGNOSTICS_H

#include "hvac_types.h"
#include "hvac_config.h"
#include "max31855_driver.h"

// ===================== FAULT REPORT MANAGEMENT =====================

inline void initFaultReport(FaultReport &r) {
  r.count = 0;
  r.diag = DiagnosticState::Normal;
}

inline void bumpDiag(FaultReport &r, DiagnosticState d) {
  // Diagnostic Priority Hierarchy (highest to lowest):
  // 1. SensorFault - sensor issues
  // 2. EvapOff, CompOff - component mismatch (hardware issues)
  // 3. LowRefrigerant, Restriction - fundamental system issues
  // 4. StaticNotEqualizing - system off but not equalizing
  // 5. WeakAirflow - symptom (could be caused by other issues)
  // 6. Poor - generic poor performance
  // 7. WeakPerformance - minor issues
  // 8. Normal
  
  // Assign numeric priority for comparison (lower number = higher priority)
  auto getPriority = [](DiagnosticState state) -> int {
    switch (state) {
      case DiagnosticState::SensorFault:          return 1;
      case DiagnosticState::EvapOff:              return 2;
      case DiagnosticState::CompOff:              return 2;
      case DiagnosticState::LowRefrigerant:       return 3;
      case DiagnosticState::Restriction:          return 3;
      case DiagnosticState::StaticNotEqualizing:  return 4;
      case DiagnosticState::WeakAirflow:          return 5;
      case DiagnosticState::Poor:                 return 6;
      case DiagnosticState::WeakPerformance:      return 7;
      case DiagnosticState::Normal:               return 8;
      default:                                    return 9;
    }
  };
  
  // Only update if new diagnostic has higher priority (lower number)
  if (getPriority(d) < getPriority(r.diag)) {
    r.diag = d;
  }
}

inline bool reportHasFault(const FaultReport &r, FaultCode code) {
  for (uint8_t i = 0; i < r.count; i++) {
    if (r.codes[i] == code) return true;
  }
  return false;
}

inline void addFault(FaultReport &r, FaultCode code, DiagnosticState severity) {
  if (code == FaultCode::None) return;

  // Avoid duplicates
  if (reportHasFault(r, code)) {
    bumpDiag(r, severity);
    return;
  }

  // Append if room
  if (r.count < MAX_ACTIVE_FAULTS) {
    r.codes[r.count++] = code;
  }
  
  // Even if full, still bump diagnostic level
  bumpDiag(r, severity);
}

// ===================== FAULT EVALUATION =====================

inline FaultReport evaluateFaultsHeatPumpAll(
  const HvacTemperatures &t,
  const HvacPressures &p,
  const HvacSaturationTemps &sat,
  const HvacShSc &shsc,
  HvacSystemState sys,
  HvacMode mode
) {
  FaultReport r;
  initFaultReport(r);

  auto ok = [](float v) { return !isnan(v); };

  // ---------- OFF-STATE EQUALIZATION CHECK ----------
  if (sys == HvacSystemState::Off) {
    // If unit is truly off, high/low should drift toward equal over time
    // If deltaPsi stays high, suggests restriction/stuck TXV/valves not bypassing
    if (ok(p.lowSidePressurePsi) && ok(p.highSidePressurePsi)) {
      const float dpsi = p.highSidePressurePsi - p.lowSidePressurePsi;
      if (!isnan(dpsi) && fabsf(dpsi) > STATIC_EQUALIZE_DELTA_PSI_MAX) {
        addFault(r, FaultCode::Fault_StaticNotEqualizing, DiagnosticState::StaticNotEqualizing);
      }
    }
    return r;  // Skip running-state checks
  }

  // Convert needed temps to F for comparisons
  const float suctionF = celsiusToFahrenheit(t.lowPressureLineTempC);
  const float liquidF  = celsiusToFahrenheit(t.highPressureLineTempC);
  const float deltaTF  = deltaCelsiusToDeltaFahrenheit(t.deltaTempC);

  // ---------- DATA VALIDITY (add faults, don't return) ----------
  if (!ok(t.supplyAirTempC) || !ok(t.returnAirTempC) || isnan(t.deltaTempC)) {
    addFault(r, FaultCode::Fault_MissingTemp, DiagnosticState::SensorFault);
  }
  if (!ok(p.lowSidePressurePsi) || !ok(p.highSidePressurePsi)) {
    addFault(r, FaultCode::Fault_MissingPressure, DiagnosticState::SensorFault);
  }
  if (!ok(sat.lowSideSatTempF) || !ok(sat.highSideSatTempF)) {
    addFault(r, FaultCode::Fault_MissingSatTemp, DiagnosticState::SensorFault);
  }
  if (!ok(shsc.superheatF) || !ok(shsc.subcoolingF)) {
    addFault(r, FaultCode::Fault_MissingShSc, DiagnosticState::SensorFault);
  }

  // ---------- RUNNING STATE CHECKS ----------
  // If we're missing data, we've already flagged it above
  const bool tempsOk = ok(t.supplyAirTempC) && ok(t.returnAirTempC) && ok(deltaTF);
  const bool pressuresOk = ok(p.lowSidePressurePsi) && ok(p.highSidePressurePsi);
  const bool satOk = ok(sat.lowSideSatTempF) && ok(sat.highSideSatTempF);
  const bool shscOk = ok(shsc.superheatF) && ok(shsc.subcoolingF);

  // ---------- DELTA-T ----------
  if (tempsOk) {
    const float absDeltaTF = fabsf(deltaTF);
    if (absDeltaTF <= DELTAT_ZERO_F) {
      addFault(r, FaultCode::Fault_DeltaTZero, DiagnosticState::WeakPerformance);
    }
    if (absDeltaTF < DELTAT_LOW_F) {
      addFault(r, FaultCode::Fault_DeltaTTooLow, DiagnosticState::WeakPerformance);
    }
    if (absDeltaTF > DELTAT_HIGH_F) {
      addFault(r, FaultCode::Fault_DeltaTTooHigh, DiagnosticState::WeakPerformance);
    }
  }

  // ---------- PRESSURES ----------
  if (pressuresOk && mode == HvacMode::Cooling) {
    if (p.highSidePressurePsi > PSI_HIGH_MAX_COOL) {
      addFault(r, FaultCode::Fault_HighPsiTooHigh, DiagnosticState::WeakPerformance);
    }
    if (p.lowSidePressurePsi < PSI_LOW_MIN_COOL) {
      addFault(r, FaultCode::Fault_LowPsiTooLow, DiagnosticState::WeakPerformance);
    }
  }

  // ---------- LINE TEMPS ----------
  if (ok(suctionF) && suctionF <= SUCTION_FREEZE_F) {
    addFault(r, FaultCode::Fault_SuctionLineFreezing, DiagnosticState::WeakPerformance);
  }
  if (ok(liquidF) && liquidF >= LIQUID_TOO_HOT_F) {
    addFault(r, FaultCode::Fault_LiquidLineTooHot, DiagnosticState::WeakPerformance);
  }

  // ---------- SH / SC ----------
  if (shscOk) {
    if (shsc.subcoolingF < SUBCOOL_LOW_F) {
      addFault(r, FaultCode::Fault_SubcoolLow, DiagnosticState::WeakPerformance);
    }
    if (shsc.subcoolingF > SUBCOOL_HIGH_F) {
      addFault(r, FaultCode::Fault_SubcoolHigh, DiagnosticState::WeakPerformance);
    }
    if (shsc.superheatF < SUPERHEAT_LOW_F) {
      addFault(r, FaultCode::Fault_SuperheatLow, DiagnosticState::WeakPerformance);
    }
    if (shsc.superheatF > SUPERHEAT_HIGH_F) {
      addFault(r, FaultCode::Fault_SuperheatHigh, DiagnosticState::WeakPerformance);
    }
  }

  // ---------- COMBO PATTERNS ----------
  if (shscOk) {
    // Low charge: low subcool + high superheat
    if (shsc.subcoolingF < SUBCOOL_LOW_F && shsc.superheatF > SUPERHEAT_HIGH_F) {
      addFault(r, FaultCode::Fault_LowChargePattern, DiagnosticState::LowRefrigerant);
    }
    // Restriction: high subcool + high superheat
    if (shsc.subcoolingF > SUBCOOL_HIGH_F && shsc.superheatF > SUPERHEAT_HIGH_F) {
      addFault(r, FaultCode::Fault_RestrictionPattern, DiagnosticState::Restriction);
    }
  }

  // Weak airflow: low deltaT + high superheat (evaporator not absorbing heat)
  if (tempsOk && shscOk) {
    const float absDeltaTF = fabsf(deltaTF);
    // DeltaT is low (but not zero) and superheat is high
    if (absDeltaTF > DELTAT_ZERO_F && absDeltaTF < DELTAT_LOW_F && shsc.superheatF > SUPERHEAT_HIGH_F) {
      addFault(r, FaultCode::Fault_WeakAirflowPattern, DiagnosticState::WeakAirflow);
    }
  }

  return r;
}

// ===================== PRINTING =====================

inline void printFaultReportLines(const FaultReport &r) {
  if (r.count == 0) {
    Serial.println("FAULTS: NONE");
    return;
  }
  Serial.printf("FAULTS (%u):\n", (unsigned)r.count);
  for (uint8_t i = 0; i < r.count; i++) {
    Serial.printf(" - %s\n", faultCodeToString(r.codes[i]));
  }
}

#endif // HVAC_DIAGNOSTICS_H
