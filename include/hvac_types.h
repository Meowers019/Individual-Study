#ifndef HVAC_TYPES_H
#define HVAC_TYPES_H

#include <stdint.h>

// ===================== HVAC ENUMS =====================
enum class HvacMode : uint8_t {
  Off = 0,
  Cooling,
  Heating
};

enum class DiagnosticState : uint8_t {
  Normal = 0,
  SensorFault,
  WeakPerformance,
  Unknown
};

enum class HvacSystemState : uint8_t {
  Off = 0,
  Running
};

enum class HvacSubstate : uint8_t {
  None = 0,
  Unknown
};

// ===================== FAULT CODES =====================
enum class FaultCode : uint16_t {
  None = 0,

  // Sensor / compute validity
  Fault_MissingTemp,
  Fault_MissingPressure,
  Fault_MissingSatTemp,
  Fault_MissingShSc,

  // Pressure-based
  Fault_StaticNotEqualizing,
  Fault_HighPsiTooHigh,
  Fault_LowPsiTooLow,

  // Line-temp based
  Fault_SuctionLineFreezing,
  Fault_LiquidLineTooHot,

  // Delta-T based
  Fault_DeltaTTooLow,
  Fault_DeltaTTooHigh,
  Fault_DeltaTZero,

  // SH/SC based
  Fault_SubcoolLow,
  Fault_SubcoolHigh,
  Fault_SuperheatLow,
  Fault_SuperheatHigh,

  // Cross-check combos
  Fault_LowChargePattern,
  Fault_RestrictionPattern
};

// ===================== STRUCTURES =====================
struct HvacState {
  HvacSystemState systemState;
  HvacMode mode;
  HvacSubstate substate;
  DiagnosticState diagnostic;
  uint32_t lastModeChangeMs;
};

struct HvacTemperatures {
  float highPressureLineTempC;  // Liquid line (condenser outlet) in °C
  float lowPressureLineTempC;   // Suction line (evaporator outlet) in °C
  float supplyAirTempC;         // Supply duct air temp in °C
  float returnAirTempC;         // Return duct air temp in °C
  float deltaTempC;             // Return - Supply (°C)
  uint32_t updatedAtMs;
};

struct HvacPressures {
  float lowSidePressurePsi;     // Low side pressure in PSI
  float highSidePressurePsi;    // High side pressure in PSI
  uint32_t updatedAtMs;
};

struct HvacSaturationTemps {
  float lowSideSatTempF;        // Saturation temp from low side pressure (°F)
  float highSideSatTempF;       // Saturation temp from high side pressure (°F)
  uint32_t updatedAtMs;
};

struct HvacShSc {
  float superheatF;             // Superheat: suction temp - low side sat temp (°F)
  float subcoolingF;            // Subcooling: high side sat temp - liquid temp (°F)
  uint32_t updatedAtMs;
};

struct Max31855Reading {
  float thermocoupleTempC;      // NAN if fault
  float coldJunctionTempC;      // Usually valid even if TC fault
  bool hasFault;
  bool faultOpenCircuit;
  bool faultShortToGND;
  bool faultShortToVCC;
  uint32_t rawFrame32;
};

struct PressureToTempRangeF {
  float psiLow;
  float psiHigh;
  float satTempF;
};

static constexpr uint8_t MAX_ACTIVE_FAULTS = 16;

struct FaultReport {
  FaultCode codes[MAX_ACTIVE_FAULTS];
  uint8_t count;
  DiagnosticState diag;
};

// ===================== STRING HELPERS =====================
inline const char* hvacModeToString(HvacMode mode) {
  switch (mode) {
    case HvacMode::Off:     return "OFF";
    case HvacMode::Cooling: return "COOLING";
    case HvacMode::Heating: return "HEATING";
    default:                return "UNKNOWN";
  }
}

inline const char* diagToString(DiagnosticState diag) {
  switch (diag) {
    case DiagnosticState::Normal:          return "NORMAL";
    case DiagnosticState::SensorFault:     return "SENSOR_FAULT";
    case DiagnosticState::WeakPerformance: return "WEAK_PERFORMANCE";
    case DiagnosticState::Unknown:         return "UNKNOWN";
    default:                               return "UNKNOWN";
  }
}

inline const char* faultCodeToString(FaultCode f) {
  switch (f) {
    case FaultCode::None: return "NONE";

    case FaultCode::Fault_MissingTemp: return "FAULT_MISSING_TEMP";
    case FaultCode::Fault_MissingPressure: return "FAULT_MISSING_PRESSURE";
    case FaultCode::Fault_MissingSatTemp: return "FAULT_MISSING_SATTEMP";
    case FaultCode::Fault_MissingShSc: return "FAULT_MISSING_SHSC";

    case FaultCode::Fault_StaticNotEqualizing: return "FAULT_STATIC_NOT_EQUALIZING";
    case FaultCode::Fault_HighPsiTooHigh: return "FAULT_HIGH_PSI_TOO_HIGH";
    case FaultCode::Fault_LowPsiTooLow: return "FAULT_LOW_PSI_TOO_LOW";

    case FaultCode::Fault_SuctionLineFreezing: return "FAULT_SUCTION_LINE_FREEZING";
    case FaultCode::Fault_LiquidLineTooHot: return "FAULT_LIQUID_LINE_TOO_HOT";

    case FaultCode::Fault_DeltaTTooLow: return "FAULT_DELTAT_TOO_LOW";
    case FaultCode::Fault_DeltaTTooHigh: return "FAULT_DELTAT_TOO_HIGH";
    case FaultCode::Fault_DeltaTZero: return "FAULT_DELTAT_ZERO";

    case FaultCode::Fault_SubcoolLow: return "FAULT_SUBCOOL_LOW";
    case FaultCode::Fault_SubcoolHigh: return "FAULT_SUBCOOL_HIGH";
    case FaultCode::Fault_SuperheatLow: return "FAULT_SUPERHEAT_LOW";
    case FaultCode::Fault_SuperheatHigh: return "FAULT_SUPERHEAT_HIGH";

    case FaultCode::Fault_LowChargePattern: return "FAULT_LOW_CHARGE_PATTERN";
    case FaultCode::Fault_RestrictionPattern: return "FAULT_RESTRICTION_PATTERN";

    default: return "FAULT_UNKNOWN";
  }
}

#endif // HVAC_TYPES_H
