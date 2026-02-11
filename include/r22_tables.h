#ifndef R22_TABLES_H
#define R22_TABLES_H

#include "hvac_types.h"

// ===================== R-22 PRESSURE/TEMPERATURE TABLE =====================
// Standard R-22 saturation table: PSI -> Temperature (°F)
// Format: [psi_low_inclusive, psi_high_exclusive, sat_temp_F]
// Source: Standard R-22 pressure-temperature chart

static constexpr PressureToTempRangeF R22_PT_TABLE_F[] = {
  {0, 50, 0}, {50, 52, 1}, {52, 54, 3}, {54, 56, 4}, {56, 58, 6},
  {58, 60, 7}, {62, 64, 10}, {64, 66, 11}, {66, 68, 12}, {68, 70, 14},
  {70, 72, 15}, {72, 74, 16}, {74, 76, 17}, {76, 78, 18}, {78, 80, 20},
  {80, 85, 21}, {85, 90, 24}, {90, 95, 26}, {95, 100, 29}, {100, 105, 31},
  {105, 110, 34}, {110, 115, 36}, {115, 120, 38}, {120, 125, 41},
  {125, 130, 43}, {130, 135, 45}, {135, 140, 47}, {140, 145, 49},
  {145, 150, 51}, {150, 155, 53}, {155, 160, 54}, {160, 165, 56},
  {165, 170, 58}, {170, 175, 60}, {175, 180, 61}, {180, 185, 63},
  {185, 190, 65}, {190, 195, 66}, {195, 200, 68}, {200, 205, 69},
  {205, 210, 71}, {210, 220, 72}, {220, 230, 75}, {230, 240, 78},
  {240, 250, 81}, {250, 260, 84}, {260, 275, 86}, {275, 290, 90},
  {290, 305, 94}, {305, 320, 97}, {320, 335, 100}, {335, 350, 104},
  {350, 365, 107}, {365, 380, 110}, {380, 400, 113}, {400, 420, 116},
  {420, 440, 120}, {440, 460, 124}, {460, 480, 127}, {480, 500, 130},
  {500, 525, 134},
  {525, 10000, 1000}  // Catch-all for out-of-range
};

static constexpr size_t R22_PT_TABLE_COUNT = sizeof(R22_PT_TABLE_F) / sizeof(R22_PT_TABLE_F[0]);

// ===================== LOOKUP FUNCTIONS =====================
// Convert R-22 pressure to saturation temperature
inline float r22PsiToSaturationTempF(float psi) {
  if (isnan(psi) || psi < 0.0f) return NAN;

  // Linear search through table
  for (size_t i = 0; i < R22_PT_TABLE_COUNT; i++) {
    const auto &entry = R22_PT_TABLE_F[i];
    if (psi >= entry.psiLow && psi < entry.psiHigh) {
      return entry.satTempF;
    }
  }
  
  return NAN; // Should not reach due to catch-all entry
}

// ===================== SUPERHEAT / SUBCOOLING =====================
inline float computeSuperheatF(float suctionLineTempF, float lowSideSatTempF) {
  if (isnan(suctionLineTempF) || isnan(lowSideSatTempF)) return NAN;
  return suctionLineTempF - lowSideSatTempF;
}

inline float computeSubcoolingF(float highSideSatTempF, float liquidLineTempF) {
  if (isnan(highSideSatTempF) || isnan(liquidLineTempF)) return NAN;
  return highSideSatTempF - liquidLineTempF;
}

// ===================== PRINTING =====================
inline void printSatTempOrFault(const char* label, float tempF) {
  if (isnan(tempF)) {
    Serial.printf("%s: FAULT\n", label);
  } else {
    Serial.printf("%s: %.1f F\n", label, tempF);
  }
}

inline void printShScOrFault(const char* label, float valueF) {
  if (isnan(valueF)) {
    Serial.printf("%s: FAULT\n", label);
  } else {
    Serial.printf("%s: %.1f F\n", label, valueF);
  }
}

#endif // R22_TABLES_H
