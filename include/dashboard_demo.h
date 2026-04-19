#ifndef DASHBOARD_DEMO_H
#define DASHBOARD_DEMO_H

// ===================== DASHBOARD DEMO =====================
// Simulates N independent HVAC units, each running a chosen
// test scenario with real-world sensor variance.
//
// Routes registered in network.cpp:
//   GET /dashboard               — full HTML dashboard
//   GET /dashboard/json          — JSON snapshot of all units
//   GET /dashboard/setscenario   — ?unit=N&scenario=S (0-based index)
//
// Call initDashboard() once in setup(), updateDashboard() each loop().

#include <Arduino.h>
#include <WiFi.h>
#include "hvac_types.h"
#include "hvac_config.h"
#include "hvac_test_data.h"    // addVariance(), TEST_SCENARIOS[]
#include "hvac_diagnostics.h"
#include "hvac_sensors.h"
#include "max31855_driver.h"   // celsiusToFahrenheit, deltaCelsiusToDeltaFahrenheit

// ===================== CONFIGURATION =====================

#define DASH_UNIT_COUNT 5

// Human-readable scenario names (must match TestScenario enum order)
static const char* DASH_SCENARIO_NAMES[] = {
  "Normal Cooling",
  "Normal Heating",
  "System Off",
  "Low Refrigerant",
  "Dirty Condenser",
  "Dirty Evaporator",
  "TXV Restriction",
  "Suction Freezing",
  "Static Not Equal.",
  "Weak Airflow",
  "Multiple Faults",
  "All Sensors Fault",
  "Custom"
};
static const uint8_t DASH_SCENARIO_COUNT = 13;

// ===================== PER-UNIT STATE =====================

struct DashUnit {
  const char*         name;
  TestScenario        scenario;
  HvacState           state;
  HvacTemperatures    temps;
  HvacPressures       pressures;
  HvacSaturationTemps satTemps;
  HvacShSc            shSc;
  FaultReport         faultReport;
};

static DashUnit dashUnits[DASH_UNIT_COUNT];

// ===================== INIT =====================

inline void dashInitInternal() {
  // Default unit names and spread of interesting starting scenarios
  static const char* names[DASH_UNIT_COUNT] = {
    "Unit 101 — embeddedSensor1",
    "Unit 202 — embeddedSensor2",
    "Unit 303 — embeddedSensor3",
    "Unit 404 — embeddedSensor4",
    "Unit 505 — embeddedSensor5"
  };
  static const TestScenario defaults[DASH_UNIT_COUNT] = {
    TestScenario::Normal_Cooling,
    TestScenario::Low_Refrigerant_Charge,
    TestScenario::Dirty_Condenser,
    TestScenario::Restriction_TXV,
    TestScenario::Suction_Line_Freezing
  };

  for (uint8_t i = 0; i < DASH_UNIT_COUNT; i++) {
    dashUnits[i].name     = names[i];
    dashUnits[i].scenario = defaults[i];
    dashUnits[i].state    = { HvacSystemState::Off, HvacMode::Off, HvacSubstate::None, DiagnosticState::Unknown, 0 };
    dashUnits[i].temps    = { NAN, NAN, NAN, NAN, NAN, 0 };
    dashUnits[i].pressures = { NAN, NAN, 0 };
    dashUnits[i].satTemps  = { NAN, NAN, 0 };
    dashUnits[i].shSc      = { NAN, NAN, 0 };
    initFaultReport(dashUnits[i].faultReport);
  }

  Serial.println("[Dashboard] Initialized " + String(DASH_UNIT_COUNT) + " demo units");
  Serial.println("[Dashboard] http://" + WiFi.localIP().toString() + "/dashboard");
}

// ===================== PER-UNIT SENSOR SIMULATION =====================

inline void applyScenarioToUnit(DashUnit& u) {
  const TestSensorData& d = TEST_SCENARIOS[(uint8_t)u.scenario];

  // Temperatures — fixed to scenario baseline (no variance)
  u.temps.highPressureLineTempC = d.highPressureLineTempC;
  u.temps.lowPressureLineTempC  = d.lowPressureLineTempC;
  u.temps.supplyAirTempC        = d.supplyAirTempC;
  u.temps.returnAirTempC        = d.returnAirTempC;

  if (!isnan(u.temps.returnAirTempC) && !isnan(u.temps.supplyAirTempC)) {
    u.temps.deltaTempC = u.temps.returnAirTempC - u.temps.supplyAirTempC;
  } else {
    u.temps.deltaTempC = NAN;
  }
  u.temps.updatedAtMs = millis();

  // Apply variance to pressures
  u.pressures.lowSidePressurePsi  = addVariance(d.lowSidePressurePsi);
  u.pressures.highSidePressurePsi = addVariance(d.highSidePressurePsi);
  u.pressures.updatedAtMs = millis();
}

// ===================== PER-UNIT DIAGNOSTICS =====================

inline void runUnitDiagnostics(DashUnit& u) {
  u.state.diagnostic = DiagnosticState::Unknown;
  updateHvacRunStateFromDeltaTOrDeltaPsi(u.temps, u.pressures, u.state);
  updateHvacModeFromTemperatures(u.temps, u.state);
  updateHvacSaturationTempsFromPressures(u.pressures, u.satTemps);
  updateHvacSuperheatSubcool(u.temps, u.satTemps, u.shSc);

  u.faultReport = evaluateFaultsHeatPumpAll(
    u.temps, u.pressures, u.satTemps, u.shSc,
    u.state.systemState, u.state.mode
  );

  if (u.state.diagnostic != DiagnosticState::SensorFault) {
    u.state.diagnostic = u.faultReport.diag;
  }
}

// ===================== UPDATE ALL UNITS =====================

inline void dashUpdateInternal() {
  for (uint8_t i = 0; i < DASH_UNIT_COUNT; i++) {
    applyScenarioToUnit(dashUnits[i]);
    runUnitDiagnostics(dashUnits[i]);
  }
}

// ===================== SCENARIO CONTROL =====================

inline void setDashUnitScenario(uint8_t idx, uint8_t scenarioIdx) {
  if (idx < DASH_UNIT_COUNT && scenarioIdx < DASH_SCENARIO_COUNT) {
    dashUnits[idx].scenario = (TestScenario)scenarioIdx;
    Serial.printf("[Dashboard] Unit %u scenario -> %s\n", idx, DASH_SCENARIO_NAMES[scenarioIdx]);
  }
}

// ===================== HTML HELPERS =====================

static String dashFmtF(float tempC) {
  if (isnan(tempC)) return "<span style='color:#dc3545'>N/A</span>";
  return String(celsiusToFahrenheit(tempC), 1) + "&deg;F";
}

static String dashFmtPsi(float psi) {
  if (isnan(psi)) return "<span style='color:#dc3545'>N/A</span>";
  return String(psi, 1) + " PSI";
}

static String dashFmtDegF(float f) {
  if (isnan(f)) return "<span style='color:#dc3545'>N/A</span>";
  return String(f, 1) + "&deg;F";
}

static String dashDiagColor(DiagnosticState d) {
  switch (d) {
    case DiagnosticState::Normal:      return "#28a745";
    case DiagnosticState::SensorFault: return "#dc3545";
    case DiagnosticState::Unknown:     return "#6c757d";
    default:                           return "#e07b00"; // warning/degraded
  }
}

static String dashStateColor(HvacSystemState s) {
  return s == HvacSystemState::Running ? "#28a745" : "#6c757d";
}

// ===================== UNIT CARD HTML =====================

static String buildUnitCard(uint8_t idx) {
  const DashUnit& u = dashUnits[idx];
  bool hasFaults    = u.faultReport.count > 0;
  String accent     = hasFaults ? "#dc3545" : "#28a745";

  String h = "";

  // Card wrapper
  h += "<div class='card' style='border-left:5px solid " + accent + "'>";

  // Card header: unit name left, scenario badge right
  h += "<div class='card-header'>";
  h += "<span class='unit-name'>" + String(u.name) + "</span>";
  h += "<span class='scenario-badge'>" + String(DASH_SCENARIO_NAMES[(uint8_t)u.scenario]) + "</span>";
  h += "</div>";

  // State / Mode / Diagnostic badges
  h += "<div class='status-row'>";
  h += "<span class='badge' style='background:" + dashStateColor(u.state.systemState) + "'>";
  h += String(u.state.systemState == HvacSystemState::Running ? "RUNNING" : "OFF") + "</span>";
  h += "<span class='badge' style='background:#17a2b8'>" + String(hvacModeToString(u.state.mode)) + "</span>";
  h += "<span class='badge' style='background:" + dashDiagColor(u.state.diagnostic) + "'>";
  h += String(diagToString(u.state.diagnostic)) + "</span>";
  h += "</div>";

  // Fault list or OK indicator
  h += "<div class='fault-row'>";
  if (hasFaults) {
    for (uint8_t f = 0; f < u.faultReport.count; f++) {
      h += "<span class='fault-badge'>" + String(faultCodeToString(u.faultReport.codes[f])) + "</span>";
    }
  } else {
    h += "<span class='ok-badge'>&#10003; No Active Faults</span>";
  }
  h += "</div>";

  // Scenario selector
  h += "<div class='scenario-row'>";
  h += "<label>Load scenario:</label>";
  h += "<select class='scenario-select' onchange=\"location='/dashboard/setscenario?unit=" + String(idx) + "&amp;scenario='+this.value\">";
  for (uint8_t s = 0; s < DASH_SCENARIO_COUNT; s++) {
    h += "<option value='" + String(s) + "'";
    if (s == (uint8_t)u.scenario) h += " selected";
    h += ">" + String(DASH_SCENARIO_NAMES[s]) + "</option>";
  }
  h += "</select>";
  h += "</div>";

  // Collapsible sensor data (native HTML details/summary — no JS required)
  h += "<details class='sensor-details'>";
  h += "<summary>&#9654; View Sensor Data</summary>";
  h += "<div class='sensor-grid'>";

  // --- Temperatures ---
  h += "<div class='sensor-section'>";
  h += "<div class='sensor-section-title'>Temperatures</div>";
  h += "<div class='sensor-row'><span>HP Line</span><span>" + dashFmtF(u.temps.highPressureLineTempC) + "</span></div>";
  h += "<div class='sensor-row'><span>LP Line</span><span>" + dashFmtF(u.temps.lowPressureLineTempC) + "</span></div>";
  h += "<div class='sensor-row'><span>Supply Air</span><span>" + dashFmtF(u.temps.supplyAirTempC) + "</span></div>";
  h += "<div class='sensor-row'><span>Return Air</span><span>" + dashFmtF(u.temps.returnAirTempC) + "</span></div>";

  float absDeltaF = isnan(u.temps.deltaTempC) ? NAN : fabsf(deltaCelsiusToDeltaFahrenheit(u.temps.deltaTempC));
  h += "<div class='sensor-row'><span>Delta-T</span><span>" + dashFmtDegF(absDeltaF) + "</span></div>";
  h += "</div>";

  // --- Pressures ---
  h += "<div class='sensor-section'>";
  h += "<div class='sensor-section-title'>Pressures</div>";
  h += "<div class='sensor-row'><span>Low Side</span><span>"  + dashFmtPsi(u.pressures.lowSidePressurePsi)  + "</span></div>";
  h += "<div class='sensor-row'><span>High Side</span><span>" + dashFmtPsi(u.pressures.highSidePressurePsi) + "</span></div>";
  float dp = computeDeltaPsi(u.pressures);
  h += "<div class='sensor-row'><span>Delta PSI</span><span>" + dashFmtPsi(dp) + "</span></div>";
  h += "</div>";

  // --- Performance (Sat Temps, SH/SC) ---
  h += "<div class='sensor-section'>";
  h += "<div class='sensor-section-title'>Performance</div>";
  h += "<div class='sensor-row'><span>Low Sat Temp</span><span>"  + dashFmtDegF(u.satTemps.lowSideSatTempF)  + "</span></div>";
  h += "<div class='sensor-row'><span>High Sat Temp</span><span>" + dashFmtDegF(u.satTemps.highSideSatTempF) + "</span></div>";
  h += "<div class='sensor-row'><span>Superheat</span><span>"     + dashFmtDegF(u.shSc.superheatF)           + "</span></div>";
  h += "<div class='sensor-row'><span>Subcooling</span><span>"    + dashFmtDegF(u.shSc.subcoolingF)          + "</span></div>";
  h += "</div>";

  h += "</div>"; // sensor-grid
  h += "</details>";
  h += "</div>"; // card
  return h;
}

// ===================== FULL DASHBOARD HTML =====================

inline String getDashboardHtml() {
  String ip   = WiFi.localIP().toString();
  uint32_t up = millis() / 1000;

  String html = "<!DOCTYPE html><html><head>";
  html += "<meta charset='UTF-8'>";
  html += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<title>HVAC Dashboard Demo</title>";
  html += "<meta http-equiv='refresh' content='5'>";

  // ---- CSS ----
  html += "<style>"
    "body{font-family:'Segoe UI',Arial,sans-serif;margin:0;background:#f0f4f8;color:#222}"

    // Header
    ".header{background:#1e3a5f;color:white;padding:16px 24px;"
      "display:flex;justify-content:space-between;align-items:center;flex-wrap:wrap;gap:8px}"
    ".header h1{margin:0;font-size:21px;font-weight:700;letter-spacing:.3px}"
    ".header .meta{font-size:12px;color:#a8c0d6;text-align:right;line-height:1.6}"

    // Unit grid
    ".grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(330px,1fr));"
      "gap:16px;padding:20px;max-width:1400px;margin:0 auto}"

    // Card
    ".card{background:white;border-radius:10px;padding:16px;"
      "box-shadow:0 2px 8px rgba(0,0,0,.08)}"
    ".card-header{display:flex;justify-content:space-between;align-items:flex-start;margin-bottom:10px}"
    ".unit-name{font-weight:700;font-size:15px;color:#1e3a5f;line-height:1.3}"
    ".scenario-badge{font-size:10px;background:#e8f0fe;color:#1e3a5f;border-radius:4px;"
      "padding:3px 7px;font-weight:600;white-space:nowrap;margin-left:8px;flex-shrink:0}"

    // Status badges
    ".status-row{display:flex;gap:5px;flex-wrap:wrap;margin-bottom:8px}"
    ".badge{font-size:11px;color:white;border-radius:4px;padding:2px 7px;font-weight:700;letter-spacing:.3px}"

    // Fault row
    ".fault-row{margin-bottom:9px;min-height:22px;display:flex;flex-wrap:wrap;gap:4px;align-items:center}"
    ".fault-badge{font-size:10px;background:#dc3545;color:white;border-radius:4px;"
      "padding:2px 7px;font-weight:600}"
    ".ok-badge{font-size:11px;background:#e7f5ec;color:#28a745;border-radius:4px;"
      "padding:2px 8px;font-weight:700}"

    // Scenario selector
    ".scenario-row{margin-bottom:10px;font-size:12px;display:flex;align-items:center;gap:6px}"
    ".scenario-row label{color:#666;white-space:nowrap}"
    ".scenario-select{font-size:12px;padding:3px 6px;border-radius:4px;border:1px solid #ccc;"
      "background:white;cursor:pointer;flex:1;min-width:0}"

    // Sensor details dropdown
    "details.sensor-details{border-top:1px solid #eee;padding-top:8px;margin-top:2px}"
    "details summary{cursor:pointer;color:#1e3a5f;font-weight:600;font-size:12px;"
      "list-style:none;outline:none;user-select:none}"
    "details summary::-webkit-details-marker{display:none}"
    ".sensor-grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(150px,1fr));gap:10px;margin-top:10px}"
    ".sensor-section{font-size:12px}"
    ".sensor-section-title{font-weight:700;color:#888;margin-bottom:4px;font-size:10px;"
      "text-transform:uppercase;letter-spacing:.6px}"
    ".sensor-row{display:flex;justify-content:space-between;padding:2px 0;"
      "border-bottom:1px dotted #f0f0f0}"
    ".sensor-row span:last-child{font-weight:600;color:#1e3a5f}"

    // Footer
    ".footer{text-align:center;padding:16px;font-size:11px;color:#aaa}"
    "</style></head><body>";

  // ---- Header ----
  html += "<div class='header'>";
  html += "<div>";
  html += "<h1>&#128295; HVAC Dashboard Demo</h1>";
  html += "<div style='font-size:12px;color:#a8c0d6;margin-top:3px'>";
  html += "Scalable Monitoring Prototype &nbsp;|&nbsp; ";
  html += String(DASH_UNIT_COUNT) + " Units Online";
  html += "</div></div>";
  html += "<div class='meta'>";
  html += "Device IP: <b style='color:white'>" + ip + "</b><br>";
  html += "/dashboard &nbsp;|&nbsp; Uptime: " + String(up) + "s &nbsp;|&nbsp; Auto-refresh: 5s";
  html += "</div></div>";

  // ---- Unit Cards ----
  html += "<div class='grid'>";
  for (uint8_t i = 0; i < DASH_UNIT_COUNT; i++) {
    html += buildUnitCard(i);
  }
  html += "</div>";

  // ---- Footer ----
  html += "<div class='footer'>";
  html += "HVAC Monitoring Prototype &mdash; Individual Study &nbsp;|&nbsp; ";
  html += "Test data with real-world &#177;3&ndash;5 unit variance &nbsp;|&nbsp; ";
  html += "Routes: /dashboard &nbsp;/dashboard/json &nbsp;/dashboard/setscenario";
  html += "</div></body></html>";

  return html;
}

// ===================== DASHBOARD JSON =====================

inline String getDashboardJson() {
  String j = "{\"unitCount\":" + String(DASH_UNIT_COUNT) + ",\"units\":[";

  for (uint8_t i = 0; i < DASH_UNIT_COUNT; i++) {
    const DashUnit& u = dashUnits[i];
    if (i > 0) j += ",";

    j += "{";
    j += "\"id\":"         + String(i) + ",";
    j += "\"name\":\""     + String(u.name) + "\",";
    j += "\"scenario\":\""  + String(DASH_SCENARIO_NAMES[(uint8_t)u.scenario]) + "\",";
    j += "\"state\":\""    + String(u.state.systemState == HvacSystemState::Running ? "Running" : "Off") + "\",";
    j += "\"mode\":\""     + String(hvacModeToString(u.state.mode)) + "\",";
    j += "\"diagnostic\":\"" + String(diagToString(u.state.diagnostic)) + "\",";

    auto fj = [](float v, int d = 1) -> String {
      return isnan(v) ? "null" : String(v, d);
    };

    j += "\"temps\":{";
    j += "\"highPressureLineF\":" + fj(celsiusToFahrenheit(u.temps.highPressureLineTempC)) + ",";
    j += "\"lowPressureLineF\":"  + fj(celsiusToFahrenheit(u.temps.lowPressureLineTempC))  + ",";
    j += "\"supplyAirF\":"        + fj(celsiusToFahrenheit(u.temps.supplyAirTempC))        + ",";
    j += "\"returnAirF\":"        + fj(celsiusToFahrenheit(u.temps.returnAirTempC))        + "},";

    j += "\"pressures\":{";
    j += "\"lowSidePsi\":"  + fj(u.pressures.lowSidePressurePsi)  + ",";
    j += "\"highSidePsi\":" + fj(u.pressures.highSidePressurePsi) + "},";

    j += "\"performance\":{";
    j += "\"superheatF\":"  + fj(u.shSc.superheatF)  + ",";
    j += "\"subcoolingF\":" + fj(u.shSc.subcoolingF) + "},";

    j += "\"faults\":{\"count\":" + String(u.faultReport.count) + ",\"codes\":[";
    for (uint8_t f = 0; f < u.faultReport.count; f++) {
      if (f > 0) j += ",";
      j += "\"" + String(faultCodeToString(u.faultReport.codes[f])) + "\"";
    }
    j += "]}}";
  }

  j += "]}";
  return j;
}

#endif // DASHBOARD_DEMO_H
