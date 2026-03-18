#include <WiFi.h>
#include "network_setup.h"
#include <WebServer.h>
#include <HTTPClient.h>
#include <WiFiManager.h>
#include "hvac_types.h"
#include "hvac_config.h"
#include "hvac_diagnostics.h"
#include "hvac_sensors.h"

// Access global HVAC data from main.cpp
extern HvacState hvacState;
extern HvacTemperatures hvacTemps;
extern HvacPressures hvacPressures;
extern HvacSaturationTemps hvacSatTemps;
extern HvacShSc hvacShSc;
extern FaultReport faultReport;

WebServer server(80);

// Rate limiting for AWS sends
static uint32_t lastAwsSendMs = 0;

// ===================== JSON HELPERS =====================

// Returns "null" for NaN values, otherwise a formatted decimal number
static String floatToJson(float value, int decimals = 2) {
    if (isnan(value)) return "null";
    return String(value, decimals);
}

// Returns a JSON array of active fault code strings: ["FAULT_X","FAULT_Y"]
static String buildFaultCodesJson() {
    String s = "[";
    for (uint8_t i = 0; i < faultReport.count; i++) {
        if (i > 0) s += ",";
        s += "\"" + String(faultCodeToString(faultReport.codes[i])) + "\"";
    }
    s += "]";
    return s;
}

// Returns the "system" JSON object
static String buildSystemJson() {
    String s = "\"system\":{";
    s += "\"state\":\"" + String(hvacState.systemState == HvacSystemState::Running ? "Running" : "Off") + "\",";
    s += "\"mode\":\"" + String(hvacModeToString(hvacState.mode)) + "\",";
    s += "\"diagnostic\":\"" + String(diagToString(hvacState.diagnostic)) + "\"}";
    return s;
}

// Returns the "performance" JSON object (superheat + subcooling)
static String buildPerformanceJson() {
    String s = "\"performance\":{";
    s += "\"superheatF\":"  + floatToJson(hvacShSc.superheatF, 1) + ",";
    s += "\"subcoolingF\":" + floatToJson(hvacShSc.subcoolingF, 1) + "}";
    return s;
}

// ===================== HTTP HANDLERS =====================

void handleData() {
    String body = server.arg("plain");
    Serial.println("Received POST data:");
    Serial.println(body);
    server.send(200, "text/plain", "OK");
}

void handleRoot() {
    // Convert all temps to Fahrenheit once
    float hpLineF = celsiusToFahrenheit(hvacTemps.highPressureLineTempC);
    float lpLineF = celsiusToFahrenheit(hvacTemps.lowPressureLineTempC);
    float supplyF = celsiusToFahrenheit(hvacTemps.supplyAirTempC);
    float returnF = celsiusToFahrenheit(hvacTemps.returnAirTempC);

    String html = "<!DOCTYPE html><html><head>";
    html += "<meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1'>";
    html += "<title>HVAC Monitor</title>";
    html += "<style>"
            "body{font-family:Arial;margin:20px;background:#f0f0f0}"
            ".card{background:white;padding:15px;margin:10px 0;border-radius:8px;box-shadow:0 2px 4px rgba(0,0,0,0.1)}"
            "h1{color:#333}h2{color:#666;font-size:18px;margin-top:0}"
            ".value{font-size:24px;font-weight:bold;color:#007bff}"
            ".label{color:#666;font-size:14px}"
            "</style>";
    html += "<meta http-equiv='refresh' content='3'></head><body>";
    html += "<h1>🌡️ HVAC System Monitor</h1>";

    // System Status
    html += "<div class='card'><h2>System Status</h2>";
    html += "<div class='label'>State: <span class='value'>" + String(hvacState.systemState == HvacSystemState::Running ? "RUNNING" : "OFF") + "</span></div>";
    html += "<div class='label'>Mode: <span class='value'>" + String(hvacModeToString(hvacState.mode)) + "</span></div>";
    html += "<div class='label'>Diagnostic: <span class='value'>" + String(diagToString(hvacState.diagnostic)) + "</span></div>";
    html += "</div>";

    // Temperatures
    html += "<div class='card'><h2>Temperatures (°F)</h2>";
    html += "<div class='label'>High Pressure Line: <span class='value'>" + (isnan(hpLineF) ? "N/A" : String(hpLineF, 1)) + "°F</span></div>";
    html += "<div class='label'>Low Pressure Line: <span class='value'>"  + (isnan(lpLineF) ? "N/A" : String(lpLineF, 1)) + "°F</span></div>";
    html += "<div class='label'>Supply Air: <span class='value'>"         + (isnan(supplyF) ? "N/A" : String(supplyF, 1)) + "°F</span></div>";
    html += "<div class='label'>Return Air: <span class='value'>"         + (isnan(returnF) ? "N/A" : String(returnF, 1)) + "°F</span></div>";
    html += "</div>";

    // Pressures
    html += "<div class='card'><h2>Pressures (PSI)</h2>";
    html += "<div class='label'>Low Side: <span class='value'>"  + (isnan(hvacPressures.lowSidePressurePsi)  ? "N/A" : String(hvacPressures.lowSidePressurePsi,  1)) + " PSI</span></div>";
    html += "<div class='label'>High Side: <span class='value'>" + (isnan(hvacPressures.highSidePressurePsi) ? "N/A" : String(hvacPressures.highSidePressurePsi, 1)) + " PSI</span></div>";
    html += "</div>";

    // Performance
    html += "<div class='card'><h2>Performance</h2>";
    html += "<div class='label'>Superheat: <span class='value'>"  + (isnan(hvacShSc.superheatF)  ? "N/A" : String(hvacShSc.superheatF,  1)) + "°F</span></div>";
    html += "<div class='label'>Subcooling: <span class='value'>" + (isnan(hvacShSc.subcoolingF) ? "N/A" : String(hvacShSc.subcoolingF, 1)) + "°F</span></div>";
    html += "</div>";

    // Active Faults
    if (faultReport.count > 0) {
        html += "<div class='card' style='border-left:4px solid #dc3545'>";
        html += "<h2>⚠️ Active Faults (" + String(faultReport.count) + ")</h2>";
        for (uint8_t i = 0; i < faultReport.count; i++) {
            html += "<div class='label'>• " + String(faultCodeToString(faultReport.codes[i])) + "</div>";
        }
        html += "</div>";
    }

    html += "<div class='card' style='font-size:12px;color:#999'>Auto-refresh every 3 seconds | Uptime: " + String(millis() / 1000) + "s</div>";
    html += "</body></html>";

    server.send(200, "text/html", html);
}

void handleJson() {
    float hpLineF = celsiusToFahrenheit(hvacTemps.highPressureLineTempC);
    float lpLineF = celsiusToFahrenheit(hvacTemps.lowPressureLineTempC);
    float supplyF = celsiusToFahrenheit(hvacTemps.supplyAirTempC);
    float returnF = celsiusToFahrenheit(hvacTemps.returnAirTempC);

    String json = "{";
    json += buildSystemJson() + ",";

    json += "\"temperatures\":{";
    json += "\"highPressureLineF\":" + floatToJson(hpLineF) + ",";
    json += "\"lowPressureLineF\":"  + floatToJson(lpLineF) + ",";
    json += "\"supplyAirF\":"        + floatToJson(supplyF) + ",";
    json += "\"returnAirF\":"        + floatToJson(returnF) + "},";

    json += "\"pressures\":{";
    json += "\"lowSidePsi\":"  + floatToJson(hvacPressures.lowSidePressurePsi,  1) + ",";
    json += "\"highSidePsi\":" + floatToJson(hvacPressures.highSidePressurePsi, 1) + "},";

    json += buildPerformanceJson() + ",";

    json += "\"faults\":{\"count\":" + String(faultReport.count) + ",\"codes\":" + buildFaultCodesJson() + "}}";

    server.send(200, "application/json", json);
}

// ===================== SERVER LIFECYCLE =====================

void startLocalServer() {
    server.on("/",     HTTP_GET,  handleRoot);
    server.on("/json", HTTP_GET,  handleJson);
    server.on("/data", HTTP_POST, handleData);
    server.begin();
    Serial.println("Local HTTP server started on port 80");
    Serial.println("Dashboard: http://" + WiFi.localIP().toString());
}

void handleLocalServer() {
    server.handleClient();
}

// ===================== WIFI PROVISIONING =====================

void wifiProvision() {
    Serial.println("[WiFi] Starting provisioning...");

    WiFiManager wm;
    wm.setConnectTimeout(10);

    // Uncomment to reset saved WiFi credentials:
    // wm.resetSettings();

    bool connected = wm.autoConnect("ESP32_Setup_AP", "setup1234");

    if (!connected) {
        Serial.println("[WiFi] Connection failed, restarting...");
        delay(3000);
        ESP.restart();
    }

    Serial.println("[WiFi] Connected!");
    Serial.println(WiFi.localIP());
}

// ===================== AWS DATA SEND =====================

void sendToAWS() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[AWS] WiFi not connected, skipping send");
        return;
    }

    uint32_t now = millis();
    if (now - lastAwsSendMs < NETWORK_SEND_INTERVAL_MS) return;
    lastAwsSendMs = now;

    // Pre-compute Fahrenheit values
    float hpLineF = celsiusToFahrenheit(hvacTemps.highPressureLineTempC);
    float lpLineF = celsiusToFahrenheit(hvacTemps.lowPressureLineTempC);
    float supplyF = celsiusToFahrenheit(hvacTemps.supplyAirTempC);
    float returnF = celsiusToFahrenheit(hvacTemps.returnAirTempC);
    float deltaF  = deltaCelsiusToDeltaFahrenheit(hvacTemps.deltaTempC);
    float deltaPsi = computeDeltaPsi(hvacPressures);

    bool allTempsValid     = !isnan(hpLineF) && !isnan(lpLineF) && !isnan(supplyF) && !isnan(returnF);
    bool allPressuresValid = !isnan(hvacPressures.lowSidePressurePsi) && !isnan(hvacPressures.highSidePressurePsi);

    String payload = "{";
    payload += "\"device\":\"esp32_hvac_1\",";
    payload += "\"uptime\":"    + String(millis()) + ",";
    payload += "\"wifi_rssi\":" + String(WiFi.RSSI()) + ",";

    payload += buildSystemJson() + ",";

    payload += "\"temperatures\":{";
    payload += "\"highPressureLineF\":" + floatToJson(hpLineF) + ",";
    payload += "\"lowPressureLineF\":"  + floatToJson(lpLineF) + ",";
    payload += "\"supplyAirF\":"        + floatToJson(supplyF) + ",";
    payload += "\"returnAirF\":"        + floatToJson(returnF) + ",";
    payload += "\"deltaTempF\":"        + floatToJson(deltaF)  + ",";
    payload += "\"lowSideSatF\":"       + floatToJson(hvacSatTemps.lowSideSatTempF)  + ",";
    payload += "\"highSideSatF\":"      + floatToJson(hvacSatTemps.highSideSatTempF) + "},";

    payload += "\"pressures\":{";
    payload += "\"lowSidePsi\":"  + floatToJson(hvacPressures.lowSidePressurePsi,  1) + ",";
    payload += "\"highSidePsi\":" + floatToJson(hvacPressures.highSidePressurePsi, 1) + ",";
    payload += "\"deltaPsi\":"    + floatToJson(deltaPsi, 1) + "},";

    payload += buildPerformanceJson() + ",";

    payload += "\"faults\":{";
    payload += "\"count\":"      + String(faultReport.count) + ",";
    payload += "\"diagnostic\":\"" + String(diagToString(faultReport.diag)) + "\",";
    payload += "\"codes\":"      + buildFaultCodesJson() + "},";

    payload += "\"validity\":{";
    payload += "\"allTempsValid\":"     + String(allTempsValid     ? "true" : "false") + ",";
    payload += "\"allPressuresValid\":" + String(allPressuresValid ? "true" : "false") + "}}";

    Serial.println("[AWS] Sending data...");
    Serial.println("[AWS] Payload: " + payload.substring(0, 100) + "...");

    HTTPClient http;
    http.begin(AWS_URL);
    http.addHeader("Content-Type", "application/json");

    int code = http.POST(payload);
    Serial.printf("[AWS] Response code: %d\n", code);
    if (code > 0) {
        Serial.println("[AWS] Response: " + http.getString());
    } else {
        Serial.println("[AWS] POST failed: " + http.errorToString(code));
    }
    http.end();

    Serial.println("[AWS] Send complete");
}

