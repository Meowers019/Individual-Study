#ifndef NETWORK_SETUP_H
#define NETWORK_SETUP_H

// ===================== NETWORK CONFIGURATION =====================

// Access point credentials used for initial WiFi setup (captive portal)
#define LOCAL_SSID     "esp_router"
#define LOCAL_PASSWORD "password123"

// AWS API Gateway endpoint
#define AWS_URL "https://oyhd0jydrg.execute-api.us-east-2.amazonaws.com/data"

// ===================== FUNCTION DECLARATIONS =====================

// Connect to WiFi using saved credentials, or open a captive portal for setup
void wifiProvision();

// Local HTTP server routes:
//   GET /              — single-unit live monitor page
//   GET /json          — single-unit JSON snapshot
//   POST /data         — receive external POST data
//   GET /dashboard     — multi-unit demo dashboard (5 simulated units)
//   GET /dashboard/json            — JSON array of all dashboard units
//   GET /dashboard/setscenario     — ?unit=N&scenario=S to change a unit's scenario
void startLocalServer();
void handleLocalServer();

// Transmit current HVAC snapshot to AWS (rate-limited by NETWORK_SEND_INTERVAL_MS)
void sendToAWS();

// Dashboard demo: 5 simulated HVAC units with real-world variance.
// Defined in network.cpp (via dashboard_demo.h) to keep static storage in one TU.
void initDashboard();
void updateDashboard();

#endif // NETWORK_SETUP_H
