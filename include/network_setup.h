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

// Local HTTP server: serves a live dashboard at "/" and JSON API at "/json"
void startLocalServer();
void handleLocalServer();

// Transmit current HVAC snapshot to AWS (rate-limited by NETWORK_SEND_INTERVAL_MS)
void sendToAWS();

#endif // NETWORK_SETUP_H
