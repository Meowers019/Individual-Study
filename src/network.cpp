#include <WiFi.h>
#include "network_setup.h"
#include <WebServer.h>
#include <HTTPClient.h>
#include <WiFiManager.h>

const char *ssid = WIFI_SSID;
const char *password = WIFI_PASS;
const char *aws_url = AWS_URL;

WebServer server(80);

void wifiProvision()
{
    WiFiManager wm; // creates an object of WifiManger called wm. could be called anything
    // This will allow us to make function calls to this WifiManager

    wm.resetSettings();
    // This is a line of code i'd like to run later. It would essentally say that if we press
    // A button or something then it would go through and reset the wifi so we have to enter
    // everything in again

    bool res = wm.autoConnect(ssid, password);
    // This is the most imporant function. Essentally it will go through and Check Memory for
    //  previously stored wifi passwords. If and only if there are no previously stored wifi
    //  creds then it will create its own access point for you to connect and do everything
    //  you need to on it. Imporant to note that this will wait for a decent amount of time
    //  before going to next stage, which will make it so its less likly to time out

    if (!res)
    { // if and only if we fail to connect to the internet via the autoconnect
        // function then we will go through and
        Serial.println("WiFi failed, restarting...");
        delay(3000);
        ESP.restart();
    }

    Serial.println("WiFi connected!");
    Serial.println(WiFi.localIP());
}

void handleData()
{
    String body = server.arg("plain");
    Serial.println("Received data:");
    Serial.println(body);
    server.send(200, "text/plain", "OK");
}

void startLocalServer()
{
    Serial.println("starting local server (not really)");

    // server.on("/data", HTTP_POST, handleData); //this will basically say
    // that if there is a POST request /data of this esp IP, send it too
    // "handle data", which in turn will pretty much send back a ack
    // to the sender

    //   server.begin(); // This just tells the esp go go ahead and start lisening
    //    Serial.println("Local HTTP server started");
}

void handleLocalServer()
{
    Serial.println("handel local server");
    //  server.handleClient(); //This will go through and see if something comes in
    // (like a post or get request or sum like that) and if so it will then go
    // through and handel the data appropately
}

void sendToAWS()
{
    if (WiFi.status() != WL_CONNECTED)
        return;

    HTTPClient http;     // creates object http
    http.begin(aws_url); // http is connected to aws_URL
    http.addHeader("Content-Type", "application/json");

    String payload = "{\"temp\":24.5, \"device\":\"esp32_1\"}";

    int code = http.POST(payload);
    // we go through and we actually send the data to the AWS server. After
    // we send this data it will return some value (hopefully 200 because
    // that means it was a sucess)

    Serial.print("AWS POST code: ");
    Serial.println(code);

    http.end();
}
