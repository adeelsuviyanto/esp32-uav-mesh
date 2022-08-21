/*
  * ESP32-based Inter-UAV Communications System
  * Muhammad Adeel Mahdi Suviyanto
  * Telkom University
  * 
  * Base station receiver, ESP32 Web Server-side
  * for web server to display location and mesh data
  * 
  * Version 2.0, 2022-08-08
  * Node number unapplicable
*/
#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <HardwareSerial.h>
#include <WebServer.h>

// SSID for Web Server
const char* ssid = "UAV_WebServer";
const char* password = "12345678";

// IP Address details
IPAddress local_ip(192,168,1,1);
IPAddress gateway(192,168,1,1);
IPAddress subnet(255,255,255,0);

// Declaration for web server
WebServer server(80);

// Sender Data
double latitude;
double longitude;
double altitude;
uint8_t satellite;
boolean gpsReady;
uint32_t timestamp;
uint8_t nodeType;

// Function prototypes
void handle_OnConnect();
void handle_GetSender();
void handle_GetGPSStatus();
void handle_GetLocationData();
String sendHTML();

String sendHTML(){
  String ptr = "<!DOCTYPE html> <html>\n";
  ptr += "<head><title>Mesh Base Station</title>\n";
  ptr += "</head>\n";
  ptr += "<body>\n";
  ptr += "<h1>Mesh Base Station</h1>\n";
  ptr += "<p>Node type: " + String(nodeType) + "\n";
  ptr += "<p>GPS Ready: ";
    if(gpsReady) ptr += "ready\n"; else ptr += "not ready\n";
  ptr += "Latitude: " + String(latitude,6) + "\n";
  ptr += "Longitude: " + String(longitude, 6) +"\n";
  ptr += "Altitude: " + String(altitude, 6) + "\n";
  ptr += "Satellite: " + String(satellite) + "\n";
  ptr += "</body>\n";
  ptr += "</html>\n";
  return ptr;
}

void handle_OnConnect(){
  server.send(200, "text/html", sendHTML());
}

void handle_GetSender(){
  StaticJsonDocument<200> doc;
  doc["type"] = 0;
  serializeJson(doc, Serial2);
  doc.clear();
}

void handle_GetGPSStatus(){
  StaticJsonDocument<200> doc;
  doc["type"] = 1;
  serializeJson(doc, Serial2);
  doc.clear();
}

void sendRequest(uint8_t msgtype){
  StaticJsonDocument<200> doc;
  doc["type"] = msgtype;
  serializeJson(doc, Serial2);
  doc.clear();
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(4800, SERIAL_8N1);

  WiFi.softAP(ssid, password);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  delay(100);

  server.on("/", handle_OnConnect);
  server.begin();
  delay(10000);
  sendRequest(0);
  delay(5000);
  sendRequest(1);
}

void loop() {
  server.handleClient();
  sendRequest(2);
  delay(3000);
  if(Serial2.available()){
    StaticJsonDocument<300> doc;
    DeserializationError err = deserializeJson(doc, Serial);

    if(err == DeserializationError::Ok){
      timestamp = doc["timestamp"];
      gpsReady = doc["GPSReady"];
      nodeType = doc["nodetype"];
      latitude = doc["latitude"];
      longitude = doc["longitude"];
      altitude = doc["altitude"];
      satellite = doc["satellite"];
    }
    else{
      Serial.println(err.c_str());
      while(Serial2.available() > 0) Serial2.read();
    }
  }
}