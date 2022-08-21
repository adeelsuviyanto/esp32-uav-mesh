/*
  * ESP32-based Inter-UAV Communications System
  * Muhammad Adeel Mahdi Suviyanto
  * Telkom University
  * 
  * Base station receiver, ESP8266-side
  * for web server to display location and mesh data
  * 
  * Version 1.0, 2022-08-07
  * Node number unapplicable
*/

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ArduinoJson.h>
#include <SoftwareSerial.h>
#include <StreamUtils.h>

// SSID for Web Server
const char* ssid = "UAV_WebServer";
const char* password = "12345678";

// IP Address details
IPAddress local_ip(192,168,1,1);
IPAddress gateway(192,168,1,1);
IPAddress subnet(255,255,255,0);

// Web server port declaration
ESP8266WebServer server(80);

SoftwareSerial Serial2(13, 15);

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
  ptr += "<meta http-equiv=\"refresh\" content=\"2\">\n";
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
  ptr += "<a href=\"/getsender\">Get Sender Node</a>\n";
  ptr += "<a href=\"/getgps\">Get GPS Status</a>\n";
  ptr += "<a href=\"/getlocation\">Get Location Data</a>\n";
  ptr += "<a href=\"/\">Reset address</a>\n";
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
  server.send(200, "text/html", sendHTML());
}

void handle_GetGPSStatus(){
  StaticJsonDocument<200> doc;
  doc["type"] = 1;
  serializeJson(doc, Serial2);
  doc.clear();
  server.send(200, "text/html", sendHTML());
}

void handle_GetLocationData(){
  StaticJsonDocument<200> doc;
  doc["type"] = 2;
  serializeJson(doc, Serial2);
  doc.clear();
  server.send(200, "text/html", sendHTML());
}

void sendRequest(uint8_t msgtype){
  StaticJsonDocument<200> doc;
  doc["type"] = msgtype;
  serializeJson(doc, Serial2);
  doc.clear();
}

void setup() {
  Serial.begin(9600);
  Serial2.begin(4800, SWSERIAL_8N1);

  WiFi.softAP(ssid, password);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  delay(100);

  server.on("/", handle_OnConnect);
  server.on("/getsender", handle_GetSender);
  server.on("/getgps", handle_GetGPSStatus);
  server.on("/getlocation", handle_GetLocationData);
  server.begin();
}

void loop() {
  server.handleClient();
  while(Serial2.available() > 0){
    Serial.setTimeout(10000);

    ReadLoggingStream docWithLog(Serial2, Serial);    
    StaticJsonDocument<300> doc;
    DeserializationError err = deserializeJson(doc, docWithLog);

    if(err == DeserializationError::Ok){
      timestamp = doc["timestamp"];
      if(doc.containsKey("nodetype")) nodeType = doc["nodetype"];
      if(doc.containsKey("GPSReady")) gpsReady = doc["GPSReady"];
      if(doc.containsKey("latitude")){
        latitude = doc["latitude"];
        longitude = doc["longitude"];
        altitude = doc["altitude"];
        satellite = doc["satellite"];
      }
      server.send(200, "text/html", sendHTML()); 
    }
    else{
      Serial.println(err.c_str());
      while(Serial2.available() > 0) Serial2.read();
    }
  }
}