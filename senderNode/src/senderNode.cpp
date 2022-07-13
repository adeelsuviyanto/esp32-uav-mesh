/*
 * ESP32-based Inter-UAV Communications System
 * Muhammad Adeel Mahdi Suviyanto
 * Telkom University
 * 
 * 4th GPS implementation, adjusted some flags
 * 
 * Version 2.0, 2021-07-06
 * 
 * Node 1
 */

#include <Arduino.h>
#include "painlessMesh.h"
#include "esp_wifi.h"
//#include <Arduino_JSON.h>
#include <WiFi.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

//Mesh details
#define MESH_PREFIX "UAV" //name of mesh
#define MESH_PASS "12345678" //mesh password
#define MESH_PORT 5555

//Board details
#define ledRed 9 //cek lagi nanti gimana enaknya posisiin LED-nya

//Node number for current board
int nodeNumber = 1;

//Location data to send
String locationData;

//Declarations
static const int RXPin = 16, TXPin = 17;
static const uint32_t GPSBaud = 9600;
Scheduler userScheduler;
painlessMesh mesh;
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);
void sendMessage();
String getLocation();

//Variables
double currentLatitude;
double currentLongitude;
double currentAltitude;
int currentSatellites;
int currentGPSDate;
int currentGPSTime;
uint32_t receiverNode;
long rssi;

void showGPSData();
Task taskShowGPSData(TASK_SECOND, -1, &showGPSData);

void getLocation(uint32_t toNode){
    locationData = "";
    Serial.println("Sending Location Data");
    DynamicJsonDocument locDat(1024);
    locDat["latitude"] = currentLatitude;
    locDat["longitude"] = currentLongitude;
    locDat["altitude"] = currentAltitude;
    locDat["satellites"] = currentSatellites;
    locDat["rssi"] = rssi;
    serializeJson(locDat, locationData);
    mesh.sendSingle(receiverNode, locationData);
    Serial.print("Sent message: ");
    Serial.println(locationData);
    locDat.clear();
}

void showGPSData(){
    Serial.printf("Latitude: %f\n", currentLatitude);
    Serial.printf("Longitude: %f\n", currentLongitude);
    Serial.printf("Altitude: %f\n", currentAltitude);
    Serial.printf("Satellites: %i\n", currentSatellites);
}

void replyNodeQuery(uint32_t toNode){
    Serial.println("Sending Node Type");
    receiverNode = toNode;
    String msg;
    DynamicJsonDocument JsonDocument(1024);
    JsonObject reply = JsonDocument.to<JsonObject>();
    reply["nodeType"] = "sender";
    reply["rssi"] = rssi;
    serializeJson(reply, msg);
    mesh.sendSingle(toNode, msg);
    JsonDocument.clear();
}

void replyGPSQuery(uint32_t toNode){
    Serial.println("Sending GPS Status Query");
    String msg;
    DynamicJsonDocument reply(1024);
    if(currentSatellites < 3 || (currentLatitude == 0 && currentLongitude == 0 && currentAltitude == 0)) reply["locationReady"] = 0; //Set locationReady flag to 0 if location data is not ready
    else reply["locationReady"] = 1; //Or else, send locationReady flag set to 1.
    reply["rssi"] = rssi;
    serializeJson(reply, msg);
    mesh.sendSingle(toNode, msg);
    Serial.print("Sent message: ");
    Serial.println(msg);
    reply.clear();
}

void newConnectionCallback(uint32_t nodeId){
    Serial.printf("New Connection, nodeId = %u\n", nodeId);
}

void changedConnectionCallback() {
    Serial.printf("Changed connections\n");
}

void nodeTimeAdjustedCallback(int32_t offset) {
    Serial.printf("Adjusted time %u. Offset = %d\n", mesh.getNodeTime(),offset);
}

void receivedCallback(uint32_t from, String &msg){
    Serial.printf("Message from %u msg=%s\n", from, msg.c_str());
    if (strcmp(msg.c_str(), "getType") == 0) replyNodeQuery(from);
    if (strcmp(msg.c_str(), "gpsStatus") == 0) replyGPSQuery(from);
    if (strcmp(msg.c_str(), "reqLocation") == 0) getLocation(from);
}

void setup(){
    Serial.begin(115200);
    ss.begin(GPSBaud);
    mesh.setDebugMsgTypes( ERROR | STARTUP | CONNECTION);
    mesh.init(MESH_PREFIX, MESH_PASS, MESH_PORT);
    esp_wifi_set_protocol(WIFI_IF_AP, WIFI_PROTOCOL_LR);
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);
    mesh.onReceive(&receivedCallback);
    mesh.onNewConnection(&newConnectionCallback);
    mesh.onChangedConnections(&changedConnectionCallback);
    mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);  
    userScheduler.addTask(taskShowGPSData);
    taskShowGPSData.enable();  

    //LED Status setup
    pinMode(ledRed, OUTPUT); //Entar dicek lagi
}

void loop(){
    mesh.update();
    userScheduler.execute();
    while(ss.available() > 0){
        gps.encode(ss.read());
    }
    if(millis() > 5000 && gps.charsProcessed() < 10){
        Serial.println("Error: No GPS data received. Check wiring and reset.");
        Serial.println("Board will now dump GPS stream: ");
        digitalWrite(ledRed, HIGH);
        while(true)
        if(ss.available() > 0) {
            Serial.write(ss.read());
        }
    }
    currentLatitude = (gps.location.lat());
    currentLongitude = (gps.location.lng());
    currentAltitude = (gps.altitude.meters());
    currentSatellites = (gps.satellites.value());
    rssi = WiFi.RSSI();
}