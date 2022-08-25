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
uint32_t timestampRXTX;
uint32_t timestampTXRX;
uint32_t timestampDelta;

void showGPSData();
Task taskShowGPSData(TASK_SECOND, -1, &showGPSData);

void getLocation(uint32_t toNode){
    timestampTXRX = mesh.getNodeTime();
    locationData = "";
    Serial.println("Sending Location Data");
    DynamicJsonDocument locDat(1024);
    locDat["type"] = 3;
    locDat["latitude"] = currentLatitude;
    locDat["longitude"] = currentLongitude;
    locDat["altitude"] = currentAltitude;
    locDat["satellites"] = currentSatellites;
    locDat["timestamp"] = timestampTXRX;
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

    reply["type"] = 1;
    reply["timestamp"] = timestampTXRX;
    reply["nodeType"] = 1;
    reply["rssi"] = rssi;
    serializeJson(reply, msg);
    mesh.sendSingle(toNode, msg);
    JsonDocument.clear();
}

void replyGPSQuery(uint32_t toNode){
    timestampTXRX = mesh.getNodeTime();
    Serial.println("Sending GPS Status Query");
    String msg;
    DynamicJsonDocument reply(1024);
    reply["type"] = 2;

    // Logic check, if GPS is ready then send locationReady = 1
    //if(currentSatellites < 3 || (currentLatitude == 0 && currentLongitude == 0 && currentAltitude == 0)) reply["locationReady"] = 0; //Set locationReady flag to 0 if location data is not ready
    //else reply["locationReady"] = 1; // Or else, send locationReady flag set to 1.
    // Disabled for testing, comment out the line below to reenable logic check
    reply["locationReady"] = 1;

    reply["timestamp"] = timestampTXRX;
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
    int timestampReceived = mesh.getNodeTime();
    Serial.printf("Message from %u msg=%s\n", from, msg.c_str());

    // Parsing JSON messages to JSON objects
    DynamicJsonDocument JsonDocument(1024 + msg.length());
    JsonDocument.clear();
    deserializeJson(JsonDocument, msg);
    JsonObject receivedMsg = JsonDocument.as<JsonObject>();

    uint8_t type = receivedMsg["type"];
    uint8_t timestampTXRX = receivedMsg["timestamp"];

    if (type == 1) replyNodeQuery(from);
    if (type == 2) replyGPSQuery(from);
    if (type == 3) getLocation(from);
}

void setup(){
    Serial.begin(115200);
    ss.begin(GPSBaud);
    mesh.setDebugMsgTypes( ERROR | STARTUP | CONNECTION);
    mesh.init(MESH_PREFIX, MESH_PASS, MESH_PORT, WIFI_MODE_APSTA, 6);
    esp_wifi_set_protocol(WIFI_IF_AP, WIFI_PROTOCOL_11N);
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11N);
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