/*
 * ESP32-based Inter-UAV Communications System
 * Muhammad Adeel Mahdi Suviyanto
 * Telkom University
 * 
 * First GPS implementation
 * 
 * Version 0.2, 2021-11-22
 * Node 1
 */


#include "painlessMesh.h"
#include <Arduino_JSON.h>
#include <WiFi.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>


//Mesh details
#define MESH_PREFIX "UAV" //name of mesh
#define MESH_PASS "12345678" //mesh password
#define MESH_PORT 5555

//Node number for current board
int nodeNumber = 1;

//Data string to send
String locationData;

//Declare RX/TX pin and baud rate of GPS Module
static const int RXPin = 3, TXPin = 1;
static const uint32_t GPSBaud = 9600;

//Declarations
Scheduler userScheduler;
painlessMesh mesh;
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);
void sendMessage();
String getLocation();


//Placeholder data, to clear later and replace const float to float
//Update v0.2, first implementation of real data, comments to be removed in next revision.
double currentLatitude;// = -6.97689;
double currentLongitude;// = 107.62975;
double currentAltitude;// = 30.00000;

//Send repetitively via Task Scheduler
Task taskSendMessage(TASK_SECOND * 10, TASK_FOREVER, &sendMessage);

//Get Location Data from GPS board (NOT YET IMPLEMENTED, USING PLACEHOLDER DATA)
String getLocation(){
//lagi dipindahin
   if(gps.location.isUpdated()){
     currentLatitude = (gps.location.lat());
     Serial.print("Lat = "); Serial.println(currentLatitude, 6);
     currentLongitude = (gps.location.lng());
     Serial.print("Lng = "); Serial.println(currentLongitude, 6);
     currentAltitude = (gps.altitude.meters());
   }
  JSONVar jsonReadings;
  jsonReadings["node"] = nodeNumber;
  jsonReadings["latitude"] = currentLatitude;
  jsonReadings["longitude"] = currentLongitude;
  jsonReadings["altitude"] = currentAltitude;
  locationData = JSON.stringify(jsonReadings);
  return locationData;
}

void sendMessage(){
  String msg = getLocation();
  mesh.sendBroadcast(msg);
}

//painlessMesh serial outputs
/*void receivedCallback(uint32_t from, String &msg){
  Serial.printf("Message from %u msg=%s\n", from, msg.c_str());
  
}*/
void receivedCallback(uint32_t from, String &msg){
  Serial.printf("Message from %u msg=%s\n", from, msg.c_str());
  /*JSONVar myObject = JSON.parse(msg.c_str());
  int node = myObject["node"];
  double latitude = myObject["latitude"];
  double longitude = myObject["longitude"];
  double altitude = myObject["altitude"];
  Serial.print("Node: ");
  Serial.println(node);
  Serial.print("Latitude: ");
  Serial.println(latitude);
  Serial.print("Longitude: ");
  Serial.println(longitude);
  Serial.print("Altitude: ");
  Serial.println(altitude);*/
  long rssi = WiFi.RSSI();
  Serial.print("RSSI: ");
  Serial.println(rssi);
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

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  ss.begin(GPSBaud);
  mesh.setDebugMsgTypes( ERROR | STARTUP );
  mesh.init( MESH_PREFIX, MESH_PASS, &userScheduler, MESH_PORT );
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);
  mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);

  userScheduler.addTask(taskSendMessage);
  taskSendMessage.enable();

}

void loop() {
  // put your main code here, to run repeatedly:
  mesh.update();
  while(ss.available() > 0){
    gps.encode(ss.read());

  }
}
