/*
  * ESP32-based Inter-UAV Communications System
  * Muhammad Adeel Mahdi Suviyanto
  * Telkom University
  * 
  * 2nd receiver implementation, new algorithm
  * 
  * Version 0.2, 2021-11-28
  * Node 2 (receiver)
*/
#include <Arduino.h>
#include <Arduino_JSON.h>
#include "painlessMesh.h"
#include <WiFi.h>
#include <TaskScheduler.h>
#include <TaskSchedulerDeclarations.h>

//Mesh settings
#define MESH_PREFIX "UAV"
#define MESH_PASS "12345678"
#define MESH_PORT 5555

//Node number for current board
int nodeNumber = 2;

//Declarations
Scheduler userScheduler;
painlessMesh mesh;
int i = 0;

int error;
double currentLatitude;
double currentLongitude;
double currentAltitude;
int currentSatellites;
int currentGPSDate;
int currentGPSTime;

//Functions
void requestLocation();
void queryNodeType();
void acknowledge();

struct{
  uint32_t senderNodeId;
  boolean gpsReady = false;
}senderNodes;

Task taskReqQuery( TASK_SECOND * 10, TASK_FOREVER, &requestLocation);
Task taskReqLocation( 5000, -1, &requestLocation);

void requestLocation(uint32_t toNode){
  String msg = "reqLocation";
  mesh.sendSingle(toNode, msg);
}

void queryNodeType(uint32_t toNode){
  String msg = "getType";
  mesh.sendSingle(toNode, msg);
}

void acknowledge(uint32_t toNode){
  String msg = "OK Location Data received from ";
  msg = msg + toNode;
  mesh.sendSingle(toNode, msg);
}

void newConnectionCallback(uint32_t nodeId){
  Serial.printf("New Connection, nodeId = %u\n", nodeId);
  queryNodeType(nodeId);
}

void receivedCallback(uint32_t from, String &msg){
  //JSONVar recObj = JSON.parse(msg.c_str()); //parse JSON string to objects
   DynamicJsonDocument JsonDocument(1024 + msg.length());
   deserializeJson(JsonDocument, msg);
   JsonObject recObj = JsonDocument.as<JsonObject>();
   long rssi = WiFi.RSSI();
  if(recObj.containsKey("nodeType")){ //check if sender node sent "sender" after query node type
    String nodeType = recObj["nodeType"];
    if(nodeType == "sender"){
      senderNodes.senderNodeId = from;
      if(taskReqQuery.isEnabled() == false) taskReqQuery.enable();
    }
    Serial.printf("Added node %u to sender nodes", from);
    Serial.printf("RSSI: ");
    Serial.println(rssi);
  }
  if(recObj.containsKey("locationReady")){
    boolean locationReady = recObj["locationReady"];
    if(locationReady == true && taskReqQuery.isEnabled() == true){
      senderNodes.gpsReady = true;
      taskReqQuery.disable();
      taskReqLocation.enable();
    }
    else senderNodes.gpsReady = false;
    Serial.printf("Sender GPS ready: %s", senderNodes.gpsReady ? "true" : "false");
  }
  else{
    Serial.printf("Message from %u msg=%s\n", from, msg.c_str());
    if(senderNodes.gpsReady == true){
      int node;
      double latitude;
      double longitude;
      double altitude;
      int satellites;
      node = recObj["node"];
      Serial.println(node);
      latitude = recObj["latitude"];
      Serial.println(latitude, 6);
      longitude = recObj["longitude"];
      Serial.println(longitude, 6);
      altitude = recObj["altitude"];
      Serial.println(altitude);
      satellites = recObj["satellites"];
      acknowledge(from);
    }
  }
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
  mesh.setDebugMsgTypes (ERROR | STARTUP | CONNECTION);
  mesh.init(MESH_PREFIX, MESH_PASS, MESH_PORT);
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);
  mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);
  
  userScheduler.addTask(taskReqQuery);
  userScheduler.addTask(taskReqLocation);
  //taskReqQuery.enable();

}

void loop() {
  // put your main code here, to run repeatedly:
  mesh.update();
}
