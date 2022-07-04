/*
  * ESP32-based Inter-UAV Communications System
  * Muhammad Adeel Mahdi Suviyanto
  * Telkom University
  * 
  * 2nd receiver implementation, new algorithm
  * 
  * Changelog:
  * v0.1, 2021-11-19: First implementation of receiver node and painlessMesh
  * v0.2, 2021-11-24: Slight revision to algorithm
  * v1.0, 2021-11-29: New algorithm, polling from receiver instead of sender
  * 
  * Version 1.0, 2021-11-29
  * Node 2 (receiver)
*/
#include <Arduino.h>
//#include <ArduinoJson.h>
#include "painlessMesh.h"
#include "esp_wifi.h"
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
long rssi;

unsigned long timing;

struct{
  uint32_t senderNodeId;
  boolean gpsReady = false;
  uint8_t retryCounter;
  uint16_t sentPingCounter;
  uint16_t receivedPingCounter;
}senderNodes;
uint32_t destNode;

void requestLocation();
void queryGPSStatus();
void queryNodeType();

Task taskReqQuery(TASK_SECOND * 20, -1, &queryGPSStatus );
Task taskQueryNode(TASK_SECOND * 5, -1, &queryNodeType );
Task taskReqLocation(TASK_SECOND * 10, -1, &requestLocation );

void requestLocation(){
  if(taskReqQuery.isEnabled() == true) taskReqQuery.disable();
  Serial.printf("\nRequesting location data from sender..");
  String msg = "reqLocation";
  mesh.sendSingle(senderNodes.senderNodeId, msg);
  Serial.printf("\nWiFi RSSI value: %li dBm", rssi);
}

void queryGPSStatus(){
  if(taskQueryNode.isEnabled() == true) taskQueryNode.disable();
  Serial.printf("\nQuerying GPS status from sender..");
  String msg = "gpsStatus";
  mesh.sendSingle(senderNodes.senderNodeId, msg);
  Serial.printf("\nWiFi RSSI value: %li dBm", rssi);
  senderNodes.retryCounter++;

  //Disable query if no reply from sender for 5 requests
  if(senderNodes.retryCounter >= 5){
    taskReqQuery.disable();
  }
}

void queryNodeType(){
  Serial.printf("\nQuerying node type...");
  Serial.println(millis());
  String msg = "getType";
  mesh.sendSingle(destNode, msg);
  Serial.printf("\nWiFi RSSI value: %li dBm", rssi);
}

void acknowledge(uint32_t toNode){
  Serial.println("Sending acknowledge..");
  String msg = "OK Location Data received from ";
  msg = msg + toNode;
  mesh.sendSingle(toNode, msg);
}

void newConnectionCallback(uint32_t nodeId){
  Serial.printf("New Connection, nodeId = %u\n", nodeId);
  destNode = nodeId;
  if(taskQueryNode.isEnabled() == false) {
    taskQueryNode.enable();
    Serial.printf("taskQueryNode %s", taskQueryNode.isEnabled() ? "enabled" : "disabled");
    ;
  }
  else {
    queryNodeType();
    Serial.printf("taskQueryNode is not enabled");
  }
}

void receivedCallback(uint32_t from, String &msg){
  Serial.printf("\nMessage from %u msg=%s\n", from, msg.c_str());
  long int senderRSSI;
  //JSONVar recObj = JSON.parse(msg.c_str()); 
  
  //parse JSON string to objects
  DynamicJsonDocument JsonDocument(1024 + msg.length());
  JsonDocument.clear();
  deserializeJson(JsonDocument, msg);
  JsonObject recObj = JsonDocument.as<JsonObject>();
  senderRSSI = recObj["rssi"];
  Serial.printf("\nClient-side RSSI: %li dBm\n", senderRSSI);

  if(recObj.containsKey("nodeType")){ //check if sender node sent "sender" after query node type
    String nodeType = recObj["nodeType"];
    if(nodeType.equals("sender") == true){
      senderNodes.senderNodeId = from;
      Serial.printf("senderNodes.senderNodeId = %u", from);
      taskReqQuery.enable();
      ;
      //queryGPSStatus();
    }
    Serial.printf("\nAdded node %u to sender nodes", from);
  }
  if(recObj.containsKey("locationReady")){
    //Reset retry counter to 0
    senderNodes.retryCounter = 0;

    uint8_t locationReady = recObj["locationReady"];
    if(locationReady == 1){
      senderNodes.gpsReady = true;
      taskReqQuery.disable();
      taskReqLocation.enable();
      ;
    }
    else{
      senderNodes.gpsReady = false;
      if(taskReqLocation.isEnabled() == true )taskReqLocation.disable();
    }
    Serial.printf("Sender GPS ready: %s", senderNodes.gpsReady ? "true" : "false");
  }
  else{
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
      Serial.println(satellites);
      if(taskQueryNode.isEnabled() == true) taskQueryNode.disable();
      acknowledge(from);
    }
  }
  JsonDocument.clear();
}

void changedConnectionCallback() {
  Serial.printf("Changed connections\n");
  if(taskQueryNode.isEnabled() == false) taskQueryNode.enable();
  else queryNodeType();
}

void nodeTimeAdjustedCallback(int32_t offset) {
  Serial.printf("Adjusted time %u. Offset = %d\n", mesh.getNodeTime(),offset);
}

void nodeTripDelay(uint32_t nodeId, int32_t delay){
  Serial.printf("\nTrip delay measurement from %u: %i ms, %i us", nodeId, delay/1000, delay);
  senderNodes.receivedPingCounter++;
}

void packetLossTest(unsigned long timeDelay) {
  unsigned long currentTime = millis();
  if(millis() >= currentTime + timeDelay){
    mesh.startDelayMeas(senderNodes.senderNodeId);
    senderNodes.sentPingCounter++;
  }
  
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  mesh.setDebugMsgTypes (ERROR | STARTUP | CONNECTION);
  mesh.init(MESH_PREFIX, MESH_PASS, MESH_PORT);
  esp_wifi_set_protocol(WIFI_IF_AP, WIFI_PROTOCOL_LR);
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);
  mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);
  mesh.onNodeDelayReceived(&nodeTripDelay);
  
  userScheduler.addTask(taskQueryNode);
  userScheduler.addTask(taskReqQuery);
  userScheduler.addTask(taskReqLocation);
  userScheduler.enable();
  //taskReqQuery.enable();

}

void loop() {
  // put your main code here, to run repeatedly:
  mesh.update();
  userScheduler.execute();
  rssi = WiFi.RSSI();
  if(Serial.available()){
    String command = Serial.readStringUntil('\n');

    if (command == "ping"){
      senderNodes.sentPingCounter = 0;
      senderNodes.receivedPingCounter = 0;
      mesh.startDelayMeas(senderNodes.senderNodeId);
      Serial.printf("\nStarting trip delay measurement to NodeId %u", senderNodes.senderNodeId);
      senderNodes.sentPingCounter++;
    }

    if (command == "packet-loss-test"){
      Serial.printf("\nSpecify time delay in seconds for ping on serial input");
      int timeDelay = Serial.parseInt();
      while(command != "stop-test"){
        packetLossTest(timeDelay * 1000);
      }
      float packetSuccess = senderNodes.receivedPingCounter / senderNodes.sentPingCounter * 100;
      float packetLoss = 100 - packetSuccess;
      Serial.printf("\nSuccessful ping %f percent, Packet loss %f percent", packetSuccess, packetLoss);
    }
  }
  //Serial.printf("\ntaskQueryNode %s", taskQueryNode.isEnabled() ? "true" : "false");
  //Serial.printf("\ntaskReqQuery %s", taskReqQuery.isEnabled() ? "true" : "false");
}