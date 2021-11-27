/*
 * ESP32-based Inter-UAV Communications System
 * Muhammad Adeel Mahdi Suviyanto
 * Telkom University
 * 
 * Receiver Node
 * 
 * Version 0.1, 2021-11-19
 * Node 2 (receiver)
 */


#include "painlessMesh.h"
#include <Arduino_JSON.h>
#include <WiFi.h>

//Mesh details
#define MESH_PREFIX "UAV" //name of mesh
#define MESH_PASS "12345678" //mesh password
#define MESH_PORT 5555

//Node number for current board
int nodeNumber = 2;

//Data string to send
//String locationData;

//Declarations
Scheduler userScheduler;
painlessMesh mesh;
void sendMessage();
//String getLocation();

//Placeholder data, to clear later and replace const float to float
//const float currentLatitude = -6.97689;
//const float currentLongitude = 107.62975;
//const float currentAltitude = 30.00000;

//Send repetitively via Task Scheduler
//Task taskSendMessage(TASK_SECOND * 10, TASK_FOREVER, &sendMessage);

//Get Location Data from GPS board (NOT YET IMPLEMENTED, USING PLACEHOLDER DATA)
/*String getLocation(){
  JSONVar jsonReadings;
  jsonReadings["node"] = nodeNumber;
  jsonReadings["latitude"] = currentLatitude;
  jsonReadings["longitude"] = currentLongitude;
  jsonReadings["altitude"] = currentAltitude;
  locationData = JSON.stringify(jsonReadings);
  return locationData;
}*/

void sendMessage(uint32_t senderNodeId){
  String msg = "OK - Received Location Data from ";
  msg = msg + senderNodeId;
  mesh.sendBroadcast(msg);
}

//painlessMesh serial outputs
/*void receivedCallback(uint32_t from, String &msg){
  Serial.printf("Message from %u msg=%s\n", from, msg.c_str());
  
}*/
void receivedCallback(uint32_t from, String &msg){
  Serial.printf("Message from %u msg=%s\n", from, msg.c_str());
  JSONVar myObject = JSON.parse(msg.c_str());
  int node = myObject["node"];
  double latitude = myObject["latitude"];
  double longitude = myObject["longitude"];
  double altitude = myObject["altitude"];
  Serial.print("Node: ");
  Serial.println(node);
  Serial.print("Latitude: ");
  Serial.println(latitude, 6);
  Serial.print("Longitude: ");
  Serial.println(longitude, 6);
  Serial.print("Altitude: ");
  Serial.println(altitude);
  long rssi = WiFi.RSSI();
  Serial.print("RSSI: ");
  Serial.println(rssi);
  sendMessage(from);
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
  mesh.setDebugMsgTypes( ERROR | STARTUP );
  mesh.init( MESH_PREFIX, MESH_PASS, &userScheduler, MESH_PORT );
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);
  mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);

  //userScheduler.addTask(taskSendMessage);
  //taskSendMessage.enable();

}

void loop() {
  // put your main code here, to run repeatedly:
  mesh.update();
}
