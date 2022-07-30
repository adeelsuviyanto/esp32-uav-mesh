/*
  * ESP32-based Inter-UAV Communications System
  * Muhammad Adeel Mahdi Suviyanto
  * Telkom University
  * 
  * 3rd receiver implementation, for flying receiver
  * 
  * Version 3.0, 2022-07-26
  * Node 2 (flying receiver)
*/
#include <Arduino.h>
#include <painlessMesh.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <TaskScheduler.h>


// Mesh settings
#define MESH_PREFIX "UAV"
#define MESH_PASS "12345678"
#define MESH_PORT 5555


// Node number for current board
int nodeNumber = 2;


// Declarations
Scheduler userScheduler;
painlessMesh mesh;


// Variables
int i = 0;
int error;

struct{
  // Location data
  double latitude;
  double longitude;
  double altitude;
  uint8_t satellite;
  boolean gpsReady = false;

  // Node data and network parameters
  uint32_t senderNodeId;
  uint8_t retryCounter;
  uint8_t sentPingCounter;
  uint8_t receivedPingCounter;
} senderNodes;

uint32_t destinationNode;
uint32_t timestampRXTX; // stores timestamp for messages sent from receiver node to GPS sender node
uint32_t timestampTXRX; // stores timestamp for messages received from GPS sender node to receiver node
uint32_t timestampDelta; // stores timestamp difference between RX-TX comms and TX-RX comms


// Function prototypes
// Send functions
void sendLocationReq();
void sendGPSQuery();
void sendNodeQuery();
void sendPingTest();

// Receive functions
void receivedLocation();
void receivedGPSQuery();
void receivedNodeQuery();

// Mesh callbacks
void receivedCallback(uint32_t from, String &msg);
void newConnectionCallback(uint32_t nodeId);
void changedConnectionCallback();
void nodeTimeAdjustedCallback(int32_t offset);
void nodeTripDelay(uint32_t nodeId, int32_t delay);


// Task Scheduler declarations
Task GPSQuery(TASK_SECOND *20, -1, &sendGPSQuery);
Task nodeQuery(TASK_SECOND * 5, -1, &sendNodeQuery);
Task locationReq(TASK_SECOND * 10, -1, &sendLocationReq);


// I will try to sort the code based on flowchart step, wish me luck.

// Node type query
// This query is to determine a new node's type, whether or not it is a GPS sender node.
void sendNodeQuery(){
  Serial.println("Querying node type...");

  // Initialize message string
  String msg;
  
  // Initialize JSON
  DynamicJsonDocument JsonDocument(1024); // Allocate enough memory for JSON document (I probably could use a much smaller memory allocation, todo.)
  JsonObject query = JsonDocument.to<JsonObject>(); // Instantiate a JSON Object named query
  timestampRXTX = mesh.getNodeTime(); // Get current node time as timestamp
  query["type"] = 1; // Messages with type 1 are for node type query
  query["timestamp"] = timestampRXTX;
  serializeJson(query, msg); // Serialize JSON Object "query" to msg as string

  // Send message
  mesh.sendSingle(destinationNode, msg); // Sends single message to destination node stored on destinationNode variable
  Serial.println("Sent node type query. Timestamp is ");
  Serial.print(timestampRXTX);

  // Clear memory from JSON Document
  JsonDocument.clear();
}

void sendGPSQuery(){
  Serial.println("Disabling node type query task...");
  if(nodeQuery.isEnabled() == true) nodeQuery.disable(); // Disables scheduled node type query as sender node should already be known to receiver

  Serial.println("Querying GPS status...");

  String msg;

  DynamicJsonDocument JsonDocument(1024);
  JsonObject query = JsonDocument.to<JsonObject>();
  timestampRXTX = mesh.getNodeTime();
  query["type"] = 2; // Messages with type 2 are for GPS status query
  query["timestamp"] = timestampRXTX;
  serializeJson(query, msg);

  mesh.sendSingle(senderNodes.senderNodeId, msg); // Sends single message to sender node stored on senderNodes.senderNodeId variable
                                                  // No longer uses destinationNode as receiver should already know that target is a sender node
  senderNodes.retryCounter++; // Count amount of times receiver tries to query GPS status from sender
  Serial.println("Sent GPS status query. Timestamp is ");
  Serial.print(timestampRXTX);

  // Disable GPS status query if there's no reply from sender for 5 requests
  if(senderNodes.retryCounter >= 5) GPSQuery.disable();
}

void sendLocationReq(){
  Serial.println("Disabling GPS status query...");
  if(GPSQuery.isEnabled() == true) GPSQuery.disable(); // Disables GPS status query, as sender should report GPS status as ready, thus receiver starts location request

  Serial.println("Requesting location data from sender node...");

  String msg;

  DynamicJsonDocument JsonDocument(1024);
  JsonObject query = JsonDocument.to<JsonObject>();
  timestampRXTX = mesh.getNodeTime();
  query["type"] = 3; //Messages with type 3 are for location requests
  query["timestamp"] = timestampRXTX;
  serializeJson(query, msg);

  mesh.sendSingle(senderNodes.senderNodeId, msg);

  Serial.println("Sent location request. Timestamp is ");
  Serial.print(timestampRXTX);
}

void sendPingTest(){

}

void setup() {
  Serial.begin(115200);
  mesh.setDebugMsgTypes(ERROR|STARTUP|CONNECTION); // PainlessMesh debug messages, output to serial
  mesh.init(MESH_PREFIX, MESH_PASS, MESH_PORT); // Initialize mesh

  // Set ESP32 802.11 Mode
  esp_wifi_set_protocol(WIFI_IF_AP, WIFI_PROTOCOL_LR);
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);

  // Setup mesh callbacks
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);
  mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);
  mesh.onNodeDelayReceived(&nodeTripDelay);
}

void loop() {
  // put your main code here, to run repeatedly:
}