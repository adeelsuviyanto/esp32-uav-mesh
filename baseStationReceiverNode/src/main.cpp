/*
  * ESP32-based Inter-UAV Communications System
  * Muhammad Adeel Mahdi Suviyanto
  * Telkom University
  * 
  * Base station receiver, ESP32-side
  * for joining mesh network
  * 
  * Version 1.0, 2022-08-06
  * Node 3 (base station receiver)
*/
#include <Arduino.h>
#include <HardwareSerial.h>
#include <painlessMesh.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <ArduinoJson.h>

// Mesh settings
#define MESH_PREFIX "UAV"
#define MESH_PASS "12345678"
#define MESH_PORT 5555

// Node number for current board
int nodeNumber = 3;

// Declarations
painlessMesh mesh;
HardwareSerial SerialPort(2); // Use UART2 for communications with ESP8266

// Variables
int8_t rssi;

struct{
  double latitude;
  double longitude;
  double altitude;
  uint8_t satellite;
  boolean gpsReady = false;
  int8_t senderRSSI;
  uint32_t senderNodeId;
} senderNodes;

uint32_t destinationNode;
uint32_t timestampRXTX; // stores timestamp for messages sent from receiver node to GPS sender node
uint32_t timestampTXRX; // stores timestamp for messages received from GPS sender node to receiver node
int32_t timestampDelta; // stores timestamp difference between RX-TX comms and TX-RX comms

// Function prototypes
// Send functions
void sendNodeQuery();
void sendGPSQuery();
void sendLocationReq();

// Receive functions
void receiveNodeQuery(bool nodeType, int from);
void receivedGPSQuery(bool GPSStatus);
void receivedLocation();

// PainlessMesh callbacks
void receivedCallback(uint32_t from, String &msg);
void newConnectionCallback(uint32_t nodeId);
void changedConnectionCallback();
void nodeTimeAdjustedCallback(int32_t offset);

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
  Serial.println("Sent GPS status query. Timestamp is ");
  Serial.print(timestampRXTX);
}

void sendLocationReq(){
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

void receivedNodeQuery(bool nodeType, int from){
  StaticJsonDocument<200> doc;
  if(nodeType == true){
    senderNodes.senderNodeId = from;
    Serial.printf("Sender Node ID recorded, %u", senderNodes.senderNodeId);
    doc["timestamp"] = timestampTXRX;
    doc["nodetype"] = 1;
  }
  else {
    doc["timestamp"] = timestampTXRX;
    doc["nodetype"] = 0;
  }
  serializeJson(doc, SerialPort);
  doc.clear();
}

void receivedGPSQuery(bool GPSStatus){
  StaticJsonDocument<200> doc;
  if(GPSStatus == true){
    senderNodes.gpsReady = true;
    Serial.println("Sender Node reports GPS system is ready. Commencing location data request..");
    doc["timestamp"] = timestampTXRX;
    doc["GPSReady"] = "true";
  }
  else{
    doc["timestamp"] = timestampTXRX;
    doc["GPSReady"] = "false";
  }
  serializeJson(doc, SerialPort);
  doc.clear();
}

void receivedCallback(uint32_t from, String &msg){
  uint32_t timestampReceived = mesh.getNodeTime();
  Serial.printf("\nMessage from %u msg = %s\n", from, msg.c_str());

  // Parsing JSON messages to JSON objects
  DynamicJsonDocument JsonDocument(1024 + msg.length());
  JsonDocument.clear();
  // Deserialize JSON message to extract keys and values
  deserializeJson(JsonDocument, msg);
  JsonObject receivedMsg = JsonDocument.as<JsonObject>();

  // Measure message size
  int msgSize;
  msgSize = sizeof(msg);

  // Get RSSI value from sender node
  senderNodes.senderRSSI = receivedMsg["rssi"];
  Serial.println("Sender-side RSSI: ");
  Serial.print(senderNodes.senderRSSI);

  // Message type corresponds with the query type, thus 1 is node type query, 2 is GPS status query, 3 is location requests
  uint8_t type = receivedMsg["type"];

  // Store timestamps
  timestampTXRX = receivedMsg["timestamp"];
  timestampDelta = timestampReceived - timestampTXRX;
  Serial.println(timestampDelta);

  // If received message with type 1, go to receivedNodeQuery();
  if(type == 1){
    bool nodeType = receivedMsg["nodeType"];
    receivedNodeQuery(nodeType, from);
  }
  if(type == 2){
    bool GPSStatus = receivedMsg["locationReady"];
    receivedGPSQuery(GPSStatus);
  }
  if(type == 3){
    senderNodes.latitude = receivedMsg["latitude"];
    senderNodes.longitude = receivedMsg["longitude"];
    senderNodes.altitude = receivedMsg["altitude"];
    senderNodes.satellite = receivedMsg["satellites"];
    Serial.printf("\nlat %f, lon %f, alt %f, sat %i", senderNodes.latitude, senderNodes.longitude, senderNodes.altitude, senderNodes.satellite);
    
    // Create new Static JSON Document to push to serial
    StaticJsonDocument<200> doc;
    doc["timestamp"] = timestampTXRX;
    doc["latitude"] = senderNodes.latitude;
    doc["longitude"] = senderNodes.longitude;
    doc["altitude"] = senderNodes.altitude;
    doc["satellite"] = senderNodes.satellite;
    serializeJson(doc, SerialPort);
    doc.clear();
  }
  JsonDocument.clear();
}

void newConnectionCallback(uint32_t nodeId){
  Serial.printf("New Connection, nodeId = %u\n", nodeId);
  destinationNode = nodeId;
}

void changedConnectionCallback(){
  Serial.printf("Changed connections\n");
}

void nodeTimeAdjustedCallback(int32_t offset){
  Serial.printf("Adjusted time %u. Offset = %d\n", mesh.getNodeTime(),offset);  
}

void setup() {
  Serial.begin(115200);
  SerialPort.begin(4800, SERIAL_8N1, 16, 17);
  mesh.setDebugMsgTypes(ERROR|STARTUP|CONNECTION); // PainlessMesh debug messages, output to serial
  mesh.init(MESH_PREFIX, MESH_PASS, MESH_PORT, WIFI_MODE_APSTA, 6); // Initialize mesh

  // Set ESP32 802.11 Mode
  esp_wifi_set_protocol(WIFI_IF_AP, WIFI_PROTOCOL_11N);
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11N);

  // Setup mesh callbacks
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);
  mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);
}

void loop() {
  mesh.update();
  rssi = WiFi.RSSI();
  if(SerialPort.available()){
    StaticJsonDocument<300> doc;
    uint8_t type;
    DeserializationError err = deserializeJson(doc, SerialPort);
    if(err == DeserializationError::Ok){
      type = doc["type"];
      if(type == 0) sendNodeQuery();
      if(type == 1) sendGPSQuery();
      if(type == 2) sendLocationReq();
    }
    else{
      Serial.println(err.c_str());
      while(SerialPort.available() > 0) SerialPort.read();
    }
  }
}