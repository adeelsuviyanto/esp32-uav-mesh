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
#include <ArduinoJson.h>
#include <FS.h>
#include <SPI.h>
#include <SD.h>


// Mesh settings
#define MESH_PREFIX "UAV"
#define MESH_PASS "12345678"
#define MESH_PORT 5555

// SD Card settings
#define SD_CS 5

// Node number for current board
int nodeNumber = 2;


// Declarations
Scheduler userScheduler;
painlessMesh mesh;


// Variables
int i = 0;
int error;
char buffer[128];
int8_t rssi;

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
  uint8_t gpsReceivedCounter;
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
void receivedGPSQuery(bool GPSStatus);
void receivedNodeQuery(bool nodeType, int from);

// Print data to MicroSD
void storeData();

// Counts packet loss
void packetLossCounter();

// Mesh callbacks
void receivedCallback(uint32_t from, String &msg);
void newConnectionCallback(uint32_t nodeId);
void changedConnectionCallback();
void nodeTimeAdjustedCallback(int32_t offset);
void nodeTripDelay(uint32_t nodeId, int32_t delay);

// SD Card functions
void appendFile(fs::FS &fs, const char * path, const char * message);
void writeFile(fs::FS &fs, const char * path, const char * message);

// Task Scheduler declarations
Task GPSQuery(TASK_SECOND *20, -1, &sendGPSQuery);
Task nodeQuery(TASK_SECOND * 5, -1, &sendNodeQuery);
Task locationReq(TASK_SECOND * 2, -1, &sendLocationReq);
Task pingTest(TASK_SECOND, -1, &sendPingTest);
//Task packetLoss(TASK_SECOND, -1, &packetLossCounter);


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
  // This function is to send a ping test based on the startDelayMeas() function of PainlessMesh
  // This function should be activated by a Task Scheduler
  Serial.println("Sending ping test...");
  mesh.startDelayMeas(senderNodes.senderNodeId);
  senderNodes.sentPingCounter++;
  if(senderNodes.sentPingCounter % 10 == 0 && senderNodes.sentPingCounter != 0){
    float packetLoss = senderNodes.receivedPingCounter / senderNodes.sentPingCounter;
    snprintf(buffer, sizeof(buffer), "%f percent packet loss\n", (1.0 - packetLoss) * 100.0);
    appendFile(SD, "/packetLoss.txt", buffer);
  }
}

void nodeTripDelay(uint32_t nodeId, int32_t delay){
  Serial.println("Round-trip delay: ");
  Serial.print(delay);
  senderNodes.receivedPingCounter++;
  Serial.printf("\n%i received ping counter, %i sent ping counter\n", senderNodes.receivedPingCounter, senderNodes.sentPingCounter);
  snprintf(buffer, sizeof(buffer), "%i us, %i ms, %ld\n", delay, delay / 1000, millis());
  appendFile(SD, "/delay.txt", buffer);
}

void packetLossCounter(){
  
}

void newConnectionCallback(uint32_t nodeId){
  Serial.printf("New Connection, nodeId = %u\n", nodeId);
  destinationNode = nodeId;

  if(nodeQuery.isEnabled() == false && pingTest.isEnabled() == false){
    nodeQuery.enable();
  }
  else if(pingTest.isEnabled() == false) sendNodeQuery();
  else if(pingTest.isEnabled() == true) return;
}

void receivedCallback(uint32_t from, String &msg){
  uint32_t timestampReceived = mesh.getNodeTime();
  Serial.printf("\nMessage from %u msg = %s\n", from, msg.c_str());
  uint8_t senderRSSI;

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
  senderRSSI = receivedMsg["rssi"];
  Serial.println("Sender-side RSSI: ");
  Serial.print(senderRSSI);

  // Message type corresponds with the query type, thus 1 is node type query, 2 is GPS status query, 3 is location requests
  uint8_t type = receivedMsg["type"];

  // Store timestamps
  timestampTXRX = receivedMsg["timestamp"];
  timestampDelta = abs(timestampReceived - timestampTXRX);
  Serial.println(timestampDelta);
  
  // Calculate incidental speed of transmission (throughput)
  float deltaSeconds = timestampDelta / 1000000.0;
  float throughput = msgSize / deltaSeconds;
  Serial.println(deltaSeconds, 6);
  Serial.println(msgSize);
  Serial.println("Throughput: ");
  Serial.print(throughput, 6);
  Serial.print(" bps");
  snprintf(buffer, sizeof(buffer), "%f B/s %ld %i dBm %i dBm\n", throughput, millis(), senderRSSI, rssi);
  appendFile(SD, "/throughput.txt", buffer);

  // If received message with type 1, go to receivedNodeQuery();
  if(pingTest.isEnabled() == false){
    if(type == 1){
      bool nodeType = receivedMsg["nodeType"];
      receivedNodeQuery(nodeType, from);
    }
    if(type == 2){
      bool GPSStatus = receivedMsg["locationReady"];
      receivedGPSQuery(GPSStatus);
    }
    if(type == 3){
      if(nodeQuery.isEnabled() == true) nodeQuery.disable();
      senderNodes.latitude = receivedMsg["latitude"];
      senderNodes.longitude = receivedMsg["longitude"];
      senderNodes.altitude = receivedMsg["altitude"];
      senderNodes.satellite = receivedMsg["satellites"];
      Serial.printf("\nlat %f, lon %f, alt %f, sat %i", senderNodes.latitude, senderNodes.longitude, senderNodes.altitude, senderNodes.satellite);
      senderNodes.gpsReceivedCounter++;
      snprintf(buffer, sizeof(buffer), "\n%f    %f    %f    %i", senderNodes.latitude, senderNodes.longitude, senderNodes.altitude, senderNodes.satellite);
      appendFile(SD, "/location.txt", buffer);
    }
  }
  JsonDocument.clear();
}

void receivedNodeQuery(bool nodeType, int from){
  if(nodeType == true){
    senderNodes.senderNodeId = from;
    Serial.printf("Sender Node ID recorded, %u", senderNodes.senderNodeId);
    GPSQuery.enable();
  }
  else return;
}

void receivedGPSQuery(bool GPSStatus){
  if(GPSStatus == true){
    nodeQuery.disable();
    senderNodes.gpsReady = true;
    locationReq.enable();
    Serial.println("Sender Node reports GPS system is ready. Commencing location data request..");
  }
  else return;
}

void changedConnectionCallback(){
  Serial.printf("Changed connections\n");
  if(nodeQuery.isEnabled() == false) nodeQuery.enable();
  else sendNodeQuery();
}

void nodeTimeAdjustedCallback(int32_t offset){
  Serial.printf("Adjusted time %u. Offset = %d\n", mesh.getNodeTime(),offset);  
}

void writeFile(fs::FS &fs, const char * path, const char * message){
  Serial.printf("Writing file: %s\n", path);

  // Opens a path to file
  File file = fs.open(path, FILE_WRITE);
  if(!file){
    Serial.println("Failed to open file for writing");
    return;
  }
  if(file.print(message)) Serial.println("File written.");
  else Serial.println("Write failed.");
  file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message){
  Serial.printf("Appending file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if(!file){
    Serial.println("Failed to open file for appending.");
    return;
  }
  if(file.print(message)) Serial.printf("Message %s appended.", message);
  else Serial.println("Append failed.");
  file.close();
}

void setup() {
  Serial.begin(115200);
  mesh.setDebugMsgTypes(ERROR|STARTUP|CONNECTION); // PainlessMesh debug messages, output to serial
  mesh.init(MESH_PREFIX, MESH_PASS, MESH_PORT); // Initialize mesh

  // Set ESP32 802.11 Mode
  esp_wifi_set_protocol(WIFI_IF_AP, WIFI_PROTOCOL_11N);
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11N);

  // Setup mesh callbacks
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);
  mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);
  mesh.onNodeDelayReceived(&nodeTripDelay);

  // Task scheduler tasks
  userScheduler.addTask(nodeQuery);
  userScheduler.addTask(GPSQuery);
  userScheduler.addTask(locationReq);
  userScheduler.enable();
  userScheduler.addTask(pingTest);
  //userScheduler.addTask(packetLoss);

  // SD card initialization
  if(!SD.begin(SD_CS)){
    Serial.println("SD Card mount failed! Check wiring and ensure an SD card is inserted.");
    return;
  }
  uint8_t cardType = SD.cardType();
  if(cardType == CARD_NONE) Serial.println("No SD card detected.");
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card size %lluMB\n", cardSize);

  // Initialize files
  // Here we use append instead of write in order to prevent overwrites
  appendFile(SD, "/throughput.txt", "Start throughput recording, timestamp in ms.\nThroughput Timestamp SenderRSSI ClientRSSI\n");

  // Debug code for SD
  // Delete if necessary
  writeFile(SD, "/timestamp.txt", "Start timestamp recording");

  // Write new file for location data
  writeFile(SD, "/location.txt", "Latitude    Longitude    Altitude    Satellite");
}

void loop() {
  mesh.update();
  userScheduler.execute();
  rssi = WiFi.RSSI();

  // Debug code for SD
  // Delete if necessary
  if((millis() % 10000) == 0){
    snprintf(buffer, sizeof(buffer), "%ld\n", millis()/1000);
    appendFile(SD, "/timestamp.txt", buffer);
  }
  if(senderNodes.gpsReceivedCounter >= 10 && millis() >= 200000 && (pingTest.isEnabled() == false)){
    nodeQuery.disable();
    locationReq.disable();
    GPSQuery.disable();
    pingTest.enable();
    //packetLoss.enable();
  }
}