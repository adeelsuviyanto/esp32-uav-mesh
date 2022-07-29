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

void requestLocation();
void queryGPSStatus();
void queryNodeType();

void setup() {
  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly:
}