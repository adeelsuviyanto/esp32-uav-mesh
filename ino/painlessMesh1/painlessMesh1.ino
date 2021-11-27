/*
 * ESP32-based Inter-UAV Communications System
 * Muhammad Adeel Mahdi Suviyanto
 * Telkom University
 * 
 * 2nd GPS implementation
 * 
 * Version 0.3, 2021-11-23
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
void sendGPSFail();
String getLocation();

//As of v0.3 (2021-11-23), placeholder data has been removed
double currentLatitude;
double currentLongitude;
double currentAltitude;
int currentSatellites;
int currentGPSDate;
int currentGPSTime;

//Send repetitively via Task Scheduler
//Send location data every 5 seconds
Task taskSendMessage(TASK_SECOND * 5, TASK_FOREVER, &sendMessage);

//Get Location Data from GPS board
String getLocation(){
  //Parse data from GPS module
  currentLatitude = (gps.location.lat());
  Serial.println(currentLatitude);
  currentLongitude = (gps.location.lng());
  Serial.println(currentLongitude);
  currentAltitude = (gps.altitude.meters());
  Serial.println(currentAltitude);
  currentSatellites = (gps.satellites.value());
  Serial.println(currentSatellites);
  currentGPSDate = (gps.date.value());
  Serial.println(currentGPSDate);
  currentGPSTime = (gps.time.value());
  Serial.println(currentGPSTime);
  
  /*
  //GPS date
  String GPSYear = String((gps.date.year()));
  String GPSMonth = String((gps.date.month()));
  String GPSDay = String((gps.date.day()));
  currentGPSDate = GPSYear + "-" + GPSMonth + "-" + GPSDay;

  //GPS time
  String GPSHour = String((gps.time.hour()));
  String GPSMinute = String((gps.time.minute()));
  String GPSSecond = String((gps.time.second()));
  currentGPSTime = GPSHour + ":" + GPSMinute + ":" + GPSSecond;
  */
  
  JSONVar jsonMsg;
  jsonMsg["error"] = 0;
  jsonMsg["node"] = nodeNumber;
  jsonMsg["latitude"] = currentLatitude;
  jsonMsg["longitude"] = currentLongitude;
  jsonMsg["altitude"] = currentAltitude;
  jsonMsg["satellites"] = currentSatellites;
  jsonMsg["date"] = currentGPSDate;
  jsonMsg["time"] = currentGPSTime;
  locationData = JSON.stringify(jsonMsg);
  return locationData;
}

void sendMessage(){
  String msg = getLocation();
  mesh.sendBroadcast(msg);
}

void sendGPSFail(){
  String msg;
  JSONVar jsonMsg;
  jsonMsg["error"] = 1;
  msg = JSON.stringify(jsonMsg);
  mesh.sendBroadcast(msg);
}

//painlessMesh serial outputs

void receivedCallback(uint32_t from, String &msg){
  Serial.printf("Message from %u msg=%s\n", from, msg.c_str());
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
  mesh.update();
  while(ss.available() > 0){
    gps.encode(ss.read());
  }
  //GPS Check
  if(millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println("Error: No GPS data received, check cabling!");
    Serial.println("GPS stream dump: ");
    while(true)
    if(ss.available() > 0) Serial.write(ss.read());
  }
  /*  
  if(gps.location.isUpdated()){
    taskSendMessage.enable();    
  } */ 
}
