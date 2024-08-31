#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include "FS.h"
#include <LittleFS.h>
#include <WiFiManager.h>
#include "tof.h"
#include <WebSocketsServer.h>

Adafruit_VL53L0X front_lox;
Adafruit_VL53L0X left_lox;
Adafruit_VL53L0X right_lox;


tofSensor_t frontSensor = { "front_distance", false, &front_lox, 0x30, A0, Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_SPEED, 0 };
tofSensor_t leftSensor  = { "left_distance",  false, &left_lox,  0x31, D7, Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_SPEED, 0 };
tofSensor_t rightSensor = { "right_distance", false, &right_lox, 0x32, D0, Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_SPEED, 0 };

tofSensor_t sensors[] = { frontSensor, leftSensor, rightSensor };
const int COUNT_SENSORS = sizeof(sensors) / sizeof(sensors[0]);
const uint16_t ALL_SENSORS_PENDING = ((1 << COUNT_SENSORS) - 1);
uint16_t sensors_pending = ALL_SENSORS_PENDING;
uint32_t sensor_last_cycle_time;


int PING_TIMEOUT = 3500; // stops all motors if no ping is received in 2 seconds

int PIN_FORWARD = D5;
int PIN_REVERSE = D6;
int PIN_LEFT = D3;
int PIN_RIGHT = D4;

void Initialize_sensors();
void Process_continuous_range();

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);

void handleRoot();
void handlePing();
void handleFile(char *filename, char *contentType);

void stopAllMotors();
void broadcastSensorData();

ESP8266WebServer server(1234);
WebSocketsServer webSocket = WebSocketsServer(1235);

// save timestamp of last ping
unsigned long lastPing = 0;

void initVL53L0X() {
  Serial.println(F("VL53LOX_multi start, initialize IO pins"));
  for (int i = 0; i < COUNT_SENSORS; i++) {
    pinMode(sensors[i].shutdown_pin, OUTPUT);
    digitalWrite(sensors[i].shutdown_pin, LOW);
  }
  Serial.println(F("Starting..."));
  Initialize_sensors();
}

void startSensorsContinuous() {
  Serial.println("Starting sensors in continuous mode");
  for (uint8_t i = 0; i < COUNT_SENSORS; i++) {
    // skip if not started
    if (!sensors[i].started) {
      continue;
    }
    sensors[i].psensor->startRangeContinuous(100); // do 100ms cycle
  }
}

void setup() {
  Serial.begin(115200);

  while (! Serial) {
    delay(1);
  }

  Wire.begin();

  initVL53L0X();

  WiFiManager wm;
  wm.setHostname("rc-car");
  wm.autoConnect("RC-CAR");

  if(!LittleFS.begin()){
    Serial.println("Not able to mount LittleFS");
    return;
  }

  pinMode(PIN_FORWARD,  OUTPUT);
  pinMode(PIN_REVERSE, OUTPUT);
  pinMode(PIN_LEFT,     OUTPUT);
  pinMode(PIN_RIGHT,    OUTPUT);

  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // Start the web server
  server.on("/", handleRoot);

  server.begin();
  Serial.println("HTTP server started");

  webSocket.begin();
  Serial.println("WS server started");
  webSocket.onEvent(webSocketEvent);

  startSensorsContinuous();
}



void loop() {
  Process_continuous_range();

  // dead-man switch. if last command sent by the browser is to accelerate but no ping is received,
  // let's stop all motors to prevent the car running away into the sunset
  if (millis() - lastPing > PING_TIMEOUT) {
    stopAllMotors();
  }

  // send sensor data to all clients every 100ms
  broadcastSensorData();


  server.handleClient();
  webSocket.loop();
}

void handleRoot() {
  handleFile("/index.html", "text/html");
}

void handlePing() {
  lastPing = millis();
  server.send(200);
}

void stopAllMotors() {
  analogWrite(PIN_FORWARD, 0);
  analogWrite(PIN_REVERSE, 0);
  digitalWrite(PIN_LEFT, LOW);
  digitalWrite(PIN_RIGHT, LOW);
}

void handleFile(char *filename, char *contentType) {
  File f = LittleFS.open(filename, "r");
  server.streamFile(f, contentType);
  f.close();
}

const int SENSOR_BROADCAST_INTERVAL = 300;
int lastSensorBroadcast = 0;

void broadcastSensorData() {
  if (millis() - lastSensorBroadcast < SENSOR_BROADCAST_INTERVAL) {
    return;
  }

  String response = "SENSORS#";
  for (int i = 0; i < COUNT_SENSORS; i++) {
    response += sensors[i].name + ":" + String(sensors[i].last_range) + ";";
  }
  webSocket.broadcastTXT(response);
  lastSensorBroadcast = millis();
}

void Initialize_sensors() {
  bool found_any_sensors = false;
  // Set all shutdown pins low to shutdown sensors
  for (int i = 0; i < COUNT_SENSORS; i++)
    digitalWrite(sensors[i].shutdown_pin, LOW);
  delay(10);

  for (int i = 0; i < COUNT_SENSORS; i++) {
    // one by one enable sensors and set their ID
    digitalWrite(sensors[i].shutdown_pin, HIGH);
    delay(50); // give time to wake up.
    if (sensors[i].psensor->begin(sensors[i].id, false, &Wire, sensors[i].sensor_config)) {
      found_any_sensors = true;
      Serial.print("Sensor ");
      Serial.print(sensors[i].id, HEX);
      Serial.println(" started");
      sensors[i].started = true;
    } else {
      Serial.print(sensors[i].id, HEX);
      Serial.print(F(": failed to start\n"));
    }
  }
  if (!found_any_sensors) {
    Serial.println("No valid sensors found");
    while (1)
      ;
  }
}

void Process_continuous_range() {

  uint16_t mask = 1;
  for (uint8_t i = 0; i < COUNT_SENSORS; i++) {
    if (sensors_pending & mask) {

      if (sensors[i].started && sensors[i].psensor->isRangeComplete()) {
        sensors[i].last_range = sensors[i].psensor->readRangeResult();
        sensors[i].sensor_status = sensors[i].psensor->readRangeStatus();
        sensors_pending ^= mask;
      }
    }
    mask <<= 1; // setup to test next one
  }
  // See if we have all of our sensors read OK
  uint32_t delta_time = millis() - sensor_last_cycle_time;
  if (!sensors_pending || (delta_time > 1000)) {
    // Serial.print(delta_time, DEC);
    // Serial.print(F("("));
    // Serial.print(sensors_pending, HEX);
    // Serial.print(F(")"));
    mask = 1;

    sensor_last_cycle_time = millis();
    sensors_pending = ALL_SENSORS_PENDING;
  }
}


// data: MOTOR:FORWARD:on:100
int getSpeedFromString(String &data) {
  int index = data.indexOf(":on");
  String speed = data.substring(index + 4);
  return map(speed.toInt(), 0, 100, 0, 255);
}

void wsHandleForward(String &data, bool stop = false) {
  if (stop) {
    analogWrite(PIN_FORWARD, 0);
    analogWrite(PIN_REVERSE, 0);
  } else {

    analogWrite(PIN_FORWARD, getSpeedFromString(data));
    analogWrite(PIN_REVERSE, 0);
  }
}

void wsHandleBackward(String &data, bool stop = false) {
  if (stop) {
    analogWrite(PIN_REVERSE, 0);
    analogWrite(PIN_FORWARD, 0);
  } else {
    analogWrite(PIN_FORWARD, 0);
    analogWrite(PIN_REVERSE, getSpeedFromString(data));
  }
}

void wsHandleLeft(bool stop = false) {
  if (stop) {
    digitalWrite(PIN_LEFT, LOW);
    digitalWrite(PIN_RIGHT, LOW);
  } else {
    digitalWrite(PIN_LEFT, HIGH);
    digitalWrite(PIN_RIGHT, LOW);
  }
}

void wsHandleRight(bool stop = false) {
  if (stop) {
    digitalWrite(PIN_LEFT, LOW);
    digitalWrite(PIN_RIGHT, LOW);
  } else {
    digitalWrite(PIN_LEFT, LOW);
    digitalWrite(PIN_RIGHT, HIGH);
  }
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      break;
    case WStype_TEXT:
      String data = (char *)payload;

      if (data == "PING") {
        lastPing = millis();
        webSocket.sendTXT(num, "PONG");
      }

      if (data.startsWith("MOTOR")) {
        if (data.startsWith("MOTOR:FORWARD:on")) {
          wsHandleForward(data, false);
        } else if (data == "MOTOR:FORWARD:off") {
          wsHandleForward(data, true);
        } else if (data.startsWith("MOTOR:REVERSE:on")) {
          wsHandleBackward(data, false);
        } else if (data == "MOTOR:REVERSE:off") {
          wsHandleBackward(data, true);
        } else if (data == "MOTOR:LEFT:on") {
          wsHandleLeft(false);
        } else if (data == "MOTOR:LEFT:off") {
          wsHandleLeft(true);
        } else if (data == "MOTOR:RIGHT:on") {
          wsHandleRight(false);
        } else if (data == "MOTOR:RIGHT:off") {
          wsHandleRight(true);
        } else if (data == "MOTOR:STOP") {
          stopAllMotors();
        }
      }

      break;
  }

}
