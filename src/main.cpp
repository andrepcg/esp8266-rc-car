#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include "FS.h"
#include <LittleFS.h>
#include <WiFiManager.h>
#include "tof.h"
#include <WebSocketsServer.h>

#define PING_TIMEOUT                 3500
#define WS_SERVER_PORT               1235
#define WEB_SERVER_PORT              1234
#define TOF_CONTINUOUS_CYCLE_MS      100
#define SENSOR_BROADCAST_INTERVAL_MS 300

#define PIN_FORWARD D5
#define PIN_REVERSE D6
#define PIN_LEFT    D3
#define PIN_RIGHT   D4

void Initialize_sensors();
void Process_continuous_range();

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);

void handleRoot();
void handleFile(char *filename, char *contentType);

void stopAllMotors();
void broadcastSensorData();

ESP8266WebServer server(WEB_SERVER_PORT);
WebSocketsServer webSocket = WebSocketsServer(WS_SERVER_PORT);

// save timestamp of last ping
unsigned long lastPingTime = 0;
unsigned long lastSensorBroadcastTime = 0;

tofSensor_t sensors[] = {
  { false, {}, 0x30, A0, Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_SPEED, 0 },
  { false, {}, 0x31, D7, Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_SPEED, 0 },
  { false, {}, 0x32, D0, Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_SPEED, 0 }
};

const int COUNT_SENSORS = sizeof(sensors) / sizeof(sensors[0]);
const uint16_t ALL_SENSORS_PENDING = ((1 << COUNT_SENSORS) - 1);
uint16_t sensors_pending = ALL_SENSORS_PENDING;
uint32_t sensor_last_cycle_time;

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
    sensors[i].psensor.startRangeContinuous(TOF_CONTINUOUS_CYCLE_MS); // do 100ms cycle
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
  pinMode(PIN_REVERSE,  OUTPUT);
  pinMode(PIN_LEFT,     OUTPUT);
  pinMode(PIN_RIGHT,    OUTPUT);

  Serial.println("");
  Serial.print("WiFi connected. IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println("");

  // Start the web server
  server.on("/", handleRoot);

  server.begin();
  webSocket.begin();
  Serial.println("HTTP server started");
  Serial.println("WS server started");

  webSocket.onEvent(webSocketEvent);

  startSensorsContinuous();
}



void loop() {
  Process_continuous_range();

  // dead-man switch. if last command sent by the browser is to accelerate but no ping is received,
  // let's stop all motors to prevent the car running away into the sunset
  if (millis() - lastPingTime > PING_TIMEOUT) {
    stopAllMotors();
  }

  broadcastSensorData();

  server.handleClient();
  webSocket.loop();
}

void handleRoot() {
  handleFile("/index.html", "text/html");
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


void broadcastSensorData() {
  // dont send too often
  if (millis() - lastSensorBroadcastTime < SENSOR_BROADCAST_INTERVAL_MS) {
    return;
  }

  // let's encode the sensor data in binary format in the following way: [sensor1.id, sensor1.range, sensor2.id, sensor2.range, ...]
  int length = (COUNT_SENSORS * 3) + 1;
  uint8_t data[length];
  data[0] = 0x02; // message type

  // [0x2,0x30,0xff,0xff,0x31,0xff,0xff,0x32,0xff,0xff]

  for (int i = 0; i < COUNT_SENSORS; i++) {
    data[i * 3 + 1] = sensors[i].id;
    // we have a 16-bit value to send, so we need to split it into two 8-bit values
    data[i * 3 + 1 + 1] = sensors[i].last_range >> 8;
    data[i * 3 + 2 + 1] = sensors[i].last_range & 0xFF;
  }
  webSocket.broadcastBIN(data, sizeof(uint8_t) * length);
  lastSensorBroadcastTime = millis();
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
    if (sensors[i].psensor.begin(sensors[i].id, false, &Wire, sensors[i].sensor_config)) {
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

      if (sensors[i].started && sensors[i].psensor.isRangeComplete()) {
        sensors[i].last_range = sensors[i].psensor.readRangeResult();
        sensors[i].sensor_status = sensors[i].psensor.readRangeStatus();
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

void wsHandleForward(int speed = 0) {
  if (speed == 0) {
    analogWrite(PIN_FORWARD, 0);
    analogWrite(PIN_REVERSE, 0);
  } else {
    analogWrite(PIN_FORWARD, map(speed, 0, 255, 0, 255));
    analogWrite(PIN_REVERSE, 0);
  }
}

void wsHandleBackward(int speed = 0) {
  if (speed) {
    analogWrite(PIN_REVERSE, 0);
    analogWrite(PIN_FORWARD, 0);
  } else {
    analogWrite(PIN_FORWARD, 0);
    analogWrite(PIN_REVERSE, map(speed, 0, 255, 0, 255));
  }
}

void wsHandleLeft(int speed = 0) {
  if (speed == 0) {
    digitalWrite(PIN_LEFT, LOW);
    digitalWrite(PIN_RIGHT, LOW);
  } else {
    digitalWrite(PIN_LEFT, HIGH);
    digitalWrite(PIN_RIGHT, LOW);
  }
}

void wsHandleRight(int speed = 0) {
  if (speed == 0) {
    digitalWrite(PIN_LEFT, LOW);
    digitalWrite(PIN_RIGHT, LOW);
  } else {
    digitalWrite(PIN_LEFT, LOW);
    digitalWrite(PIN_RIGHT, HIGH);
  }
}

void onBinary(uint8_t *binary, uint8_t num) {
  switch(binary[0]) {
    case 0x01: // PING
      lastPingTime = millis();
      webSocket.sendBIN(num, binary, 1);
      break;

    case 0x2: // motor forward
      wsHandleForward(binary[1]);
      break;

    case 0x3: // motor reverse
      wsHandleBackward(binary[1]);
      break;

    case 0x4: // motor left
      wsHandleLeft(binary[1]);
      break;

    case 0x5: // motor right
      wsHandleRight(binary[1]);
      break;

    case 0x6: // motor stop
      stopAllMotors();
      break;
  }
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Disconnected!\n", num);
      stopAllMotors();
      break;

    case WStype_BIN:
      onBinary((uint8_t *)payload, num);
      break;

    case WStype_TEXT:
      String data = (char *)payload;
      Serial.println("Received text: " + data);
      break;
  }
}
