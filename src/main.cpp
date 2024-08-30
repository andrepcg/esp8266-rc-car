#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include "FS.h"
#include <LittleFS.h>
#include <WiFiManager.h>

int PING_TIMEOUT = 2000; // stops all motors if no ping is received in 2 seconds

int PIN_FORWARD = D1;
int PIN_BACKWARD = D2;
int PIN_LEFT = D3;
int PIN_RIGHT = D4;

void handleForward();
void handleBackward();
void handleLeft();
void handleRight();
void handleStop();
void handleRoot();
void handlePing();
void handleFile(char *filename, char *contentType);

void handleStopForward();
void handleStopBackward();
void handleStopLeft();
void handleStopRight();
void stopAllMotors();
int getSpeedValue();

ESP8266WebServer server(1234);

// save timestamp of last ping
unsigned long lastPing = 0;

void setup() {
  Serial.begin(115200);

  WiFiManager wm;
  wm.setHostname("rc-car");
  wm.autoConnect("RC-CAR");

  if(!LittleFS.begin()){
    Serial.println("Not able to mount LittleFS");
    return;
  }

  pinMode(PIN_FORWARD,  OUTPUT);
  pinMode(PIN_BACKWARD, OUTPUT);
  pinMode(PIN_LEFT,     OUTPUT);
  pinMode(PIN_RIGHT,    OUTPUT);

  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // Start the web server
  server.on("/", handleRoot);
  server.on("/ping", handlePing);
  server.on("/forward", handleForward);
  server.on("/backward", handleBackward);
  server.on("/left", handleLeft);
  server.on("/right", handleRight);
  server.on("/stopForward", handleStopForward);
  server.on("/stopBackward", handleStopBackward);
  server.on("/stopLeft", handleStopLeft);
  server.on("/stopRight", handleStopRight);

  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  // dead-man switch. if last command sent by the browser is to accelerate but no ping is received,
  // let's stop all motors to prevent the car running away into the sunset
  if (millis() - lastPing > PING_TIMEOUT) {
    stopAllMotors();
  }

  server.handleClient();
}

void handleRoot() {
  handleFile("/index.html", "text/html");
}

// Motor control handlers
void handleForward() {
  analogWrite(PIN_FORWARD, getSpeedValue());
  analogWrite(PIN_BACKWARD, 0);
  server.send(200, "text/plain", "Moving Forward");
}

void handleBackward() {
  analogWrite(PIN_FORWARD, 0);
  analogWrite(PIN_BACKWARD, getSpeedValue());
  server.send(200, "text/plain", "Moving Backward");
}

void handleLeft() {
  digitalWrite(PIN_LEFT, HIGH);
  digitalWrite(PIN_RIGHT, LOW);
  server.send(200, "text/plain", "Turning Left");
}

void handleRight() {
  digitalWrite(PIN_LEFT, LOW);
  digitalWrite(PIN_RIGHT, HIGH);
  server.send(200, "text/plain", "Turning Right");
}

void handleStopForward() {
  analogWrite(PIN_FORWARD, 0);
  server.send(200, "text/plain", "Stopped forward");
}

void handleStopBackward() {
  analogWrite(PIN_BACKWARD, 0);
  server.send(200, "text/plain", "Stopped backward");
}

void handleStopLeft() {
  digitalWrite(PIN_LEFT, LOW);
  server.send(200, "text/plain", "Stopped left");
}

void handleStopRight() {
  digitalWrite(PIN_RIGHT, LOW);
  server.send(200, "text/plain", "Stopped right");
}

void handlePing() {
  lastPing = millis();
  server.send(200, "text/plain", "Pong");
}

void stopAllMotors() {
  analogWrite(PIN_FORWARD, 0);
  analogWrite(PIN_BACKWARD, 0);
  digitalWrite(PIN_LEFT, LOW);
  digitalWrite(PIN_RIGHT, LOW);
}

void handleFile(char *filename, char *contentType) {
  File f = LittleFS.open(filename, "r");
  server.streamFile(f, contentType);
  f.close();
}

// takes the speed value from the speed query param. The value is a percentage.
// This functions maps linearly the percentage to the 0-255 range.
int getSpeedValue() {
  String value = server.arg("speed");
  int speed = value.toInt();
  return map(speed, 0, 100, 0, 255);
}
