#include "Arduino.h"
#include <SoftwareSerial.h>
#include <ArduinoJson.h>

//Define software serial pins for ESP8266
SoftwareSerial espSerial(3, 2); // RX, TX

//Define sensor and appliance pins
int gasSensorPin = A0;
int waterLevelSensorPin = A1;
int waterPumpPin = 8;
int light1Pin = 9;
int light2Pin = 10;
int fanPin = 11;

//Define variables to store sensor values and appliance states
int gasSensorValue;
int waterLevelSensorValue;
int waterPumpState;
int light1State;
int light2State;
int fanState;

//Define flags for controlling all appliances
bool toggleAllFlag = false;

void setup() {
  //Initialize serial communication
  Serial.begin(9600);
  espSerial.begin(9600);

  //Configure pins
  pinMode(waterPumpPin, OUTPUT);
  pinMode(light1Pin, OUTPUT);
  pinMode(light2Pin, OUTPUT);
  pinMode(fanPin, OUTPUT);
}

void loop() {
  //Read sensor values
  gasSensorValue = analogRead(gasSensorPin);
  waterLevelSensorValue = analogRead(waterLevelSensorPin);

  //Read toggleAllFlag from software serial
  if (espSerial.available() > 0) {
    String message = espSerial.readStringUntil('\n');
    if (message == "toggle_all_on") {
      toggleAllFlag = true;
    } else if (message == "toggle_all_off") {
      toggleAllFlag = false;
    }
  }

  //Control water pump based on water level sensor
  if (waterLevelSensorValue < 500) {
    digitalWrite(waterPumpPin, HIGH);
    waterPumpState = 1;
  } else {
    digitalWrite(waterPumpPin, LOW);
    waterPumpState = 0;
  }

  //Control lights and fan based on gas sensor
  if (gasSensorValue > 500) {
    digitalWrite(light1Pin, HIGH);
    digitalWrite(light2Pin, LOW);
    digitalWrite(fanPin, HIGH);
    light1State = 1;
    light2State = 0;
    fanState = 1;
  } else {
    digitalWrite(light1Pin, LOW);
    digitalWrite(light2Pin, HIGH);
    digitalWrite(fanPin, LOW);
    light1State = 0;
    light2State = 1;
    fanState = 0;
  }

  //Create JSON object with sensor values and appliance states
  DynamicJsonDocument jsonDoc(200);
  jsonDoc["gasSensorValue"] = gasSensorValue;
  jsonDoc["waterLevelSensorValue"] = waterLevelSensorValue;
  jsonDoc["waterPumpState"] = waterPumpState;
  jsonDoc["light1State"] = light1State;
  jsonDoc["light2State"] = light2State;
  jsonDoc["fanState"] = fanState;
  jsonDoc["toggleAllFlag"] = toggleAllFlag;

  //Serialize JSON object to a string and send to ESP8266 over software serial
  String jsonString;
  serializeJson(jsonDoc, jsonString);
  espSerial.println(jsonString);

  //Print sensor values and appliance states to serial monitor
  Serial.print("Gas Sensor Value: ");
  Serial.println(gasSensorValue);
  Serial.print("Water Level Sensor Value: ");
 Serial.println(waterLevelSensorValue);
Serial.print("Water Pump State: ");
Serial.println(waterPumpState);
Serial.print("Light 1 State: ");
Serial.println(light1State);
Serial.print("Light 2 State: ");
Serial.println(light2State);
Serial.print("Fan State: ");
Serial.println(fanState);
Serial.print("Toggle All Flag: ");
Serial.println(toggleAllFlag);

//Delay for 1 second
delay(1000);
}
