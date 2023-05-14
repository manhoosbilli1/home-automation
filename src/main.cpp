#include "Arduino.h"
#include <SoftwareSerial.h>
#include <ArduinoJson.h>
#include "DHT.h"
#include <NewPing.h>
#include <MQUnifiedsensor.h>

#define TRIGGER_PIN  13  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     12  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.


//Define software serial pins for ESP8266
SoftwareSerial espSerial(3, 2); // RX, TX

//Define sensor and appliance pins
uint8_t gasSensorPin = A0;
uint8_t waterLevelSensorPin = A1;
uint8_t waterPumpPin = 10;
uint8_t lightPin = 9;
uint8_t fanPin = 11;
uint8_t lockPin = 8;
#define DHTPININSIDE A0
#define DHTPINOUTSIDE A1

//Define variables to store sensor values and appliance states
uint8_t gasSensorValue;
uint8_t waterLevelSensorValue;
bool waterPumpState;
bool lightState;
bool fanState;
bool lockState; 
bool updated;  //keeps track of things that changed. 
uint8_t tempInside, tempOutside, humInside, humOutside;


//Define flags for controlling all appliances
bool toggleAllFlag = false;

//construct objects of dht
#define DHTTYPE DHT11   // DHT 11
DHT dhtInside(DHTPININSIDE, DHTTYPE);
DHT dhtOutside(DHTPINOUTSIDE, DHTTYPE);


//mq2 settings (gas sensor)
/************************Hardware Related Macros************************************/
#define         Board                   ("Arduino UNO")
#define         Pin                     (A2)  //Analog input 3 of your arduino
/***********************Software Related Macros************************************/
#define         Type                    ("MQ-2") //MQ2
#define         Voltage_Resolution      (5)
#define         ADC_Bit_Resolution      (10) // For arduino UNO/MEGA/NANO
#define         RatioMQ2CleanAir        (9.83) //RS / R0 = 9.83 ppm 
MQUnifiedsensor MQ2(Board, Voltage_Resolution, ADC_Bit_Resolution, Pin, Type);


//declaring the function
void parseJson(String jsonString);

void setup() {
  //Initialize serial communication


/*MQ2 settup*/

  //Set math model to calculate the PPM concentration and the value of constants
  MQ2.setRegressionMethod(1); //_PPM =  a*ratio^b
  MQ2.setA(574.25); MQ2.setB(-2.222); // Configure the equation to to calculate LPG concentration
  /*
    Exponential regression:
    Gas    | a      | b
    H2     | 987.99 | -2.162
    LPG    | 574.25 | -2.222
    CO     | 36974  | -3.109
    Alcohol| 3616.1 | -2.675
    Propane| 658.71 | -2.168
  */

  /*****************************  MQ Init ********************************************/ 
  //Remarks: Configure the pin of arduino as input.
  /************************************************************************************/ 
  MQ2.init(); 
  /* 
    //If the RL value is different from 10K please assign your RL value with the following method:
    MQ2.setRL(10);
  */
  /*****************************  MQ CAlibration ********************************************/ 
  // Explanation: 
   // In this routine the sensor will measure the resistance of the sensor supposedly before being pre-heated
  // and on clean air (Calibration conditions), setting up R0 value.
  // We recomend executing this routine only on setup in laboratory conditions.
  // This routine does not need to be executed on each restart, you can load your R0 value from eeprom.
  // Acknowledgements: https://jayconsystems.com/blog/understanding-a-gas-sensor
  Serial.print("Calibrating please wait.");
  float calcR0 = 0;
  for(int i = 1; i<=10; i ++)
  {
    MQ2.update(); // Update data, the arduino will read the voltage from the analog pin
    calcR0 += MQ2.calibrate(RatioMQ2CleanAir);
    Serial.print(".");
  }
  MQ2.setR0(calcR0/10);
  Serial.println("  done!.");
  
  if(isinf(calcR0)) {Serial.println("Warning: Conection issue, R0 is infinite (Open circuit detected) please check your wiring and supply"); while(1);}
  if(calcR0 == 0){Serial.println("Warning: Conection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply"); while(1);}
  /*****************************  MQ CAlibration ********************************************/ 
  MQ2.serialDebug(true);

/*rest of setup*/
  Serial.begin(9600);
  espSerial.begin(9600);

  //begin dht sensors
  dhtInside.begin();
  dhtOutside.begin();

  //Configure pins
  pinMode(waterPumpPin, OUTPUT);
  pinMode(lightPin, OUTPUT);
  pinMode(fanPin, OUTPUT);
  pinMode(lockPin, OUTPUT);
}

void loop() {
  //Read from serial and update data. 
  if (espSerial.available() > 0) {
    //receive it via json
    String message = espSerial.readStringUntil('\n');
    parseJson(message); 
  }
  //Read sensor values
  MQ2.update(); // Update data, the arduino will read the voltage from the analog pin
  gasSensorValue = MQ2.readSensor();
  //using sonar for detecting water level
  waterLevelSensorValue = sonar.ping_cm();

  //read all sensor inputs before sending to esp 
  humInside = dhtInside.readHumidity();
  // Read temperature as Celsius (the default)
  tempInside = dhtInside.readTemperature();
  humOutside = dhtOutside.readHumidity();
  tempOutside = dhtOutside.readTemperature();


  // Check if any reads failed and exit early (to try again).
  if (isnan(humInside) || isnan(tempInside) || isnan(humOutside) || isnan(tempOutside)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }
  //control appliances automatically and manually
  //TODO: time the incoming and outgoing data
  //TODO: how will it know when its in automatic mode or manual mode.


  waterPumpState = digitalRead(waterPumpPin);
  lightState = digitalRead(lightPin);
  fanState = digitalRead(fanPin);
  lockState = digitalRead(lockPin);

  //Create JSON object with sensor values and appliance states
  DynamicJsonDocument doc(96);
  doc["gasSensorValue"] = gasSensorValue;
  doc["waterLevelSensorValue"] = waterLevelSensorValue;
  doc["waterPumpState"] = waterPumpState;
  doc["lightState"] = lightState;
  doc["fanState"] = fanState;
  doc["toggleAllFlag"] = toggleAllFlag;
  doc["lockState"] = lockState;
  doc["temperatureInside"] = tempInside;
  doc["temperatureOutside"] = tempOutside;
  doc["humidityInside"] = humInside;
  doc["humidityOutside"] = humOutside;

  //Serialize JSON object to a string and send to ESP8266 over software serial
  String jsonString;
  Serial.flush();
  serializeJson(doc, jsonString);
  // espSerial.println(jsonString);  
  Serial.print(jsonString);

  if(updated == true)
  {
    digitalWrite(fanPin, fanState);
    digitalWrite(lightPin, lightState);
    digitalWrite(lockPin, lockState);
    updated = false; 
  }
delay(1000);
}


void parseJson(String jsonString) {
  DynamicJsonDocument jsonDoc(200);
  DeserializationError error = deserializeJson(jsonDoc, jsonString);
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.c_str());
    return;
  }
  waterPumpState = jsonDoc["waterPumpState"];
  lightState = jsonDoc["lightState"];
  fanState = jsonDoc["fanState"];
  toggleAllFlag = jsonDoc["toggleAllFlag"];
  lockState = jsonDoc["lockState"];
  updated = true; 
}