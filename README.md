# Home Automation System
The project is hosted at [easyeda](https://oshwlab.com/krish.shoaib55/home-automation)
This is a simple home automation system that controls appliances based on sensor values. The system consists of an Arduino board and an ESP8266 module. The Arduino board reads sensor values and controls appliances based on those values. The ESP8266 module communicates with the Arduino board over software serial and receives sensor values and appliance states. It then sends the data over Wi-Fi to a server for remote monitoring and control.

## Components
+Arduino Uno board
+ESP8266 module
+Gas sensor
+Water level sensor
+Water pump
+2 lights
+Fan

## Arduino Part
The Arduino board reads the values from the gas and water level sensors and controls the water pump, lights, and fan based on the sensor values. It also sends the sensor values and appliance states over software serial to the ESP8266 module.

## ESP8266 Part
The ESP8266 module receives the sensor values and appliance states from the Arduino board over software serial. It then encodes the data in JSON format and sends it over Wi-Fi to a server for remote monitoring and control. The data is decoded and stored on the server, and the user can view the current sensor values and appliance states and control the appliances remotely.

To run the ESP8266 code, you need to include the following libraries:
```
ESP8266WiFi
ArduinoJson
SoftwareSerial
```
You also need to set up your Wi-Fi credentials and server details in the code.

## Setup
1. Connect the gas sensor to the A0 pin of the Arduino board.
2. Connect the water level sensor to the A1 pin of the Arduino board.
3. Connect the water pump to pin 8 of the Arduino board.
4. Connect light 1 to pin 9 of the Arduino board.
5. Connect light 2 to pin 10 of the Arduino board.
6. Connect the fan to pin 11 of the Arduino board.
7. Connect the RX pin of the ESP8266 module to pin 3 of the Arduino board.
8. Connect the TX pin of the ESP8266 module to pin 2 of the Arduino board.
9. Power up the Arduino board and ESP8266 module.
10. Upload the Arduino code to the Arduino board.
11. Upload the ESP8266 code to the ESP8266 module.
12. Monitor the sensor values and appliance states on the remote server.
13. Hardware files as well as android app is inside the parent folder as rar files. take them out and use accordingly. 


### Future Improvements
+ Add more sensors to monitor the environment, such as temperature and humidity sensors.
+ Add more appliances to control, such as a security system and a smart door lock.
+ Improve the user interface for remote monitoring and control.
+ Add machine learning algorithms to learn and adapt to user preferences and behavior.

## Some pictures
![3d](https://github.com/manhoosbilli1/home-automation/assets/36271208/67110e69-fc3b-4b97-bac9-086a1e449937)
![2d](https://github.com/manhoosbilli1/home-automation/assets/36271208/424312cc-0b29-4391-a5c0-b3c392068891)

