# home-automation
Smart Home Monitoring System
This is a smart home monitoring system consisting of an Arduino Uno board and an ESP8266 Wi-Fi module. The system uses various sensors to monitor the home environment, including a gas sensor and a water level sensor, and controls appliances such as lights, fan, and water pump based on sensor readings.

The system communicates wirelessly with a remote device such as a smartphone or computer via Wi-Fi, allowing users to monitor the home environment and control appliances from a distance.

Components
The following components are used in the system:

Arduino Uno board
ESP8266 Wi-Fi module
Gas sensor
Water level sensor
Water pump
Lights (2)
Fan
Arduino Part
The Arduino Uno board reads sensor values and controls appliances based on the readings. The gas sensor and water level sensor are connected to analog pins of the Arduino board. The water pump, lights, and fan are controlled via digital pins of the board.

The board communicates with the ESP8266 Wi-Fi module via software serial. It also sends sensor readings and appliance states to the ESP8266 module over serial in JSON format.

ESP8266 Part
The ESP8266 Wi-Fi module receives sensor readings and appliance states from the Arduino board over serial in JSON format. It decodes the JSON data and stores each field in a separate variable. It also prints the sensor readings and appliance states to the serial monitor.

The module can be connected to a Wi-Fi network and act as a web server, allowing remote monitoring and control of the home environment via a web interface.

Usage
To use the system, follow these steps:

Connect the components as shown in the circuit diagram.
Upload the Arduino code to the Arduino Uno board.
Upload the ESP8266 code to the ESP8266 Wi-Fi module.
Connect the ESP8266 module to a Wi-Fi network.
Open the serial monitor to view the sensor readings and appliance states.
Use a remote device such as a smartphone or computer to connect to the ESP8266 module and control the appliances via the web interface.
Note: Make sure to install the required libraries (SoftwareSerial and ArduinoJson) before uploading the code to the respective devices.
