# arduino-river-monitor
This is a device that will record data and monitor the changes in depth and flow rate of a flowing body of water, and serve as a warning system when certain changes have been detected within a specified period of time.

## I. Parts List
#### 1. Arduino Uno Rev3
This will serve as the main controller for all the sensors and modules.
#### 2. NodeMCU V3 ESP8266 ESP-12E
This will be responsible for running the web interface to control the Arduino Uno.
#### 3. HC-SR04 Ultrasonic Sensor
This sensor will be used to measure the depth of the water. It will be mounted at a fixed height relative to the bottom of the body of water being measured.
#### 4. YF-S201 Water Flow Meter
This sensor will read the flow rate of water.
#### 5. SIM800L V2
This GSM Module will be responsible for sending long-range notifications to specified recipients that can be configured through the web interface.
#### 6. Micro SD Card Module
This module will be responsible for storing the data logs of measurements for later analysis.
#### 7. DS3231 RTC Module
This module will keep track of the current time to allow the tagging of data with a specified date/time

## II. Functions
#### A. Measurement
- [ ] Measure water depth
- [ ] Measure flow rate

#### B. Monitoring
- [ ] Store new data
- [ ] Read and analyze current data

#### C. Reports
- [ ] Send alert
- [ ] Send status report

#### D. Control
- [ ] Web Interface
- [ ] SMS Commands
