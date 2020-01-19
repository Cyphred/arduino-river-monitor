#include <SPI.h> // For SPI Communication with the SD Card Module
#include <SD.h> // Library for the SD Card Module
#include "DS3231.h"
#include <DS3231.h>

// Flow Rate Sensor
byte sensorInterrupt = 0;  // 0 = digital pin 2
byte sensorPin       = 2;
// The hall-effect flow sensor outputs approximately 4.5 pulses per second per litre/minute of flow.
float calibrationFactor = 4.5;
volatile byte pulseCount = 0;  
float flowRate;
unsigned long oldFlowRateMeasureTime;

// Ultrasonic Sensor
const int pingPin = 6;  // Trigger Pin of Ultrasonic Sensor
const int echoPin = 5;  // Echo Pin of Ultrasonic Sensor
long lastDepth;

// RTC Module
RTClib RTC;
DS3231 Clock;
uint32_t lastScan = 0;

// Indicator LEDs
byte activityLEDState = 0;
int ledActivity = A2;
int ledError = A1;
int ledConnected = A0;

// SD Card Module
const int sdPin = 4; // CS Pin of SD Card Module
File openFile;
char dataLogFile[5] = {68,65,84,65}; // DATA
char cacheFile[5] = {67,65,67,72}; // CACH
char configFile[5] = {67,79,78,70}; // CONF
char activityLogFile[4] = {65,67,84}; // ACT
char smsLogFile[4] = {83,77,83}; // SMS

// Variables to be set by config file to be read from the SD Card
byte scanInterval;         // [SETTING_ID 0] Time in between scans in seconds
long depthOffset;         // [SETTING_ID 1] CM distance of the ultrasonic sensor from the bottom of the body of water
byte depthSamplingCount;   // [SETTING_ID 2] The number of depth samples taken in one scan. One sample is 100 ms, so by default, 50 samples will take 5 seconds to measure
unsigned long alertRecipient_a;   // [SETTING_ID 3] The number of recipients for alerts and status updates
unsigned long alertRecipient_b;   // [SETTING_ID 4] The number of recipients for alerts and status updates
byte scanIntervalOverride; // [SETTING_ID 5] scan interval ovverride during alert mode // TODO Add this to config file
byte revertDuration; // [SETTING_ID 6] duration before removing alert mode

boolean serialDebug = false;
boolean sdCardReady = false;
boolean configFileApplied = false;
boolean connectedToApp = false;

unsigned long timeoutStart;
byte operationState = 0;

// TODO Sort these out
boolean alertMode = false;
byte depthStatus = 0;
byte flowRateStatus = 0;
byte lastErrorCode = 0;
long revertTime;
String status0 = "OK";
String status1 = "ABOVE NORMAL";
String status2 = "CRITICAL";

void setup() {
    // initialize Serial communicatioin at 2M baud rate to allow for faster data transfer rates,
    // mainly for large data sets consisting of tens of thousands of lines of data
    Serial.begin(2000000);
    Wire.begin();

    // Initialization for status LEDs
    pinMode(ledActivity,OUTPUT);
    pinMode(ledError,OUTPUT);
    pinMode(ledConnected,OUTPUT);

    // Turns on all status LEDs to signify initialization
    digitalWrite(ledActivity,HIGH);
    digitalWrite(ledError,HIGH);
    digitalWrite(ledConnected,HIGH);

    Serial.write(128); // Tells the app that the device is ready to connect
    // wait 3 seconds for signal from app. If the signal is not received, proceed with normal operations
    // TODO test if you can reduce this waiting time to make startup feel snappier
    timeoutStart = millis();
    while ((millis() - timeoutStart) < 3000) {
        if (Serial.available()) {
            if (Serial.read() == 129) {
                connectedToApp = true;
                Serial.write(130); // Tells the app that the connection is established
                digitalWrite(ledConnected,HIGH);
                break;
            }
        }
    }

    // Ultrasonic Sensor
    pinMode(pingPin, OUTPUT);
    pinMode(echoPin, INPUT);

    // Flow Rate Sensor
    pinMode(sensorPin, INPUT);
    digitalWrite(sensorPin, HIGH);
    pulseCount = 0;
    flowRate = 0.0;
    oldFlowRateMeasureTime = 0;
    // The Hall-effect sensor is connected to pin 2 which uses interrupt 0.
    // Configured to trigger on a FALLING state change (transition from HIGH
    // state to LOW state)
    attachInterrupt(sensorInterrupt, pulseCounter, FALLING);

    // Initializes and checks if the SD is ready for use
    if (!SD.begin(sdPin)) {
        sdCardReady = false;
        debugln("SD not ready");
    }
    else {
        sdCardReady = true;
        debugln("SD Ready");
        if (applyConfigFile()) {
            configFileApplied = true;
            debugln("Config Applied");
        }
        else {
            debugln("Config not Applied");
        }
    }

    byte initializationSubCode = 0;
    if (connectedToApp) {
        if (sdCardReady && configFileApplied) {
            digitalWrite(ledError,LOW);
            initializationSubCode = 1; // Connected and no problems
        }
        else {
            initializationSubCode = 3;  // Connected with problems
        }
    }
    // if not connected to the app
    else {
        digitalWrite(ledConnected,LOW);

        if (sdCardReady && configFileApplied) {
            digitalWrite(ledError,LOW);
            initializationSubCode = 0; // Disconnected and no problems
        }
        else {
            initializationSubCode = 2;  // Disconnected with problems
        }
    }
    logActivity(0,initializationSubCode); // Logs the current startup
    digitalWrite(ledActivity,LOW);
}

boolean nothingHappens = false; //TEMP
unsigned long lasttime = millis(); //TEMP

void loop() {
    // TEMP - This is for testing some stuff at intervals
    if ((millis() - lasttime) > 10000) {
        lasttime = millis();
    }
    // TEMP =============================================
    // if the device is connected to the app
    if (connectedToApp) {
        // if a byte arrives, read it
        if (Serial.available()) {
            operationState = Serial.read(); // set operation state according to the byte that was received
        }

        switch (operationState) {
            case 131:
                sendDepthWithOffset();
                operationState = 0;
                break;

            case 132:
                sendDepth();
                operationState = 0;
                break;

            case 133:
                sendFlowRate();
                operationState = 0;
                break;

            case 134:
                getLogSize();
                operationState = 0;
                break;

            case 135:
                uploadData(dataLogFile);
                operationState = 0;
                break;

            case 136:
                uploadData(activityLogFile);
                operationState = 0;
                break;

            case 137:
                uploadData(configFile);
                operationState = 0;
                break;

            case 138:
                uploadData(smsLogFile);
                operationState = 0;
                break;

            case 139:
                getTime();
                operationState = 0;
                break;

            case 140:
                setTime();
                operationState = 0;
                break;

            case 141:
                updateConfig();
                operationState = 0;
                break;

            case 142:
                Serial.write(2);
                if (clearFileContents(dataLogFile))  {
                    Serial.print(1);
                }
                else {
                    Serial.print(0);
                }
                Serial.write(3);
                
                operationState = 0;
                break;

            case 143:
                Serial.write(2);
                if (clearFileContents(activityLogFile))  {
                    Serial.print(1);
                }
                else {
                    Serial.print(0);
                }
                Serial.write(3);
                
                operationState = 0;
                break;

            case 144:
                Serial.write(2);
                if (clearFileContents(smsLogFile))  {
                    Serial.print(1);
                }
                else {
                    Serial.print(0);
                }
                Serial.write(3);
                
                operationState = 0;
                break;
        }
    }
    // Only perform routine operations when:
    // - the SD card module is properly initialized
    // - the device is not connected to the app
    // - the configuration file has been properly implemented
    // Otherwise, do nothing
    // TODO or maybe, make it send alerts to the alert recipient about the malfunction
    else if (!connectedToApp && sdCardReady && configFileApplied) {
        
        // Measures the flow rate every second
        if ((millis() - oldFlowRateMeasureTime) > 1000) {
            delay(100);

            // Disable the interrupt while calculating flow rate and sending the value to the host
            detachInterrupt(sensorInterrupt);
                
            // Because this loop may not complete in exactly 1 second intervals we calculate
            // the number of milliseconds that have passed since the last execution and use
            // that to scale the output. We also apply the calibrationFactor to scale the output
            // based on the number of pulses per second per units of measure (litres/minute in
            // this case) coming from the sensor.
            flowRate = ((1000.0 / (millis() - oldFlowRateMeasureTime)) * pulseCount) / calibrationFactor;
            
            // Note the time this processing pass was executed. Note that because we've
            // disabled interrupts the millis() function won't actually be incrementing right
            // at this point, but it will still return the value it was set to just before
            // interrupts went away.
            oldFlowRateMeasureTime = millis();
            
            // Reset the pulse counter so we can start incrementing again
            pulseCount = 0;
            
            // Enable the interrupt again now that we've finished sending output
            attachInterrupt(sensorInterrupt, pulseCounter, FALLING);
        }

        DateTime now = RTC.now();
        if ((now.unixtime() - lastScan) >= scanInterval) {
            if (!recordData()) {
                suspendOperations();
            }
            lastScan = now.unixtime();
        }
    }
    else {
        // TEMP nothing happens
        if (!nothingHappens) {
            nothingHappens = true;
            debugln("\n\nNothing happens now...");
        }
    }
}

// TODO Make a method that will record the time of the last reading to a cache file of some sorts, so that in the event of a power outage, the device will remember when the last reading happened and start a scan in case the scan interval time has elapsed
// TODO Implement the GSM Module

long microsecondsToCentimeters(long microseconds) {
   return microseconds / 29 / 2;
}

// Uses the value of depthSamplingCount as the number of times to measure water depth and returns the most common value.
// Each sample takes 100 ms
long checkDepth() {
    long readValues[depthSamplingCount];

    // loop 50 times with a 100ms delay between each read
    for (int x = 0; x < depthSamplingCount; x++) {
        long duration, cm;
    
        digitalWrite(pingPin, LOW);
        delayMicroseconds(2);
        digitalWrite(pingPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(pingPin, LOW);
        duration = pulseIn(echoPin, HIGH);
        cm = microsecondsToCentimeters(duration);

        readValues[x] = cm;
        delay(100);
    }

    long uniqueValues[depthSamplingCount]; // Stores all the unique values read
    byte uniqueValueInstances[depthSamplingCount]; // Keeps track of the instances a value has appeared in readValues
    // Fill uniqueValueInstances with zeroes
    for (int x = 0; x < depthSamplingCount; x++) {
        uniqueValueInstances[x] = 0;
    }
    byte foundUniqueValues = 0; // Keeps track of the number of unique values stored
    // for each value contained in readValues
    for (long value: readValues) {
        // if no unique values are stored yet, store the current value from readValues
        if (foundUniqueValues == 0) {
            uniqueValues[foundUniqueValues] = value;
            foundUniqueValues++;
        }
        else {
            boolean duplicateFound = false; // becomes true when a duplicate of a value is found
            // tests the current value from readValues against each stored unique value
            for (int x = 0; x < foundUniqueValues; x++) {
                // if the current value from readValues matches a value from uniqueValues,
                // increment that value's score in uniqueValueInstances,
                // then stop checking uniqueValues and set duplicateFound to true
                if (value == uniqueValues[x]) {
                    uniqueValueInstances[x]++;
                    duplicateFound = true;
                    break;
                }
            }

            // if no duplicate was found, add the current value from readValues to uniqueValues,
            // then increment foundUniqueValues
            if (!duplicateFound) {
                uniqueValues[foundUniqueValues] = value;
                foundUniqueValues++;
            }
            // if a duplicate was found, do nothing
        }
    }

    byte maxInstances = 0; // Keeps track of the highest score in uniqueValueInstances
    long returnValue; // Keeps track of the address of the value with the highest score from uniqueValues

    for (int x = 0; x < foundUniqueValues; x++) {
        if (uniqueValueInstances[x] > maxInstances) {
            maxInstances = uniqueValueInstances[x];
            returnValue = uniqueValues[x];
        }
    }

    lastDepth = returnValue;
    return returnValue;
}

// Writes a string to a file
boolean writeToFile(String data, String file) {
    debug("Writing \"");
    debug(data);
    debug("\" to \"");
    debug(file);
    debug("\"... ");

    // only run if the SD card is properly initialized
    if (sdCardReady) {
        openFile = SD.open(file, FILE_WRITE);
        // if the file is successfully opened
        if (openFile) {
            openFile.println(data);
            openFile.close();
            debugln("Success!");
            return true;
        }
        // if the file can't be opened
        else {
            debugln("Failed! - Can't open file!");
            return false;
        }
    }
    debugln("Failed! - SD not ready");
    return false;
}

// Applies the settings specified in the configuration file
boolean applyConfigFile() {
    /*
    * The config file is now formatted as a single line with each value separated with a slash (character decimal 47)
    * and are laid out in the following order:
    * Scan Interval / Depth Offset / Depth Sampling Count / Alert phone numbers
    */

    boolean applied_scanInterval = false;
    boolean applied_depthOffset = false;
    boolean applied_depthSamplingCount = false;
    boolean applied_alertRecipient_a = false;
    boolean applied_alertRecipient_b = false;
    boolean applied_scanIntervalOverride = false;
    boolean applied_revertDuration = false;

    // Opens the configuration file
    openFile = SD.open(configFile);

    // If the config file is successfully opened
    if (openFile) {
        byte currentRead; // Stores the currently-read currentByte
        byte currentID = 0; // The ID of the current setting being applied
        unsigned long currentValue = 0; // Stores the current value being read

        // While the config file has available data for reading
        while (openFile.available()) {
            currentRead = openFile.read();
            // If the current read byte is a separator ('/' or decimal 47)
            if (currentRead == 47) {
                switch (currentID) {
                    case 0:
                        scanInterval = currentValue;
                        applied_scanInterval = true;
                        break;
                    case 1:
                        depthOffset = currentValue;
                        applied_depthOffset = true;
                        break;
                    case 2:
                        depthSamplingCount = currentValue;
                        applied_depthSamplingCount = true;
                        break;
                    case 3:
                        alertRecipient_a = currentValue;
                        applied_alertRecipient_a = true;
                        break;
                    case 4:
                        alertRecipient_b = currentValue;
                        applied_alertRecipient_b = true;
                        break;
                    case 5:
                        scanIntervalOverride = currentValue;
                        applied_scanIntervalOverride = true;
                        break;
                    case 6:
                        revertDuration = currentValue;
                        applied_revertDuration = true;
                        break;
                }

                currentValue = 0; // Resets the currently built value
                currentID++; // Increments the currentID to apply the next setting
            }
            // If the current read byte is a digit
            else if (currentRead > 47 && currentRead < 58) {
                currentValue = (currentValue * 10) + ((char)currentRead - '0');
            }
        }

        // if all settings have been properly applied
        if (applied_scanInterval && applied_depthOffset && applied_depthSamplingCount && applied_alertRecipient_a && applied_alertRecipient_b && applied_scanIntervalOverride && applied_revertDuration) {
            return true;
        }
        else {
            return false;
        }
    }

    // If the config file can't be opened, return false
    return false;
}

// TODO Work on this
uint32_t getLastScanTimeFromCache() {
    uint32_t returnValue = 0;

    if (sdCardReady) {
        openFile = SD.open(cacheFile); // Opens the cache file
        if (openFile) {
            String tempRead = "";
            while (openFile.available()) {
                byte readByte = openFile.read();
                if (readByte == 47 || readByte == 10) {
                    returnValue = tempRead.toInt();
                }
                else {
                    tempRead = (char)readByte;
                }
            }
            openFile.close();
        }
    }

    return returnValue;
}

// Read sensors and record data to the log file
boolean recordData() {
    blinkActivityLED();

    // Get the time of start for the reading
    DateTime now = RTC.now();
    uint32_t scanStart = now.unixtime();

    long depth = checkDepth(); // Get the current depth

    // Keep track of when the depth scan finished
    now = RTC.now();
    lastScan = now.unixtime();
    
    // DATA.LOG Entries will first be collected and formatted into String logEntry.
    // The format will be the following with '/' as a column separator:
    // Time Start / Time End / Flow Rate / Depth
    String logEntry;
    boolean validData = false;

    logEntry += scanStart;
    logEntry += char(47);
    logEntry += lastScan;
    logEntry += char(47);
    logEntry += scanInterval;
    logEntry += char(47);
    logEntry += flowRate;
    logEntry += char(47);
    logEntry += depth;
    logEntry += char(47);
    logEntry += depthOffset;

    // try to write 5 times. If it fails all 5, abort the write
    for (int x = 0; x < 5; x++) {
        if (writeToFile(logEntry, dataLogFile)) {
            // TODO Add write success
            return true;
        }
        else {
            // TODO Add write failure
        }
    }

    blinkActivityLED();

    return false;
}

// Interrupt Service Routine
void pulseCounter() {
  // Increment the pulse counter
  pulseCount++;
}

// Records activity to activity log file
void logActivity(byte code, byte subcode) {
    DateTime now = RTC.now();
    String activityLogEntry = (String)now.unixtime();
    activityLogEntry += (char)47;
    activityLogEntry += code;
    activityLogEntry += (char)58;
    activityLogEntry += subcode;
    writeToFile(activityLogEntry,activityLogFile);
}

// TEMP Remove these when you're done
void debugln(String message) {
    if (serialDebug) {
        Serial.println(message);
    }
}

// TEMP Remove these when you're done
void debug(String message) {
    if (serialDebug) {
        Serial.print(message);
    }
}

// Toggles the activity LED when called
void blinkActivityLED() {
    if (activityLEDState == 0) {
        activityLEDState = 1;
        digitalWrite(ledActivity,HIGH);
    }
    else {
        activityLEDState = 0;
        digitalWrite(ledActivity,LOW);
    }
}

// Suspends all operations
void suspendOperations() {
    digitalWrite(ledError,HIGH);
    digitalWrite(ledActivity,LOW);
    logActivity(1,0);
    debugln("Operations Suspended");
    sdCardReady = false;
}

// Sets the time for the RTC Module
boolean setTime() {
    timeoutStart = millis();
    boolean timedOut = true;
    char inputString[13];
    byte year,month,date,dayOfWeek,hour,minute,second;
    byte inputCount = 0;

    // While timeout is not maxed out, and the input count is not at 20 yet
    while ((millis() - timeoutStart) < 3000 && inputCount != 13) {
        if (Serial.available()) {
            byte receivedByte = Serial.read();
            // Check if received byte is a valid digit
            if (receivedByte > 47 && receivedByte < 58) {
                inputString[inputCount] = receivedByte;
                inputCount++;
                // if the input length is satisfied
                if (inputCount == 20) {
                    timedOut = false;
                    break;
                }
            }
            // if the received byte is not a valid digit, cancel the operation
            else {
                return false;
            }
        }
    }
    
    byte converted;
    boolean invalidNumberFound = false;
    for (int x = 1; x < 12; x += 2) {
        converted = (((byte)inputString[x-1] - 48) * 10 + ((byte)inputString[x] - 48));
        switch (x) {
            case 1:
                year = converted;
                break;
            case 3:
                if (converted > 0 && converted < 13) {
                    month = converted;
                }
                else {
                    invalidNumberFound = true;
                }
                break;
            case 5:
                if (converted > 0) {
                    // if the month is february
                    if (month == 2) {
                        // if the date is 28 below, or the date is 29 but it is a leap year
                        if (converted < 29 || (converted == 29 && (year % 4) == 0)) {
                            date = converted;
                        }
                        else {
                            invalidNumberFound = true;
                        }
                    }
                    // if the month has 31 days
                    else if ((month == 1 || month == 3 || month == 5 || month == 7 || month == 8 || month == 10 || month == 12) && converted < 32) {
                        date = converted;
                    }
                    // if the month has 30 days
                    else if ((month == 4 || month == 6 || month == 9 || month == 11) && converted < 31) {
                        date = converted;
                    }
                    else {
                        invalidNumberFound = true;
                    }
                }
                else {
                    invalidNumberFound = true;
                }
                break;
            case 7:
                if (converted >= 0 && converted < 25) {
                    hour = converted;
                }
                else {
                    invalidNumberFound = true;
                }
                break;
            case 9:
                if (converted >= 0 && converted < 60) {
                    minute = converted;
                }
                else {
                    invalidNumberFound = true;
                }
                break;
            case 11:
                if (converted >= 0 && converted < 60) {
                    second = converted;
                }
                else {
                    invalidNumberFound = true;
                }
                break;
        }

        if (invalidNumberFound) {
            return false;
        }
    }
    
    converted = (byte)inputString[12] - 48;
    if (converted > 0 && converted < 8) {
        dayOfWeek = converted;
    }
    else {
        return false;
    }

    Clock.setYear(year);
    Clock.setMonth(month);
    Clock.setDate(date);
    Clock.setDoW(dayOfWeek);
    Clock.setHour(hour);
    Clock.setMinute(minute);
    Clock.setSecond(second);

    return true;
}

// Reads message templates and fills in fields for sending via the GSM Module
void parseMessage(byte type) {
    // 1 - MSGA
    // 2 - MSGB
    // 3 - MSGC
    byte messageTemplate[5] = {77,83,71,(type + 64)};
    boolean readField = false;

    DateTime now = RTC.now();

    openFile = SD.open(messageTemplate);
    if (openFile) {
        while (openFile.available()) {
            byte readByte = openFile.read();
            if (!readField) {
                // if readByte is a dollar sign
                if (readByte == 36) {
                    readField = true;
                }
                else {
                    // TODO Modify this to send the bytes to software serial
                    Serial.write(readByte);
                }
            }
            else {
                readByte -= 64; // deduct 64 from the read byte so that the alphabet-based counting starts with 1 for letter 'A'
                switch (readByte) {
                    case 1: // A - Date
                        Serial.print(now.year(), DEC);
                        Serial.write(47); // backward slash
                        Serial.print(now.month(), DEC);
                        Serial.write(47); // backward slash
                        Serial.print(now.day(), DEC);
                        break;
                    case 2: // B - Time
                        Serial.print(now.hour(), DEC);
                        Serial.write(58); // colon
                        Serial.print(now.minute(), DEC);
                        Serial.write(58); // colon
                        Serial.print(now.second(), DEC);
                        break;
                    case 3: // C - Depth Status
                        if (depthStatus == 0) {
                            Serial.print(status0);
                        }
                        else if (depthStatus == 1) {
                            Serial.print(status1);
                        }
                        else if (depthStatus == 2) {
                            Serial.print(status2);
                        }
                    break;
                    case 4: // D - Depth
                        Serial.print(lastDepth);
                        break;
                    case 5: // E - Depth delta sign
                        // TODO Print depth delta sign
                        break;
                    case 6: // F - Depth delta percentage
                        // TODO Print depth delta
                        break;
                    case 7: // G - Flow Rate Status
                        if (flowRateStatus == 0) {
                            Serial.print(status0);
                        }
                        else if (flowRateStatus == 1) {
                            Serial.print(status1);
                        }
                        else if (flowRateStatus == 2) {
                            Serial.print(status2);
                        }
                        break;
                    case 8: // H - Flow Rate
                        Serial.print(flowRate);
                        break;
                    case 9: // I - Flow Rate delta sign
                        // TODO Print flow rate delta sign
                        break;
                    case 10: // J - Flow Rate percentage
                        // TODO Print flow rate delta
                        break;
                    case 11: // K - Error code
                        Serial.print(lastErrorCode);
                        break;
                    case 12: // L - Scan interval override
                        Serial.print(scanIntervalOverride);
                        break;
                    case 13: // M - Revert duration
                        Serial.print(revertTime);
                        break;
                    case 14: // N - Revert time estimate
                        Serial.print(revertDuration);
                        break;
                }
                readField = false;
            }
        }
        
    }
}

// Commands for sending data to the app

void sendDepthWithOffset() {
    Serial.write(2);
    Serial.print((checkDepth() - depthOffset));
    Serial.write(3);
}

void sendDepth() {
    Serial.write(2);
    Serial.print(checkDepth());
    Serial.write(3);
}

void sendFlowRate() {
    Serial.write(2);
    Serial.print(flowRate);
    Serial.write(3);
}

// Queries and sends the number of entries of the log file over serial
void getLogSize() {
    int lines = 0;
    openFile = SD.open(dataLogFile);
    if (openFile) {
        while(openFile.available()) {
            byte currentByte = openFile.read();
            if (currentByte == 10) {
                lines++;
            }
        }
        openFile.close();
    }
    
    Serial.write(2);
    Serial.print(lines);
    Serial.write(3);
}

// TODO Document and optimize this if possible
// TODO create a log size query
// Starts a data upload stream
void uploadData(String file) {
    openFile = SD.open(file);
    if (openFile) {
        Serial.write(2); // upload start marker
        while(openFile.available()) {
            Serial.write(openFile.read());
        }
        Serial.write(3); // upload end marker
        openFile.close();
    }
}

// Prints the current unix time to Serial
void getTime() {
    DateTime now = RTC.now();
    unsigned long time = now.unixtime();
    Serial.write(2);
    Serial.print(time);
    Serial.write(3);
}

// End of commands for sending data to the app

// Overwrites the config file
boolean updateConfig() {
    timeoutStart = millis();
    boolean byteStreamActive = false;
    String tempBuild = "";
    while ((millis() - timeoutStart) < 1000) {
        if (Serial.available()) {
            byte readByte = Serial.read();
            if (!byteStreamActive && readByte == 2) {
                byteStreamActive = true;
            }
            else if (byteStreamActive && readByte == 3) {
                byteStreamActive = false;
                overwriteFile(tempBuild,configFile);
                return true;
            }
            else if (byteStreamActive) {
                tempBuild += (char)readByte;
            }
        }
    }
    return false;
}

boolean overwriteFile(String data, String file) {
    if (sdCardReady) {
        SD.remove(file);
        openFile = SD.open(file,FILE_WRITE);
        if (openFile) {
            openFile.print(data);
            openFile.close();
            return true;
        }
    }
    return false;
}

boolean clearFileContents(String file) {
    if (sdCardReady) {
        SD.remove(file);
        openFile = SD.open(file,FILE_WRITE);
        if (openFile) {
            openFile.print("");
            openFile.close();
            return true;
        }
    }
    return false;
}