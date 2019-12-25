#include <SPI.h> // For SPI Communication with the SD Card Module
#include <SD.h> // Library for the SD Card Module
#include "DS3231.h"

// Flow Rate Sensor
byte sensorInterrupt = 0;  // 0 = digital pin 2
byte sensorPin       = 2;
// The hall-effect flow sensor outputs approximately 4.5 pulses per second per
// litre/minute of flow.
float calibrationFactor = 4.5;
volatile byte pulseCount = 0;  
float flowRate;
unsigned int flowMilliLitres;
unsigned long oldFlowRateMeasureTime;

// Ultrasonic Sensor
const int pingPin = 6;  // Trigger Pin of Ultrasonic Sensor
const int echoPin = 5;  // Echo Pin of Ultrasonic Sensor

// RTC Module
RTClib RTC;
uint32_t lastScan = 0;

// SD Card Module
const int sdPin = 4; // CS Pin of SD Card Module
boolean sdModuleInitialized;
File openFile;
String logFile = "DATA";
String cacheFile = "CACHE";
String configFile = "SETTINGS";
// Variables to be set by config file to be read from the SD Card
int scanInterval;       // [SETTING_ID 0] Time in between scans in seconds
long depthOffset;       // [SETTING_ID 1] CM distance of the ultrasonic sensor from the bottom of the body of water
int depthSamplingCount; // [SETTING_ID 2] The number of depth samples taken in one scan. One sample is 100 ms, so by default, 50 samples will take 5 seconds to measure

// GSM Module
String alertRecipients;
int alertRecipientsCount;

boolean serialDebug = true;

void setup() {
    Serial.begin(115200);
    Wire.begin();

    // Ultrasonic Sensor
    pinMode(pingPin, OUTPUT);
    pinMode(echoPin, INPUT);

    // Flow Rate Sensor
    pinMode(sensorPin, INPUT);
    digitalWrite(sensorPin, HIGH);
    pulseCount = 0;
    flowRate = 0.0;
    flowMilliLitres = 0;
    oldFlowRateMeasureTime = 0;
    // The Hall-effect sensor is connected to pin 2 which uses interrupt 0.
    // Configured to trigger on a FALLING state change (transition from HIGH
    // state to LOW state)
    attachInterrupt(sensorInterrupt, pulseCounter, FALLING);

    // Initializes the SD Card module and chekcs if it is successful
    if (!SD.begin(sdPin)) {
        sdModuleInitialized = false;
        Serial.println("SD not Initialized"); // TEMP Remove this to save memory
    }
    else {
        sdModuleInitialized = true;
        Serial.println("SD Initialized!!!"); // TEMP Remove this to save memory

        if (!applyConfigFile()) {
            Serial.println("Settings from config file could not be applied. Setting defaults...");
            scanInterval = 1;
            depthOffset = 15;
            depthSamplingCount = 10;
        }
    }

    
    
    // TEMP Remove this to save memory
    if (serialDebug) {
        Serial.print("Scan Interval : ");
        Serial.print(scanInterval);
        Serial.println(" seconds");

        Serial.print("Depth Offset : ");
        Serial.print(depthOffset);
        Serial.println(" cm");

        Serial.print("Depth Sampling Count : ");
        Serial.print(depthSamplingCount);
        Serial.println(" samples");
    }
}

void loop() {

}

/*
void loop() {
    if ((millis() - oldFlowRateMeasureTime) > 1000) {
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
        
        // Divide the flow rate in litres/minute by 60 to determine how many litres have
        // passed through the sensor in this 1 second interval, then multiply by 1000 to
        // convert to millilitres.
        flowMilliLitres = (flowRate / 60) * 1000;
        
        // Print the flow rate for this second in litres / minute
        Serial.print("Flow rate: ");
        Serial.print(int(flowRate));  // Print the integer part of the variable
        Serial.println("L/min");
        
        // Reset the pulse counter so we can start incrementing again
        pulseCount = 0;
        
        // Enable the interrupt again now that we've finished sending output
        attachInterrupt(sensorInterrupt, pulseCounter, FALLING);
    }

    DateTime now = RTC.now();
    if ((now.unixtime() - lastScan) >= scanInterval) {
        recordData();
    }
}
*/

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
    int uniqueValueInstances[depthSamplingCount]; // Keeps track of the instances a value has appeared in readValues
    // Fill uniqueValueInstances with zeroes
    for (int x = 0; x < depthSamplingCount; x++) {
        uniqueValueInstances[x] = 0;
    }
    int foundUniqueValues = 0; // Keeps track of the number of unique values stored
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

    int maxInstances = 0; // Keeps track of the highest score in uniqueValueInstances
    long returnValue; // Keeps track of the address of the value with the highest score from uniqueValues

    for (int x = 0; x < foundUniqueValues; x++) {
        if (uniqueValueInstances[x] > maxInstances) {
            maxInstances = uniqueValueInstances[x];
            returnValue = uniqueValues[x];
        }
    }

    return returnValue;
}

boolean writeToFile(String data, String file) {
    boolean writeSuccess = false;

    // TEMP serialDebug Get rid of this to save memory
    if (serialDebug) {
        Serial.print("Writing \"");
        Serial.print(data);
        Serial.print("\" to \"");
        Serial.print(file);
        Serial.println("\"");
    }

    if (sdModuleInitialized) {
        openFile = SD.open(file, FILE_WRITE);
        if (openFile) {
            openFile.println(data);
            openFile.close();
            writeSuccess = true;
        }
    }

    // TEMP serialDebug Get rid of this to save memory
    if (serialDebug) {
        if (writeSuccess) {
            Serial.println("Write Successful");
        }
        else {
            Serial.println("Write Unsuccessful");
        }
    }

    return writeSuccess;
}

boolean applyConfigFile() {
    boolean applied_scanInterval = false;
    boolean applied_depthOffset = false;
    boolean applied_depthSamplingCount = false;
    boolean applied_alertRecipientsCount = false;
    boolean applied_alertRecipients = false;

    // Opens the configuration file
    openFile = SD.open(configFile);
    // if the file is successfully opened
    if (openFile) {
        byte currentRead; // Stores the currently-read byte
        byte lastRead; // Stores the last read byte prior to currentRead
        boolean ignore = false; // true if the current line is a comment, becomes false after a line feed
        String currentBuild = ""; // The current collection of characters until an end character is found, then it gets cleared
        int currentID = 0; // The ID of the current setting being applied
        
        // Keeps track if in the middle of a String building process.
        int ongoingBuild = 0;
        // 0 - None
        // 1 - Setting ID
        // 2 - Setting value

        // while there is available data in the file
        while(openFile.available()) {
            currentRead = openFile.read(); // read the current byte from the file
            
            // if current line is not being ignored
            if (!ignore) {
                // if the current byte is a comment marker '#'
                if (currentRead == 35) {
                    ignore = true;
                }
                // if the current byte is not a comment marker '#'
                else {
                    // if no String build is ongoing
                    if (ongoingBuild == 0) {
                        // if the last read byte is a line feed
                        if (lastRead == 10) {
                            ongoingBuild = 1; // set ongoing build status to '1' for setting ID
                            currentBuild += (char)currentRead; // add the current byte to the String build
                        }
                    }
                    // if a setting ID build is ongoing
                    else if (ongoingBuild == 1) {
                        // if the current byte is an assignment operator '='
                        if (currentRead == 61) {
                            if (currentBuild.equals("scanInterval")) {
                                delay(100);
                                currentID = 1;
                                ongoingBuild++;
                            }
                            else if (currentBuild.equals("depthOffset")) {
                                delay(100);
                                currentID = 2;
                                ongoingBuild++;
                            }
                            else if (currentBuild.equals("depthSamplingCount")) {
                                delay(100);
                                currentID = 3;
                                ongoingBuild++;
                            }
                            else if (currentBuild.equals("alertRecipientsCount")) {
                                delay(100);
                                currentID = 4;
                                ongoingBuild++;
                            }
                            else if (currentBuild.equals("alertRecipients")) {
                                delay(100);
                                currentID = 5;
                                ongoingBuild++;
                            }
                            // if setting ID does not match any existing setting IDs, ignore the rest of the line
                            else {
                                ignore = true;
                                break;
                            }
                            currentBuild = "";
                        }
                        // if the current byte is not an assignment operator '='
                        else {
                            currentBuild += (char)currentRead;
                        }
                    }
                    // if the setting data build is ongoing
                    else if (ongoingBuild == 2) {
                        // if the current byte is a newline
                        if (currentRead == 10) {
                            // if the current setting is the alert recipients
                            if (currentID == 5) {
                                alertRecipients = currentBuild;
                                applied_alertRecipients = true;
                            }
                            // if the current setting is not the alert recipients,
                            // test if the currently built value is a valid number
                            else if (validInt(currentBuild)) {
                                int parsedValue = currentBuild.toInt(); // stores the converted setting value

                                switch (currentID) {
                                case 1:
                                    scanInterval = parsedValue;
                                    applied_scanInterval = true;
                                    break;

                                case 2:
                                    depthOffset = parsedValue;
                                    applied_depthOffset= true;
                                    break;

                                case 3:
                                    depthSamplingCount = parsedValue;
                                    applied_depthSamplingCount = true;
                                    break;

                                case 4:
                                    alertRecipientsCount = parsedValue;
                                    applied_alertRecipientsCount = true;
                                    break;
                                
                                default:
                                    break;
                                }

                                ongoingBuild = 0;
                                currentBuild = "";
                                currentID = 0;
                            }
                            // if the currently built value is not a valid number
                            else {
                                Serial.print(currentID);
                                Serial.println(" invalid value !!!!!!!!!!!!!!!!!!!!!");
                                Serial.write(135); // Config value error
                            }
                        }
                        else {
                            currentBuild += (char)currentRead;
                        }
                    }
                }
            }
            // if current line is being ignored
            else {
                if (currentRead == 10) {
                    ignore = false;
                }
            }

            lastRead = currentRead;
        }
    }
    // if the file could not be opened
    else {
        Serial.write(134); // Config file error
        // TODO handling for disabled state when a config file is not available
    }

    if (applied_scanInterval && applied_depthOffset && applied_depthSamplingCount && applied_alertRecipientsCount&& applied_alertRecipients) {
        return true;
    }

    return false;
}

uint32_t getLastScanTimeFromCache() {
    uint32_t returnValue = 0;

    if (sdModuleInitialized) {
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
        else if (!openFile && serialDebug) {
            // TEMP serialDebug Get rid of this to save memory
            Serial.println("Could not open cache file");
        }
    }

    return returnValue;
}

int recordData() {
    DateTime now = RTC.now();
    uint32_t scanStart = now.unixtime();
    long depth = checkDepth();
    now = RTC.now();
    lastScan = now.unixtime();

    // DATA.LOG Entries will first be collected and formatted into String logEntry.
    // The format will be the following with '~' as a column separator:
    // Time Start / Time End / Flow Rate / Flow Amount / Depth / Depth Offset
    String logEntry = "";
    boolean validData = false;

    while (!validData)
    {
        logEntry += scanStart;
        logEntry += char(47);
        logEntry += lastScan;
        logEntry += char(47);
        logEntry += flowRate;
        logEntry += char(47);
        logEntry += flowMilliLitres;
        logEntry += char(47);
        logEntry += depth;
        logEntry += char(47);
        logEntry += depthOffset;

        if (verifyDataIntegrity(logEntry)) {
            validData = true;
        }
        else {
            Serial.println("Data integrity compromised. Retrying...");
            logEntry = "";
        }
    }
    writeToFile(logEntry, logFile);
}

// Interrupt Service Routine
void pulseCounter() {
  // Increment the pulse counter
  pulseCount++;
}

// Verifies log entries to ensure that only legal characters are used before writing to log file
boolean verifyDataIntegrity(String input) {
    for (int x = 0; x < input.length(); x++) {
        byte readByte = input.charAt(x);
        if (readByte < 45 || readByte > 57) {
            return false;
        }
    }

    return true;
}

// TODO Document and optimize this if possible
// TODO create a log size query
// Starts a data upload stream
void uploadData() {
    openFile = SD.open(logFile);
    if (openFile) {
        byte currentByte;
        byte lastByte = 10;
        while(openFile.available()) {
            currentByte = openFile.read();

            if (lastByte == 10) {
                Serial.write(2);
            }

            lastByte = currentByte;
        }
        openFile.close();
    }
}

// Queries and sends the size of the log file over serial
void getLogSize() {
    int lines = 0;
    openFile = SD.open(logFile);
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

// Checks if the current String is a valid candidate for conversion into int
boolean validInt(String input) {
    for (int x = 0; x < input.length(); x++) {
        if (!isDigit(input.charAt(x))) {
            return false;
        }
    }
    return true;
}