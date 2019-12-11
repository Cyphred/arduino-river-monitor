#include <SPI.h> // For SPI Communication with the SD Card Module
#include <SD.h> // Library for the SD Card Module
#include "DS3231.h"

const int sdPin = 4; // CS Pin of SD Card Module
const int pingPin = 6;  // Trigger Pin of Ultrasonic Sensor
const int echoPin = 5;  // Echo Pin of Ultrasonic Sensor

RTClib RTC;
uint32_t lastScan;

// Variables to be set by config file
int scanInterval;       // [SETTING_ID 0] Time in between scans in seconds
long depthOffset;       // [SETTING_ID 1] CM distance of the ultrasonic sensor from the bottom of the body of water
int depthSamplingCount; // [SETTING_ID 2] The number of depth samples taken in one scan. One sample is 100 ms, so by default, 50 samples will take 5 seconds to measure

boolean sdModuleInitialized;
File openFile;
String logFile = "DATA.LOG";
String cacheFile = "CACHE.CAC";
String configFile = "SETTINGS.CFG";

boolean serialDebug = true;

void setup() {
    Serial.begin(9600);
    pinMode(pingPin, OUTPUT);
    pinMode(echoPin, INPUT);
    Wire.begin();

    // Initializes the SD Card module and chekcs if it is successful
    if (!SD.begin(sdPin)) {
        sdModuleInitialized = false;
        Serial.println("SD not Initialized"); // TEMP Remove this to save memory
    }
    else {
        sdModuleInitialized = true;
        Serial.println("SD Initialized"); // TEMP Remove this to save memory
    }

    applyConfigFile();
    
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
    DateTime now = RTC.now();
    if ((now.unixtime() - lastScan) >= scanInterval) {
        scan();
    }
}

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

    return (depthOffset - returnValue);
}

double checkFlowRate() {
    return 0;
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

// Reads settings from the config file and applies them
boolean applyConfigFile() {
    int appliedSettings = 0; // Keeps track of the number of settings applied. Each setting has a corresponding code.

    if (sdModuleInitialized) {
        openFile = SD.open(configFile); // Opens the config file
        if (openFile) {
            String tempRead = ""; // temporarily keeps track of the read characters from the config file
            int settingID = 0; // keeps track of the code of the next setting to be updated
            while (openFile.available()) {
                byte readByte = openFile.read(); // store read byte

                // if read byte is a separator or a line break, parse the current contents of tempRead into an integer
                // and set it to the currently selected setting
                if (readByte == 126 || readByte == 10) {
                    switch (settingID) {
                        case 0:
                            scanInterval = tempRead.toInt();
                            appliedSettings++;
                            break;

                        case 1:
                            depthOffset = tempRead.toInt();
                            appliedSettings++;
                            break;

                        case 2:
                            depthSamplingCount = tempRead.toInt();
                            appliedSettings++;
                            break;
                        
                        default:
                            break;
                    }

                    tempRead = "";
                    settingID++;
                }
                else if (readByte != 10) {
                    tempRead += (char)readByte;
                }
            }
            openFile.close();
        }
        else if (!openFile && serialDebug) {
            // TEMP serialDebug Get rid of this to save memory
            Serial.println("Could not open config file");
        }
    }

    if (appliedSettings == 3) {
        return true;
    }
    return false;
}

int scan() {
    // DATA.LOG Entry format
    // Time Start / Time End / Flow Rate / Depth / Offset

    String logEntry = "";
    DateTime now = RTC.now();
    uint32_t scanStart = now.unixtime();
    logEntry += scanStart;
    logEntry += char(126);

    double flowRate = checkFlowRate();
    long depth = checkDepth();

    now = RTC.now();
    lastScan = now.unixtime();
    logEntry += lastScan;
    logEntry += char(126);
    logEntry += flowRate;
    logEntry += char(126);
    logEntry += depth;
    logEntry += char(126);
    logEntry += depthOffset;
    logEntry += char(126);
    
    writeToFile(logEntry, logFile);
}