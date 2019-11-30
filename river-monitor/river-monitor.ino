const int pingPin = 7;  // Trigger Pin of Ultrasonic Sensor
const int echoPin = 6;  // Echo Pin of Ultrasonic Sensor
long depthOffset = 14; // CM distance of the ultrasonic sensor from the bottom of the body of water

void setup() {
  Serial.begin(9600);
  pinMode(pingPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop() {
    long depth = checkDepth();
    Serial.print("Water Depth : ");
    Serial.print(depth);
    Serial.println(" cm");
    delay(1000);
}

long microsecondsToCentimeters(long microseconds) {
   return microseconds / 29 / 2;
}

// Measures the depth for 5 seconds and returns the most common value
long checkDepth() {
    long readValues[50];

    // loop 50 times with a 100ms delay between each read
    for (int x = 0; x < 50; x++) {
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

    long uniqueValues[50]; // Stores all the unique values read
    int uniqueValueInstances[50]; // Keeps track of the instances a value has appeared in readValues
    // Fill uniqueValueInstances with zeroes
    for (int x = 0; x < 50; x++) {
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