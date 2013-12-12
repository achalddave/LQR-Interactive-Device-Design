//Sensor Cuff Code

/*
 * Stephen McKinley
 * December 2013
 * CS294
 */

// Call me if you have any questions

// readability
typedef long ms;
typedef int  pin;

#define DEBUG

#ifdef DEBUG
#define LOG(x) Serial.print(x)
#define LOGLN(x) Serial.print(x)
#else
#define LOG(x)
#define LOGLN(x)
#endif

const pin zpin = A0;      // z-axis of the accelerometer (linear acceleration)
int accelReading = 0;     // The reading of the accelerometer

const pin gyroPin = A2;    // x xis of the gyro
int gyroReading = 0;        // gyro value (angular acceleration (pitch))

// Input/output leds
const pin greenLed = 9;
const pin yellowLed = 8;
const pin redLed = 12;

const pin requestNormalizeBut = 7;   // request normalize mode
const pin buttonPin2 = 10;         // currently unused

const pin accelOn = 11;  // Pull UP to wake the accelerometer
const pin gyroOn = 13;   // Pull DOWN to wake the gyro

/*
 * TODO: Basic description of the algorithm here
 */

// averaging window size
const int numReadings = 500;

// the running total for an average
long int totalAccel = 0;
long int totalGyro = 0;

// the final average
int averageAccel = 0;
int averageGyro = 0;

// boolean to start the normalization process
boolean normalizeRequest = false;

// has the device been normalized?
boolean normalized = false;

// maximum finding
int spike = 0; //stores the hammer spike value
int n = 0;     // used as an index (TODO: for what?)

// time variables
ms startTime = 0;  // start time (when hammer strikes)
ms peakTime  = 0;  // end time (at peak of kick acceleration)
ms moveTime  = 0;  // initial movement (first gyro value over threshold)

// calculated time variables
ms deltaTime    = 0; // Time to get leg to final position
ms responseTime = 0; // Time to leg's initial response (time between accel
                     // spike and gyro spike)

// threshold and search window
int reflexPeak = 0;            // stores the kick magnitude (gyro)
boolean thresholdFlag = false; // looks for an initial movement

int thresholdValue = 10; // the denotion of an initial gyro movement
// how long the device will look for a maximum in the gyro reading
ms searchTime = 500;

void setup() {
    // initialize the serial communications:
    Serial.begin(9600);

    // initialize DIO pins
    pinMode(requestNormalizeBut, INPUT);
    pinMode(buttonPin2, INPUT);

    pinMode(greenLed, OUTPUT);
    pinMode(yellowLed, OUTPUT);
    pinMode(redLed, OUTPUT);

    /* wake up the accel and gyro upon startup, can be changed to allow for
     * power mgmt */
    pinMode(accelOn,OUTPUT);
    pinMode(gyroOn,OUTPUT);
    digitalWrite(accelOn,HIGH);
    digitalWrite(gyroOn,LOW);
}

void loop() {
    // read the analog pins
    accelReading = analogRead(zpin)-averageAccel;
    gyroReading = analogRead(gyroPin)-averageGyro;

    LOG("accel - averageAccel: "); LOGLN(accelReading);

    if (digitalRead(requestNormalizeBut) == LOW) { // requested normalize
        digitalWrite(yellowLed, HIGH);
        normalizeRequest = true;
    } else {
        digitalWrite(yellowLed,LOW);
    }

    /* NOTE: This blocks loop()! */
    if (normalizeRequest) { // go into normalizing mode
        averageAccel = 0;   // reset variables
        totalAccel   = 0;
        averageGyro  = 0;
        totalGyro    = 0;

        for (int x = 0; x < numReadings; x++) {
            totalAccel += analogRead(zpin);
            totalGyro  += (-analogRead(gyroPin));
        }

        // Calculate the average
        averageAccel = totalAccel/numReadings;
        averageGyro  = totalGyro/numReadings;
        LOG("Average accel: "); LOGLN(averageAccel);

        digitalWrite(greenLed,HIGH); //some blinks as feedback (ready for readings)
        delay(100);
        digitalWrite(greenLed,LOW);
        delay(100);
        digitalWrite(greenLed,HIGH);
        delay(100);
        digitalWrite(greenLed,LOW);
        delay(100);
        digitalWrite(greenLed,HIGH);
        delay(100);
        digitalWrite(greenLed,LOW);
        delay(100);

        normalizeRequest = false; // reset normalize mode
        normalized       = true;  // i'm normalized!

    }

    /* Ensure that we've been normalized
     * TODO: Isn't this always true? */
    if (!normalizeRequest && normalized) {
        if (accelReading > 30) {
            // HAMMER TIME HAMMER TIME HAMMER TIME HAMMER TIME HAMMER TIME
            // (we saw a small spike, let's look for the max in spike)
            startTime = millis();
            LOG("Start ms: ");
            LOGLN(startTime);

            while ((millis()-startTime) < 30) { // Find the spike maximum
                accelReading = analogRead(zpin) - averageAccel;
                if (accelReading > spike) {
                    spike = accelReading;
                }
            }

            LOG("Spike: ")
            LOG(spike);

            reflexPeak = 0; // reset variable
            thresholdFlag = false;

            // Search for a gyro peak for reflex occurrence
            while ((millis()-startTime) < searchTime) {
                digitalWrite(greenLed,HIGH);

                // accelReading = analogRead(zpin)-averageAccel;
                gyroReading = -analogRead(gyroPin)-averageGyro;
                LOGLN(gyroReading);

                // An initial movement
                if(-gyroReading > thresholdValue && !thresholdFlag) {
                    moveTime = millis();
                    digitalWrite(redLed,HIGH);
                    thresholdFlag = true;
                }
                digitalWrite(redLed,LOW);

                // Further movement, keep track of peak
                if (-gyroReading > reflexPeak) {
                    reflexPeak = -gyroReading;
                    peakTime = millis();
                    LOGLN(peakTime);
                }
                normalized = false;
            }


            deltaTime    = peakTime - startTime; // time to get leg to final pos
            responseTime = moveTime - startTime; // time to first move the leg

            /*
             * use this outer loop to filter out known negatives (or
             * accomplish this on the app side
             *
             *   if (responseTime>40) {
             *     LOG(spike);
             *     LOGLN(" spike");
             *
             *     LOG(responseTime);
             *     LOGLN(" response time (ms)");
             *
             *     LOG(reflexPeak);
             *     LOGLN(" reflex peak");
             *
             *     LOG(deltaTime);
             *     LOGLN(" total time to peak (ms)");
             *
             *     LOGLN();
             *   } else {
             *     LOGLN("Bad Reading");
             *     LOGLN();
             *   }
            */

            // Reset variables
            startTime     = 0;
            deltaTime     = 0;
            responseTime  = 0;
            peakTime      = 0;
            moveTime      = 0;
            spike         = 0;
            thresholdFlag = false;
        }
    }
}
