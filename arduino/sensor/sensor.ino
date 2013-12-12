#include <SoftwareSerial.h>
#include "BGLib.h"

/** =============== **/
/** Bluetooth setup **/
/** =============== **/

// BLE STATE TRACKING (UNIVERSAL TO JUST ABOUT ANY BLE PROJECT)

// BLE state machine definitions
#define BLE_STATE_STANDBY           0
#define BLE_STATE_SCANNING          1
#define BLE_STATE_ADVERTISING       2
#define BLE_STATE_CONNECTING        3
#define BLE_STATE_CONNECTED_MASTER  4
#define BLE_STATE_CONNECTED_SLAVE   5

// BLE state/link status tracker
uint8_t ble_state = BLE_STATE_STANDBY;
uint8_t ble_encrypted = 0;  // 0 = not encrypted, otherwise = encrypted
uint8_t ble_bonding = 0xFF; // 0xFF = no bonding, otherwise = bonding handle

// HARDWARE CONNECTIONS AND GATT STRUCTURE SETUP

/* NOTE: this assumes you are using one of the following firmwares:
 *  - BGLib_U1A1P_38400_noflow
 *  - BGLib_U1A1P_38400_noflow_wake16
 *  - BGLib_U1A1P_38400_noflow_wake16_hwake15
 * If not, then you may need to change the pin assignments and/or
 * GATT handles to match your firmware.
 */

#define LED_PIN         13  // Arduino Uno LED pin
#define BLE_WAKEUP_PIN  5   // BLE wake-up pin
#define BLE_RESET_PIN   6   // BLE reset pin (active-low)

#define GATT_HANDLE_C_RX_DATA   17  // 0x11, supports "write" operation
#define GATT_HANDLE_C_TX_DATA   20  // 0x14, supports "read" and "indicate" operations

// use SoftwareSerial on pins D2/D3 for RX/TX (Arduino side)
SoftwareSerial bleSerialPort(2, 3);

/* create BGLib object:
 *  - use SoftwareSerial por for module comms
 *  - use nothing for passthrough comms (0 = null pointer)
 *  - enable packet mode on API protocol since flow control is unavailable
 */
BGLib ble112((HardwareSerial *)&bleSerialPort, 0, 1);

#define BGAPI_GET_RESPONSE(v, dType) dType *v = (dType *)ble112.getLastRXPayload()

// Figure out when we've just connected
bool disconnected = true;
unsigned long connection_time;

// ARDUINO APPLICATION SETUP AND LOOP FUNCTIONS

/*
 * PIN CONNECTIONS:
 * ===============
 * D2 -> P04
 * D3 -> P05
 * D5 -> P16
 * D6 -> RST
 */

// readability
typedef long ms;
typedef int  pin;

#define DEBUG

#ifdef DEBUG
#define LOG(x) Serial.print(x)
#define LOGLN(x) Serial.println(x)
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
    sensorSetup();
    bleSetup();
}

void sensorSetup() {
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

void bleSetup() {
    // initialize BLE reset pin (active-low)
    pinMode(BLE_RESET_PIN, OUTPUT);
    digitalWrite(BLE_RESET_PIN, HIGH);

    /* initialize BLE wake-up pin to allow (not force) sleep mode (assumes
     * active-high)
     */
    pinMode(BLE_WAKEUP_PIN, OUTPUT);
    digitalWrite(BLE_WAKEUP_PIN, LOW);

    // set up internal status handlers (these are technically optional)
    ble112.onBusy = onBusy;
    ble112.onIdle = onIdle;
    ble112.onTimeout = onTimeout;

    /* ONLY enable these if you are using the <wakeup_pin> parameter in your
     * firmware's hardware.xml file
     *
     * BLE module must be woken up before sending any UART data
     */
    ble112.onBeforeTXCommand = onBeforeTXCommand;
    ble112.onTXCommandComplete = onTXCommandComplete;

    // set up BGLib event handlers
    ble112.ble_evt_system_boot = my_ble_evt_system_boot;
    ble112.ble_evt_connection_status = my_ble_evt_connection_status;
    ble112.ble_evt_connection_disconnected = my_ble_evt_connection_disconnect;
    ble112.ble_evt_attributes_value = my_ble_evt_attributes_value;

    /* open Arduino USB serial (and wait, if we're using Leonardo)
     * use 38400 since it works at 8MHz as well as 16MHz */
    Serial.begin(9600);
    while (!Serial);

    // open BLE software serial port
    bleSerialPort.begin(38400);

    // reset module (maybe not necessary for your application)
    digitalWrite(BLE_RESET_PIN, LOW);
    delay(5); // wait 5ms
    digitalWrite(BLE_RESET_PIN, HIGH);
    LOGLN("Reset bluetooth");
}

void loop() {
    // keep polling for new data from BLE
    ble112.checkActivity();

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

            LOG("Spike: ");
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
            sendData();

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

/* Sends deltaTime, responseTime, spike over bluetooth */
void sendData() {
    if (ble_state == BLE_STATE_CONNECTED_SLAVE 
            || ble_state == BLE_STATE_CONNECTED_MASTER) {
        byte data[12] = {
            (deltaTime >> 24) & 0xff,
            (deltaTime >> 16) & 0xff,
            (deltaTime >> 8) & 0xff,
            deltaTime & 0xff,

            (responseTime >> 24) & 0xff,
            (responseTime >> 16) & 0xff,
            (responseTime >> 8) & 0xff,
            responseTime & 0xff,

            (spike >> 24) & 0xff,
            (spike >> 16) & 0xff,
            (spike >> 8) & 0xff,
            spike & 0xff,
        };

        ble112.ble_cmd_attributes_write(GATT_HANDLE_C_TX_DATA, 0, 12, data);
    } else {
        Serial.println("Attempted to send data, but wasn't connected!");
    }
}

// ================================================================
// INTERNAL BGLIB CLASS CALLBACK FUNCTIONS
// ================================================================

// called when the module begins sending a command
void onBusy() { }

// called when the module receives a complete response or "system_boot" event
void onIdle() { }

// called when the parser does not read the expected response in the specified
// time limit
void onTimeout() {
    Serial.println("Timeout");

    // reset module (might be a bit drastic for a timeout condition though)
    digitalWrite(BLE_RESET_PIN, LOW);
    delay(5); // wait 5ms
    digitalWrite(BLE_RESET_PIN, HIGH);

    Serial.println("TIMEOUT RESET");
    // set state to ADVERTISING
    ble_state = BLE_STATE_ADVERTISING;
}

// called immediately before beginning UART TX of a command
void onBeforeTXCommand() {
    Serial.println("Before TX");
    /* wake module up (assuming here that digital pin 5 is connected to the BLE
     * wake-up pin) */
    digitalWrite(BLE_WAKEUP_PIN, HIGH);

    /* wait for "hardware_io_port_status" event to come through, and parse it
     * (and otherwise ignore it) */
    uint8_t *last;
    Serial.println("Checking activity in before TX");
    while (1) {
        ble112.checkActivity();
        last = ble112.getLastEvent();
        if (last[0] == 0x07 && last[1] == 0x00) break;
    }
    Serial.println("Completed checking activity in before TX");

    /* give a bit of a gap between parsing the wake-up event and allowing the
     * command to go out */
    delayMicroseconds(1000);
}

// called immediately after finishing UART TX
void onTXCommandComplete() {
    Serial.println("TX complete");
    /* allow module to return to sleep (assuming here that digital pin 5 is
     * connected to the BLE wake-up pin) */
    digitalWrite(BLE_WAKEUP_PIN, LOW);
}



// ================================================================
// APPLICATION EVENT HANDLER FUNCTIONS
// ================================================================

void my_ble_evt_system_boot(const ble_msg_system_boot_evt_t *msg) {
#ifdef DEBUG
    Serial.print("###\tsystem_boot: { ");
    Serial.print("major: "); Serial.print(msg->major, HEX);
    Serial.print(", minor: "); Serial.print(msg->minor, HEX);
    Serial.print(", patch: "); Serial.print(msg->patch, HEX);
    Serial.print(", build: "); Serial.print(msg->build, HEX);
    Serial.print(", ll_version: "); Serial.print(msg->ll_version, HEX);
    Serial.print(", protocol_version: "); Serial.print(msg->protocol_version, HEX);
    Serial.print(", hw: "); Serial.print(msg->hw, HEX);
    Serial.println(" }");
#endif

    startAdvertising();
}

void my_ble_evt_connection_status(const ble_msg_connection_status_evt_t *msg) {
#ifdef DEBUG
    Serial.print("###\tconnection_status: { ");
    Serial.print("connection: "); Serial.print(msg->connection, HEX);
    Serial.print(", flags: "); Serial.print(msg->flags, HEX);
    Serial.print(", address: ");
    // this is a "bd_addr" data type, which is a 6-byte uint8_t array
    for (uint8_t i = 0; i < 6; i++) {
        if (msg->address.addr[i] < 16) Serial.write('0');
        Serial.print(msg->address.addr[i], HEX);
    }
    Serial.print(", address_type: "); Serial.print(msg->address_type, HEX);
    Serial.print(", conn_interval: "); Serial.print(msg->conn_interval, HEX);
    Serial.print(", timeout: "); Serial.print(msg->timeout, HEX);
    Serial.print(", latency: "); Serial.print(msg->latency, HEX);
    Serial.print(", bonding: "); Serial.print(msg->bonding, HEX);
    Serial.println(" }");
#endif

    /* "flags" bit description:
     *  - bit 0: connection_connected
     *           Indicates the connection exists to a remote device.
     *  - bit 1: connection_encrypted
     *           Indicates the connection is encrypted.
     *  - bit 2: connection_completed
     *           Indicates that a new connection has been created.
     *  - bit 3; connection_parameters_change
     *           Indicates that connection parameters have changed, and is set
     *           when parameters change due to a link layer operation.
     */

    // check for new connection established
    if ((msg->flags & 0x05) == 0x05) { // bit 0, 2 on
        // track state change based on last known state, since we can connect two ways
        Serial.println("Connected, saving millis");
        disconnected = false;
        connection_time = millis();
        if (ble_state == BLE_STATE_ADVERTISING) {
            ble_state = BLE_STATE_CONNECTED_SLAVE;
        } else {
            ble_state = BLE_STATE_CONNECTED_MASTER;
        }
    }

    // update "encrypted" status
    ble_encrypted = msg->flags & 0x02;

    // update "bonded" status
    ble_bonding = msg->bonding;

}

/*
 * We were disconnected!
 */
void my_ble_evt_connection_disconnect(const struct ble_msg_connection_disconnected_evt_t *msg) {
    disconnected = true;
#ifdef DEBUG
    Serial.print("###\tconnection_disconnect: { ");
    Serial.print("connection: "); Serial.print(msg->connection, HEX);
    Serial.print(", reason: "); Serial.print(msg->reason, HEX);
    Serial.println(" }");
#endif

    /* after disconnection, resume advertising as discoverable/connectable
     * (with user-defined advertisement data) */
    ble112.ble_cmd_gap_set_mode(BGLIB_GAP_USER_DATA, BGLIB_GAP_UNDIRECTED_CONNECTABLE);
    while (ble112.checkActivity(1000));

    // set state to ADVERTISING
    ble_state = BLE_STATE_ADVERTISING;

    // clear "encrypted" and "bonding" info
    ble_encrypted = 0;
    ble_bonding = 0xFF;
}

/*
 * Called when we receive some data.
 */
void my_ble_evt_attributes_value(const struct ble_msg_attributes_value_evt_t *msg) {
    #ifdef DEBUG
        Serial.print("###\tattributes_value: { ");
        Serial.print("connection: "); Serial.print(msg -> connection, HEX);
        Serial.print(", reason: "); Serial.print(msg -> reason, HEX);
        Serial.print(", handle: "); Serial.print(msg -> handle, HEX);
        Serial.print(", offset: "); Serial.print(msg -> offset, HEX);
        Serial.print(", value_len: "); Serial.print(msg -> value.len, HEX);
        Serial.print(", value_data: ");
        // this is a "uint8array" data type, which is a length byte and a uint8_t* pointer
        for (uint8_t i = 0; i < msg -> value.len; i++) {
            if (msg -> value.data[i] < 16) Serial.write('0');
            Serial.print(msg -> value.data[i], HEX);
        }
        Serial.println(" }");
    #endif
}

void startAdvertising() {
    /* set advertisement interval to 200-300ms, use all advertisement channels
     * (note min/max parameters are in units of 625 uSec) */
    ble112.ble_cmd_gap_set_adv_parameters(320, 480, 7);
    while (ble112.checkActivity(1000));

    // HANDLE CUSTOM ADVERTISEMENT PACKETS
    // ===================================

    /* build custom advertisement data
     * default BLE stack value: 0201061107e4ba94c3c9b7cdb09b487a438ae55a19 */
    uint8 adv_data[] = {
        0x02, // field length
        BGLIB_GAP_AD_TYPE_FLAGS, // field type (0x01)
        BGLIB_GAP_AD_FLAG_GENERAL_DISCOVERABLE | BGLIB_GAP_AD_FLAG_BREDR_NOT_SUPPORTED, // data (0x02 | 0x04 = 0x06)
        0x11, // field length
        BGLIB_GAP_AD_TYPE_SERVICES_128BIT_ALL, // field type (0x07)
        0xe4, 0xba, 0x94, 0xc3, 0xc9, 0xb7, 0xcd, 0xb0, 0x9b, 0x48, 0x7a, 0x43, 0x8a, 0xe5, 0x5a, 0x19
    };

    // set custom advertisement data
    ble112.ble_cmd_gap_set_adv_data(0, 0x15, adv_data);
    while (ble112.checkActivity(1000));

    /* build custom scan response data (i.e. the Device Name value)
     * default BLE stack value: 140942474c69622055314131502033382e344e4657 */
    uint8 sr_data[] = {
        0x14, // field length
        BGLIB_GAP_AD_TYPE_LOCALNAME_COMPLETE, // field type
        '1', '2', '3', ' ', 'F', 'a', 'k', 'e', ' ', 'S', 'r', 'e', 'e', 't', '0', '0', ':', '0', '0'
    };

    // get BLE MAC address
    ble112.ble_cmd_system_address_get();
    while (ble112.checkActivity(1000));
    BGAPI_GET_RESPONSE(r0, ble_msg_system_address_get_rsp_t);

    /* assign last three bytes of MAC address to ad packet friendly name
     * (instead of 00:00:00 above) */
    sr_data[13] = (r0->address.addr[2] / 0x10) + 48 + ((r0->address.addr[2] / 0x10) / 10 * 7); // MAC byte 4 10's digit
    sr_data[14] = (r0->address.addr[2] & 0xF)  + 48 + ((r0->address.addr[2] & 0xF ) / 10 * 7); // MAC byte 4 1's digit
    sr_data[16] = (r0->address.addr[1] / 0x10) + 48 + ((r0->address.addr[1] / 0x10) / 10 * 7); // MAC byte 5 10's digit
    sr_data[17] = (r0->address.addr[1] & 0xF)  + 48 + ((r0->address.addr[1] & 0xF ) / 10 * 7); // MAC byte 5 1's digit
    sr_data[19] = (r0->address.addr[0] / 0x10) + 48 + ((r0->address.addr[0] / 0x10) / 10 * 7); // MAC byte 6 10's digit
    sr_data[20] = (r0->address.addr[0] & 0xF)  + 48 + ((r0->address.addr[0] & 0xF ) / 10 * 7); // MAC byte 6 1's digit

    // set custom scan response data (i.e. the Device Name value)
    ble112.ble_cmd_gap_set_adv_data(1, 0x15, sr_data);
    while (ble112.checkActivity(1000));

    /* put module into discoverable/connectable mode (with user-defined
     * advertisement data) */
    ble112.ble_cmd_gap_set_mode(BGLIB_GAP_USER_DATA, BGLIB_GAP_UNDIRECTED_CONNECTABLE);
    while (ble112.checkActivity(1000));

    // set state to ADVERTISING
    ble_state = BLE_STATE_ADVERTISING;
}
