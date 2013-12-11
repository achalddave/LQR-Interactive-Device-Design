//Sensor Cuff Code

//Stephen McKinley
//December 2013
//CS294

//Call me if you have any questions

const int zpin = A0;      // z-axis of the accelerometer (linear acceleration)
int accelReading = 0;     //The reading of the accelerometer
const int gyroPin= A2;    //x xis of the gyro
int gyroReading=0;        //gyro value (angular acceleration (pitch)

//INput/output ints
const int greenLed = 9;
const int yellowLed = 8;
const int redLed = 12;

const int buttonPin1 = 7;
const int buttonPin2 = 10;

const int accelOn = 11; //Pull UP to wake the accelerometer
const int gyroOn = 13; //Pull DOWN to wake the gyro

//averaging window size
const int numReadings = 500;

//the running total for an average
long int totalAccel = 0;
long int totalGyro = 0;

//the final average
int averageAccel = 0;
int averageGyro=0;
boolean normalizeRequest = false; //boolean to start the normalization process
boolean normalized = false; //has the device been normalized?

//maximum finding
int spike=0; //stores the hammer spike value
int n=0; //used as an index

//time variables
long startTime=0; //stores the millisecond start time (when hammer strikes)
long peakTime=0; //stores the millisecond end time (at peak of kick acceleration)
long moveTime=0; //stores the millisecond time of initial movement (first gyro value over threshold)

//calculated time variables
long deltaTime=0; //calculated elapsed time
long responseTime=0; //calculater time until initial movement (time between accel spike and gyro spike)

//threshold and search window
int reflexPeak=0; //stores the kick magnitude (gyro)
boolean thresholdFlag = false; //looks for an initial movement

int thresholdValue = 10; //the denotion of an initial movement of the gyroscope
int searchTime=500; // the time during which the device will look for a maximum in the gyro reading


void setup()
{
    // initialize the serial communications:
    Serial.begin(9600);

    //initialize DIO pins
    pinMode(buttonPin1, INPUT);
    pinMode(buttonPin2, INPUT);

    pinMode(greenLed, OUTPUT);
    pinMode(yellowLed, OUTPUT);
    pinMode(redLed, OUTPUT);

    //wake up the accel and gyro upon startup, can be changed to allow for power mgmt
    pinMode(accelOn,OUTPUT);
    pinMode(gyroOn,OUTPUT);
    digitalWrite(accelOn,HIGH);
    digitalWrite(gyroOn,LOW);
}

void loop()
{
    //read the analog pins
    accelReading=analogRead(zpin)-averageAccel;
    gyroReading=analogRead(gyroPin)-averageGyro;
    //Serial.print(analogRead(zpin)-average); //(for debug) Read the z axis of the accel and subtract the normalizing factor
    // Serial.println();


    if(digitalRead(buttonPin1)==LOW) //This button press requests normalize mode
    {
        digitalWrite(yellowLed,HIGH);
        normalizeRequest=true; //Request flag for normalize mode raised
    }
    else
    {
        digitalWrite(yellowLed,LOW);
    }

    if (normalizeRequest==true) // go into the normalizing mode (norm loop)
    {
        averageAccel=0; //reset variables
        totalAccel=0;
        averageGyro=0;
        totalGyro=0;

        for(int x = 0; x < numReadings ; x++)
        {
            totalAccel=totalAccel+analogRead(zpin); // add up a total for the number of readings
            totalGyro=totalGyro+(-analogRead(gyroPin));
        }

        averageAccel=totalAccel/numReadings; // calculate the average
        averageGyro=totalGyro/numReadings;
        //Serial.println(averageAccel);  //DEBUG LINE

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

        normalizeRequest=false; // reset the normalize mode
        normalized=true; //has the device been normalized?

    } //end of normalizing mode

    if (normalizeRequest==false&&normalized==true)  //check that the device has been normalized and we are not in the norm loop
    {
        if (accelReading>30) // this triggers a spike, start looking for the spike maximum
        {
            startTime=millis();
            //Serial.println(startTime);
            //Serial.println(" ST");

            while ((millis()-startTime)<30)                  //Find the spike maximum in the next milliseconds
            {                                                //
                accelReading=analogRead(zpin)-averageAccel;  //
                if(accelReading>spike)                       //
                {                                            //
                    spike=accelReading;                      //
                    //startTime=millis();                    //
                }                                            //
                //
            }

            //
            //Serial.print(spike);      //
            //Serial.println(" spike"); //For Debugging
            //spike=0;                  //

            reflexPeak=0; //reset variable
            thresholdFlag=false;

            while((millis()-startTime)<searchTime) //look for a peak in the next searchTime ms
            {
                //wait to look for the next peak
                digitalWrite(greenLed,HIGH);

                //accelReading=analogRead(zpin)-averageAccel;
                gyroReading=-analogRead(gyroPin)-averageGyro;
                //Serial.println(gyroReading);

                if(-gyroReading>thresholdValue&&thresholdFlag==false) //look for an initial movement
                {
                    moveTime=millis(); //mark the time
                    digitalWrite(redLed,HIGH);
                    thresholdFlag=true;
                }
                digitalWrite(redLed,LOW);

                if(-gyroReading>reflexPeak)
                {
                    reflexPeak=-gyroReading;
                    peakTime=millis(); //mark the time
                    //Serial.println(peakTime);
                }
                normalized=false;
            }


            deltaTime=peakTime-startTime;  //this is the time it takes to get the leg to its final position
            responseTime=moveTime-startTime; //this is the time it takes to first move the leg

            //if (responseTime>40) //use this outer loop to filter out known negatives (or accomplish this on the app side
            //{
            Serial.print(spike);
            Serial.println(" spike");

            Serial.print(responseTime);
            Serial.println(" response time (ms)");

            Serial.print(reflexPeak);
            Serial.println(" reflex peak");

            Serial.print(deltaTime);
            Serial.println(" total time to peak (ms)");

            Serial.println();
            //}
            //else
            //{
            //  Serial.println("Bad Reading");
            //  Serial.println();
            //}

            //Reset variables
            startTime=0;
            deltaTime=0;
            responseTime=0;
            peakTime=0;
            moveTime=0;
            spike=0;
            thresholdFlag=false;


        }


    }
}
