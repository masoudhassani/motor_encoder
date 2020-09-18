/*
this code has been deleoped by masoud hassani (m.hasany@gmail.com)
last revision: July 2019

-> this is used to control a dc motor attached to a quadrature encoder
and drived by a TB9051FTG motor driver.
-> the code reads the encoder signal and generates proper pwm signal
to rotate the motor to the desired setpoint.
-> the motor controller acts as slave and can receive commands and send
motor data to the master using i2c. device address is changed in deviceAddress
-> there are two settings for pwm:
    high res, 10-bit, 15625 Hz
    low res, 8-bit, 7812 Hz
-> timer and pwm frequency manipulation has been tested on atmega 328p

!!!note:
-> pinPWM should not change unless configPWM function is modified
-> this is a single motor version
-> PID library used in this code is not the arduino PID library. Download
the correct library from https://github.com/masoudhassani/PID
*/


#include <FastGPIO.h>
#include <PID.h>
#include <Wire.h>

// ----------------------- i2c stuff ---------------------------------
const int deviceAddress = 0x04;
String receivedCommand = "";
const uint8_t sizeOfData = 2;   // motor status data size sent to master
byte buffer[sizeOfData];
byte cmdBuffer[2];     // commands coming to motor are 2 bytes
int clockFrequency = 100000;
bool directPythonInterface = false;  // True for direct interface with a raspberry pi through i2c

// ----------------------- motor and driver setup ----------------------
const byte pinA = 2;                // this is the intrupt pin and the signal A of encoder
const byte pinB = 3;                // this is the intrupt pin and the signal B of encoder
const byte pinPWM1 = 9;            // this motor driver needs two pwm pins
const byte pinPWM2 = 10;             // pin for pwm signal of motor driver, do not change it from 9 and 10 unless you modify pwm setup
const byte pinOCC = 8;              // driver over-current reset
const byte pinOCM = 14;             // analog pin to read motor current
const byte pinEN = 7;               // motor driver EN pin
const byte pinENB = 6;              // motor driver ENB pin
float gearRatio = 155.563269888;  // this value was corrected based on 16 full rotations
float ppr = 11;                // pulse per rotation of encoder
bool  pwmHighRes = true;       // if true, 10 bit pwm is used else 8 bit
bool  reverseDir = true;

// ----------------------- counter variables ----------------------
volatile int32_t currentCount = 0;
int32_t setpointCount = 0;
volatile bool state;

// -------------------- motor shaft angle, velocity, acceleration -----------
float currentAngle = 0;
float initialAngle = 0;
float setpointAngle = 0;
float prevAngle = 0;
float currentVelocity = 0;
float prevVelocity = 0;
float currentAcceleration = 0;
bool  isAccelerating = false;
bool  rotatingCW = false;
uint32_t t = 0;   // timer to calculate velocity and acceleration
uint16_t dt = 0;
uint32_t counter = 0;

// ----------------------- stuff for serial read ----------------------
String inString = "";

// ----------------------- motor current calculated from driver sensor ---------
float motorCurrent = 0;
float motorCurrentFiltered = 0;
float filterConstant = 10000;  //microseconds, 0 means no low pass filter
uint16_t motorCurrentSetpoint = 150;    // milli amps
uint16_t motorCurrentMax = 2000;       // milli amps

// ----------------------- pwm stuff ----------------------
float contEff = 0.0;     // control effort between minEffort and maxEffort
int8_t dutyCycle = 0;    // this ranges from -100 to 100 where - sign means reverse direction. 100 is full speed and 0 is stop
float pwmMax;
float minEffort = -0.6;
float maxEffort = 0.6;
float pwm = pwmMax * maxEffort;
float maxIntegral = 1.0*maxEffort;
float tol = 0.0;
bool  windupGuard = true;

//gains
float pGain = 0.00001;
float iGain = 0.5;
float dGain = 0.0000000;

// initialize a pid controller
PID pid(pGain, iGain, dGain, windupGuard, minEffort, maxEffort, tol);

// ----------------------- data structure for motor data ---------------------
/*
define data structure for sending data
it creates a structure of 24 bites including one 16bit integers and
one 8bit integer We sends the structure through i2c:
uint16_t data0 current
uint8_t  data1 temp
*/
// data structure setup (some of data were eliminated bycommenting to decrease i2c transfer time)
typedef struct motorData_t
{
    uint16_t data0;   // current
    //uint8_t  data1;   // temp
};

typedef union dataPackage_t
{
    motorData_t motor;
    byte dataPackage[sizeof(motorData_t)];
};

dataPackage_t status;

// -------------------------------------------------------------------------
// -------------------------------------------------------------------------
// set up function of arduino, this runs once
void setup()
{
    //pinMode(LED_BUILTIN, OUTPUT);

    // initialize serial connection
    Serial.begin (9600);
    Serial.println("Configure PWM ...");
    if (pwmHighRes){
        Serial.println("High resolution PWM");
        pwmMax = 1023;
    }
    else{
        Serial.println("Low resolution PWM");
        pwmMax = 255;
    }

    // configure the pwm speed and resolution
    configPWM();

    // define pins
    pinMode(pinPWM1, OUTPUT);
    pinMode(pinPWM2, OUTPUT);
    pinMode(pinOCM, INPUT);
    pinMode(pinEN, OUTPUT);
    pinMode(pinENB, OUTPUT);
    pinMode(pinOCC, OUTPUT);
    digitalWrite(pinEN, HIGH);
    digitalWrite(pinENB, LOW);
    digitalWrite(pinOCC, LOW);

    // setup i2c communication
    Wire.begin(deviceAddress);
    //Wire.setClock(clockFrequency);
    Wire.onRequest(requestEvent);
    Wire.onReceive(receiveEvent);
    resetData();  // reset motor data

    t = micros();  // start timer

    pid.constraintIntegral(maxIntegral);
}

void loop()
{
    timer();
    readSerial();
    calculateCurrent();
    calculatePWM();
    dataPacking();
    commandInterpreter(receivedCommand); 
}

void readSerial()
{
    while (Serial.available() > 0) {
        byte inChar = Serial.read();
        if (inChar != '\n') {
            inString += (char)inChar;
        }
        else{
            motorCurrentSetpoint = inString.toInt();
            motorCurrentSetpoint = min(max(motorCurrentSetpoint, 0), motorCurrentMax);
            Serial.print("current setpoint: "); Serial.println(motorCurrentSetpoint);
            
            // clear the string for new input:
            inString = "";
        }
    }
}

void calculateCurrent()
{
    float vSense = (analogRead(pinOCM)*5.0)/1024;        // motor current sensor voltage
    motorCurrent = vSense * 2000;                        // motor current in mAmp, 500 mv per amp
    //motorCurrentFiltered = lowPassFilter(motorCurrent, motorCurrentFiltered, dt, filterConstant);
    status.motor.data0 = int(motorCurrent);
}

void calculatePWM()
{
    // update pid effort min/max
    //pid.setEffort(maxEffort, minEffort);

    // update gains based on predefined schedule
    //gainScheduling();

    // calculate the control effort and map it to the current pwm bit resolution
    contEff = pid.update(motorCurrentSetpoint, motorCurrent);
    pwm = contEff * pwmMax;

    // calculate pwm from the requested duty cycle
    // pwm = int(dutyCycle * pwmMax / 100);
    
    // windup guard
    float* ce;
    ce = pid.returnControlEfforts();
    maxIntegral = maxEffort - ce[0];
    pid.constraintIntegral(maxIntegral);

    // account for direction change
    if (pwm < 0.0){
        pwm *= -1;
        reverseDir = true;
        analogWrite(pinPWM1, pwm);
        analogWrite(pinPWM2, 0);
    }
    else{
        reverseDir = false;
        analogWrite(pinPWM1, 0);
        analogWrite(pinPWM2, pwm);
    }
}

void printValues()
{
    if (currentCount%1 == 0){
        Serial.print("Setpoint Count: ");
        Serial.print(setpointCount);
        Serial.print("  Current Count: ");
        Serial.print(currentCount);
        Serial.print("  Setpoint Angle: ");
        Serial.print(setpointAngle);
        Serial.print("  Current Angle: ");
        Serial.print(currentAngle);
        Serial.print("  PWM: ");
        Serial.println(pwm);
    }
}

// configure pwm
void configPWM()
{
    // if high res pwm is used
    if (pwmHighRes){
        /*
        sets the CS10 register bit to 1 to set a prescalar of 1 on timer 1 (pin 9 and 10)
        this changes the PWM frequency on these ports to 15625 Hz
        the following discards the last 5 bits of TCCR1B register and sets the 1st bit to 1 (CS10)
        by setting WGM10 to WGM12 to 1, fast PWM 10 bit is selected
        */
        TCCR1B = _BV(WGM12) | _BV(CS10);
        TCCR1A = _BV(WGM10) | _BV(WGM11);
        Serial.print("TCCR1B: ");Serial.println(TCCR1B, BIN);
        Serial.print("TCCR1A: ");Serial.println(TCCR1A, BIN);
        Serial.print("PWM Frequency: ");Serial.println(16000000/1/1024);
    }

    // if 8 bit pwm is selected
    else{
        /*
        sets the CS11 register bit to 1 to set a prescalar of 8 on timer 1 (pin 9 and 10)
        this changes the PWM frequency on these ports to 7812 Hz
        the following discards the last 5 bits of TCCR1B register and sets the 2nd bit to 1 (CS11)
        by setting WGM10 AND WGM12 to 1, fast PWM 8 bit is selected
        */
        TCCR1B = _BV(WGM12) | _BV(CS11);
        TCCR1A = _BV(WGM10);
        Serial.print("TCCR1B: ");Serial.println(TCCR1B, BIN);
        Serial.print("TCCR1A: ");Serial.println(TCCR1A, BIN);
        Serial.print("PWM Frequency: ");Serial.println(16000000/8/256);
    }
}

// modify pwm gains based on an scheduling variable such as motor current
void gainScheduling()
{

}

// function to prepare aa data pack for i2c
void dataPacking()
{
    // break motor current to two bytes
    buffer[0] = (status.motor.data0 >> 8) & 0xFF;
    buffer[1] = status.motor.data0  & 0xFF;
    // the rest of the data are one byte each
    //buffer[2] = status.motor.data1;
}

// function to send out data to master
void requestEvent()
{
    Wire.write(buffer,sizeOfData); // respond with message of sizeOfData bytes as master expects
}

// function to receive data to master
void receiveEvent()
{
    // clear the buffer
    byte buffer[2];
    for (uint8_t i=0; i<2; i++){
        buffer[i] = Wire.read();
    }

    // combine two byte to create an integer
    uint16_t setpoint = buffer[0];
    setpoint = setpoint << 8 | buffer[1];
    motorCurrentSetpoint = setpoint;
    //Serial.println(motorCurrentSetpoint);
}

// reset/initialize data in the i2c bus
void resetData()
{
    status.motor.data0 = 0;
    //status.motor.data1 = 0;
    // status.motor.data2 = 0;
    // status.motor.data3 = 0;
    // status.motor.data4 = 0;
    // status.motor.data5 = 0;
}

float lowPassFilter(float input, float output, float timeStep, float timeConstant)
{
    float value = ((timeStep * input) + (timeConstant * output)) / (timeStep + timeConstant);
    return value;
}

void timer()
{
    dt = micros() - t;
    t = micros();
    //Serial.println(dt);
}
