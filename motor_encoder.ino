/*
this code has been deleoped by masoud hassani (m.hasany@gmail.com)
last revision: July 2019

-> this is used to control a dc motor attached to a quadrature encoder
and drived by a VNH2SP30 motor driver.
-> the code reads the encoder signal and generates proper pwm signal
to rotate the motor to the desired setpoint.
-> the motor controller acts as slave and can receive commands and send
motor data to the master using i2c. device address is changed in deviceAddress
-> there are two settings for pwm:
    high res, 10-bit, 15625 Hz
    low res, 8-bit, 7812 Hz
-> timer and pwm frequency manipulation has been tested on atmega 328p
and atmega 2560.

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
const int deviceAddress = 0x01;
String receivedCommand = "";

// ----------------------- motor and driver setup ----------------------
const byte pinA = 2;                // this is the intrupt pin and the signal A of encoder
const byte pinB = 3;                // this is the intrupt pin and the signal B of encoder
const byte pinPWM = 10;             // pin for pwm signal of motor driver, do not change it from 10 unless you modify pwm setup
const byte pinEnable = 4;           // motor enable pin
const byte pinCurrent = 14;         // analog pin to read motor current
const byte pinDir1 = 7;             // pin for direction of motor driver
const byte pinDir2 = 8;             // pin for direction of motor driver
uint16_t   gearRatio = 155.572916;  // this value was corrected based on 16 full rotations
uint8_t    ppr = 11;                // pulse per rotation of encoder
bool       pwmHighRes = true;       // if true, 10 bit pwm is used else 8 bit
bool       reverseDir = false;

// ----------------------- counter variables ----------------------
volatile int32_t currentCount = 0;
int32_t setpointCount = 0;

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

// ----------------------- stuff for serial read ----------------------
String inString = "";

// ----------------------- motor current calculated from driver sensor ---------
float motorCurrent = 0;

// ----------------------- pwm stuff ----------------------
/*
large angle diff pid gains (for 8 volts):
float pGain = 0.0008;
float iGain = 0.007;
float dGain = 0.0000000;

small angle diff pid gains (for 8 volts):
float pGain = 0.005;
float iGain = 0.005;
float dGain = 0.0000000;
*/
float pwm = 0.0;
float pwmMax;
float minEffort = -1.0;
float maxEffort = 1.0;
float tol = 2.0;
// small angle diff gains
float pGain = 0.005;
float iGain = 0.005;
float dGain = 0.0000000;
// large angle diff gains
float pGainLarge = 0.0008;
float iGainLarge = 0.007;
float dGainLarge = 0.0000000;
//gain scheduling parameters.
float threshSmall = 10.0;          // setpoint angle smaller than this is considered small and small angle diff gains are used
float threshVerySmall = 2.0;       // setpoint angle smaller than this is considered very small
float verySmallMultiplier = 8.0;   // gain multiplication factor for very small angles
float threshLarge = 20.0;          // beyon this, angle is large and large angle diff gains are used

bool  windupGuard = true;
// initialize a pid controller
PID pid(pGain, iGain, dGain, windupGuard, minEffort, maxEffort, tol);

// ----------------------- data structure for motor data ---------------------
/*
define data structure for sending data
it creates a structure of 64 bytes including 8 indivisual bytes
and it sends the structure through i2c:
uint8_t byte0 angle
uint8_t byte1 velocity
uint8_t byte2 acceleration
uint8_t byte3 current
uint8_t byte4 temp
uint8_t byte5 empty integer
uint8_t byte6 boolean set 1
uint8_t byte7 boolean set 2
*/
// data structure setup
typedef struct motorData_t
{
    int8_t byte0;
    int8_t byte1;
    int8_t byte2;
    int8_t byte3;
    int8_t byte4;
    int8_t byte5;
    uint8_t bool0 : 1;
    uint8_t bool1 : 1;
    uint8_t bool2 : 1;
    uint8_t bool3 : 1;
    uint8_t bool4 : 1;
    uint8_t bool5 : 1;
    uint8_t bool6 : 1;
    uint8_t bool7 : 1;
    uint8_t byte7;
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
    pinMode(pinPWM, OUTPUT);
    pinMode(pinCurrent, INPUT);
    pinMode(pinDir1, OUTPUT);
    pinMode(pinDir2, OUTPUT);
    pinMode(pinEnable, OUTPUT);
    digitalWrite(pinEnable, HIGH);

    // initialize encoder pins and find their initial state
    FastGPIO::Pin<pinA>::setInputPulledUp();
    FastGPIO::Pin<pinB>::setInputPulledUp();
    bool stateA = FastGPIO::Pin<pinA>::isInputHigh();
    bool stateB = FastGPIO::Pin<pinB>::isInputHigh();

    if (stateA){
        attachInterrupt(digitalPinToInterrupt(pinA), fallingEdgeA, FALLING);
    }
    else
    {
        attachInterrupt(digitalPinToInterrupt(pinA), risingEdgeA, RISING);
    }

    if (stateB){
        attachInterrupt(digitalPinToInterrupt(pinB), fallingEdgeB, FALLING);
    }
    else
    {
        attachInterrupt(digitalPinToInterrupt(pinB), risingEdgeB, RISING);
    }

    // setup i2c communication
    Wire.begin(deviceAddress);
    Wire.onRequest(requestEvent);
    Wire.onReceive(receiveEvent);
    resetData();  // reset motor data

    t = micros();  // start timer
}

void loop()
{
    readSerial();
    calculateCount();
    calculateAngle();
    calculateVelocity();
    calculateAcceleration();
    calculateCurrent();
    calculatePWM();
    analogWrite(pinPWM, pwm);
    Serial.println(pwm);
    //Serial.println(reverseDir);
    //printValues();
    //Serial.print(status.motor.byte0);Serial.print("\t");Serial.print(status.motor.byte1);Serial.print("\n");
    delay(5);
}

void readSerial()
{
    while (Serial.available() > 0) {
        byte inChar = Serial.read();
        if (inChar != '\n') {
            inString += (char)inChar;
        }
        else{
            setpointAngle = inString.toFloat();
            // clear the string for new input:
            inString = "";
        }
    }
}

void calculateCount()
{
    setpointCount = ((setpointAngle - initialAngle) * ppr * gearRatio * 4 / 360.0);
}

void calculateAngle()
{
    currentAngle = 360.0 * currentCount / ppr / gearRatio / 4;
    status.motor.byte0 = int(currentAngle);
}

void calculateVelocity()
{
    dt = micros() - t;
    currentVelocity = (currentAngle - prevAngle) * 1000000 / dt;    // deg/sec
    status.motor.byte1 = int(currentVelocity*0.166666);  // rpm
    if (currentVelocity > 0){
        rotatingCW = true;
        status.motor.bool0 = true;
    }
    else{
        rotatingCW = false;
        status.motor.bool0 = false;
    }
    prevAngle = currentAngle;
}

void calculateAcceleration()
{
    currentAcceleration = (currentVelocity - prevVelocity) * 1000000 / dt;   //deg/s^2
    status.motor.byte2 = int(currentAcceleration);
    if (currentAcceleration > 0){
        isAccelerating = true;
        status.motor.bool1 = true;
    }
    else{
        isAccelerating = false;
        status.motor.bool1 = false;
    }
    prevVelocity = currentVelocity;
    t = micros();
}

void calculateCurrent()
{
    float vSense = (analogRead(pinCurrent)*5.0)/1024;    // motor current sensor voltage
    motorCurrent = vSense * 11370.0/1.5/2;               // motor current in mAmp, /2 comes from experiment
    status.motor.byte3 = int(motorCurrent/10.0);         // convert to int for serial transfer in (0~255) range
}

void calculatePWM()
{
    // update gains based on predefined schedule
    gainScheduling();

    // calculate the control effort and map it to the current pwm bit resolution
    pwm = pid.update(setpointCount, currentCount) * pwmMax;

    // account for direction change
    if (pwm < 0.0){
        pwm *= -1;
        reverseDir = true;
        digitalWrite(pinDir1, HIGH);
        digitalWrite(pinDir2, LOW);
    }
    else{
        reverseDir = false;
        digitalWrite(pinDir1, LOW);
        digitalWrite(pinDir2, HIGH);
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
        Serial.print("  Dir Reverse: ");
        Serial.print(reverseDir);
        Serial.print("  PWM: ");
        Serial.println(pwm);
    }
}

// if a falling edge in signal A is detected
void fallingEdgeA()
{
    bool state = FastGPIO::Pin<pinB>::isInputHigh();
    if (state){
        currentCount ++;
    }
    else{
        currentCount --;
    }
    attachInterrupt(digitalPinToInterrupt(pinA), risingEdgeA, RISING);
}

// if a rising edge in signal A is detected
void risingEdgeA()
{
    bool state = FastGPIO::Pin<pinB>::isInputHigh();
    if (state){
        currentCount --;
    }
    else{
        currentCount ++;
    }
    attachInterrupt(digitalPinToInterrupt(pinA), fallingEdgeA, FALLING);
}

// if a falling edge in signal B is detected
void fallingEdgeB()
{
    bool state = FastGPIO::Pin<pinA>::isInputHigh();
    if (state){
        currentCount --;
    }
    else{
        currentCount ++;
    }
    attachInterrupt(digitalPinToInterrupt(pinB), risingEdgeB, RISING);
}

// if a rising edge in signal Bis detected
void risingEdgeB()
{
    bool state = FastGPIO::Pin<pinA>::isInputHigh();
    if (state){
        currentCount ++;
    }
    else{
        currentCount --;
    }
    attachInterrupt(digitalPinToInterrupt(pinB), fallingEdgeB, FALLING);
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

void gainScheduling()
{
    /*
    before threshSmall, small angle diff gains are used.
    for very small angles, a multiplier is used over small angle gains
    between threshSmall and threshLarge, there is a linear transition.
    After threshLarge, the gain of large angle diff is used
    */
    float diff = abs(currentAngle - setpointAngle);
    if (diff <= threshSmall){
        if (diff <= threshVerySmall){
            pid.setGain(pGain*verySmallMultiplier, iGain, dGain);
        }
        else{
            pid.setGain(pGain, iGain, dGain);
        }
    }
    else{
        if (diff <= threshLarge){
            float p = pGain + (diff-threshSmall)*((pGainLarge-pGain)/(threshLarge-threshSmall));
            float i = iGain + (diff-threshSmall)*((iGainLarge-iGain)/(threshLarge-threshSmall));
            float d = dGain + (diff-threshSmall)*((dGainLarge-dGain)/(threshLarge-threshSmall));
            pid.setGain(p, i, d);
        }
        else{
            pid.setGain(pGainLarge, iGainLarge, dGainLarge);
        }
    }
}

// function to send out data to master
void requestEvent()
{
    byte buffer[8];
    buffer[0] = status.motor.byte0;
    buffer[1] = status.motor.byte1;
    buffer[2] = status.motor.byte2;
    buffer[3] = status.motor.byte3;
    buffer[4] = status.motor.byte4;
    buffer[5] = status.motor.byte5;
    buffer[6] = 0;
    buffer[7] = 0;
    Wire.write(buffer,8); // respond with message of 8 bytes as master expects
    //Wire.write((byte *)&status.motor, sizeof status.motor);
}

// function to receive data to master
void receiveEvent()
{
    while (Wire.available() > 0){
        char c = Wire.read();
        receivedCommand += c;
    }
    Serial.println(receivedCommand);
    setpointAngle = receivedCommand.toFloat();
    receivedCommand = "";
}

// reset/initialize data in the i2c bus
void resetData()
{
    status.motor.byte0 = 0;
    status.motor.byte1 = 0;
    status.motor.byte2 = 0;
    status.motor.byte4 = 0;
    status.motor.byte3 = 0;
    status.motor.byte5 = 0;
    status.motor.bool0 = false;
    status.motor.bool1 = false;
    status.motor.bool2 = false;
    status.motor.bool3 = false;
    status.motor.bool4 = false;
    status.motor.bool5 = false;
    status.motor.bool6 = false;
    status.motor.bool7 = false;
    status.motor.byte7 = 0;
}
