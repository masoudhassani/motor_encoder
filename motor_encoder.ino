#include <FastGPIO.h>
#include <PID.h>

// motor and driver setup
const byte pinA = 2;   // this is the intrupt pin and the signal A of encoder
const byte pinB = 3;   // this is the intrupt pin and the signal B of encoder
const byte pinPWM = 10; // pin for pwm signal of motor driver, do not change it from 10 unless you modify pwm setup
const byte pinEnable = 4;  // motor enable pin
const byte pinCurrent = 16; // analog pin to read motor current
const byte pinDir1 = 7;      // pin for direction of motor driver
const byte pinDir2 = 8;      // pin for direction of motor driver
uint16_t   gearRatio = 155.55; //150; // this value was corrected based on the observation
uint8_t    ppr = 11;
bool       pwmHighRes = true;  // if true, 10 bit pwm is used else 8 bit
bool       reverseDir = false;

// counter variables
volatile int32_t currentCount = 0;
int32_t setpointCount = 0;

// motor shaft angle
float currentAngle = 0;
float initialAngle = 0;
float setpointAngle = 0;

// stuff for serial read
String inString = "";

// pwm stuff
float pwm = 0.0;
float pwmMax;

// initialize a pid controller
/*
large angle diff pid gains:
float pGain = 0.0008;
float iGain = 0.007;
float dGain = 0.0000000;
*/
/*
small angle diff pid gains:
float pGain = 0.005;
float iGain = 0.005;
float dGain = 0.0000000;
*/
float minEffort = -1.0;
float maxEffort = 1.0;
float tol = 4.0;
// small angle diff gains
float pGain = 0.005;
float iGain = 0.005;
float dGain = 0.0000000;
// large angle diff gains
float pGainLarge = 0.0008;
float iGainLarge = 0.007;
float dGainLarge = 0.0000000;
//gain scheduling parameters.
float threshSmall = 10.0;
float threshLarge = 20.0;

bool  windupGuard = true;
PID pid(pGain, iGain, dGain, windupGuard, minEffort, maxEffort, tol);

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

 }

void loop()
{
    readSerial();
    calculateCount();
    calculateAngle();
    calculatePWM();
    analogWrite(pinPWM, pwm);
    //Serial.println(pwm);
    //printValues();
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
    // if (Serial.available()){
    //     byte inChar = Serial.read();
    //     //String content;
    //     if (inChar == 'p'){
    //         while (inChar != '\n'){
    //             byte inChar = Serial.read();
    //             inString += (char)inChar;
    //             Serial.println(inChar);
    //         }
    //         pGain = inString.toFloat();
    //         inString = "";
    //         Serial.print("P gain changed to ");Serial.println(pGain);
    //     }
    //     else if (inChar == 'a'){
    //         while (inChar != '\n'){
    //             byte inChar = Serial.read();
    //             inString += (char)inChar;
    //         }
    //         setpointAngle = inString.toFloat();
    //         inString = "";
    //         Serial.print("Setpoint angle changed to ");Serial.println(setpointAngle);
    //     }
    // }

}

void calculateCount()
{
    setpointCount = ((setpointAngle - initialAngle) * ppr * gearRatio * 4 / 360.0);
}

void calculateAngle()
{
    currentAngle = 360.0 * currentCount / ppr / gearRatio / 4;
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
    if (currentCount%2 == 0){
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
    between threshSmall and threshLarge, there is a linear transition.
    After threshLarge, the gain of large angle diff is used
    */
    float diff = abs(currentAngle - setpointAngle);
    if (diff <= threshSmall){
        pid.setGain(pGain, iGain, dGain);
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
