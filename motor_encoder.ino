#include <FastGPIO.h>
#include <PID.h>

// motor and driver setup
const byte pinA = 2;   // this is the intrupt pin and the signal A of encoder
const byte pinB = 3;   // this is the intrupt pin and the signal B of encoder
const byte pinPWM = 10; // pin for pwm signal of motor driver, do not change it from 10 unless you modify pwm setup
const byte pinCurrent = 15; // analog pin to read motor current
const byte pinDir = 6;      // pin for direction of motor driver
uint16_t   gearRatio = 150;
uint8_t    ppr = 11;
bool       pwmHighRes = true;  // if true, 10 bit pwm is used else 8 bit

// counter variables
volatile int currentCount = 0;
int setpointCount = 0;

// motor shaft angle
float currentAngle = 0;
float initialAngle = 0;
float setpointAngle = 0;

// stuff for serial read
String inString = "";

// pwm stuff
uint16_t pwm = 0;

// initialize a pid controller
PID pid(0.1, 0.01, 0.0, true, 0.0, 1.0, 0.0001);

 void setup()
{
    // initialize serial connection
    Serial.begin (9600);
    Serial.println("Configure PWM ...");
    if (pwmHighRes){
        Serial.println("High resolution PWM");
    }
    else{
        Serial.println("Low resolution PWM");
    }

    // configure the pwm speed and resolution
    configPWM();

    pinMode(pinPWM, OUTPUT);

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
    calculatePWM();
    //printcurrentCount(currentCount);
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
    setpointCount = int((setpointAngle - initialAngle) * ppr * gearRatio * 4 / 360);
}

void calculatePWM()
{
    pwm = pid.update(setpointCount, currentCount);
}

void printcurrentCount(int cnt)
{
    if (cnt%10 == 0){
        Serial.print("Position: ");
        Serial.println(cnt);
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
