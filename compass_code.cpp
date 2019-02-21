#include "mbed.h"
// servo pulswidths in microseconds
#define SERVO_MIN 1000 // 1000
#define SERVO_MID 1500 // 1500
#define SERVO_MAX 2000 // 2000

BusOut leds(LED4,LED3,LED2,LED1);
Serial compass (p9, p10); // tx, rx
PwmOut servo(p21); // control signal for the servo
Timer timer;

void feedBack(); // the closer to the target the compass is pointing, the more LEDs light up
void getPos(); // read the compass heading and convert to degrees
void initCompass(); // initialise the compass
void initServo(); // initialise the servo
void moveServo(); // move the servo
float updatePI(float current,float target,float dt); // control algorithm
float antiwindup(float U, float U_hat, float dt); // anti-windup
float saturation(float U_hat); // saturation function
void valid();

// global variables
char getXYZ = 0x13; // command array for read and write
int pulse = SERVO_MID;
float output;

//controller variables
float pos; // measured position
float target = 0.0; // North
float E,Ei;  // proportional and integral error;
float U_hat; // unsaturated actuation signal
float U; // saturated actuation signal
float U_AW; // anti-windup signal
float U_MIN=SERVO_MIN-SERVO_MID, U_MAX=SERVO_MAX-SERVO_MID; // servo pulse signal saturation bounds
float Esat, Esati; // proportional and integral anti-windup error

// controller coefficients
float K = 0.1;
float Ti = 10000; 
float Tr=sqrt(Ti); // anti-windup gain 


// main code

int main()
{
    initCompass(); // initialise compass

    initServo(); // initialise servo

    Ei=0.0; // initialise integral error
     
    Esati=0.0; // initialise anti-windup error

    U_AW=0.0; // initialise anti-windup compensation    

    timer.start(); // initialise timer

    while (1) {
                
        getPos(); // get current position
        
        output = updatePI(pos,target,timer.read()); //calculate control signal
        
        timer.reset(); // reset timer
        
        pulse=SERVO_MID-output; // generate pulse signal for servo
        
        valid(); // check if pulse signal is within allowed limits

        moveServo(); // move servo

        feedBack(); // update LEDs to show where compass is facing

    }
}

void valid()
{
    if (pulse > SERVO_MAX) {
        pulse = SERVO_MAX;
        leds = 7;
    }
    if (pulse < SERVO_MIN) {
        pulse = SERVO_MIN;
        leds = 14;
    }
}

float updatePI(float pos,float target,float dt)
{

    // add your controller here
	float U_PI = 0.0;
    E = target - pos;
    Ei = Ei + E*dt

    U_PI = K * (E + 1/Ti*Ei);
    // U_AW=antiwindup(U,U_hat,dt); // anti-windup 

    U_hat=U_PI+U_AW;

    U=saturation(U_hat);    

    return U;
}

float antiwindup(float U, float U_hat, float dt)
{

   // add your anti-windup function here
    
}

float saturation(float U_hat) // saturation function
{
    if (U_hat > U_MAX)     
        U=U_MAX;
    else if (U_hat < U_MIN)
        U = U_MIN;
    else
        U=U_hat;
    
    return U;
    
}

void moveServo()
{
    servo.pulsewidth_us(pulse); // set pulsewidth - argument must be an integer

}


void initServo()
{
    servo.period(1.0/50.0);  // servo PWM waveform has 50 Hz frequency  1.0/50.0
    moveServo();
    leds = 15;
    wait_ms(2000);
    leds = 0;
}

void initCompass()
{
    compass.baud(9600);
    compass.format(8,SerialBase::None,2);
    compass.putc(getXYZ);  // Send operation
}

void getPos()
{
    compass.putc(getXYZ);  // Send operation

    char msb = compass.getc();  // read the two-byte echo result
    char lsb = compass.getc();

    pos =0.1*((msb << 8) + lsb);  //convert to degrees

    if (pos>180) {
        pos=pos-360; // converts -180 to 180
    }
}

void feedBack()
// show on mbed LED's where the compass is facing
{
    if (pos > target + 120) {
        leds = 1;
    } else if (pos < target - 120) {
        leds = 8;
    } else if (pos > target + 60) {
        leds = 3;
    } else if (pos < target - 60) {
        leds = 12;
    } else if (pos > target + 0.5) {
        leds = 7;
    } else if (pos < target - 0.5) {
        leds = 14;
    } else {
        leds = 15;
    }
}