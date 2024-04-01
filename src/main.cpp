#include <Arduino.h>
#include <Servo.h>

#include <RF24.h>
#include <SPI.h>
#include <nRF24L01.h>

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

// ------------ for the IMU MPU6050 with the DMP using I2Cdev lib -----------------
MPU6050 mpu; // MPU6050 mpu(0x69); // <-- use for AD0 high
#define INTERRUPT_PIN 33 // use pin 33 for teensy could be any digital
// MPU control/status vars
bool dmpReady = false; // set true if DMP init was successful
uint8_t mpuIntStatus; // holds actual interrupt status byte from MPU
uint8_t devStatus; // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize; // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount; // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
// orientation/motion vars
Quaternion q; // [w, x, y, z]         quaternion container
VectorInt16 aa; // [x, y, z]            accel sensor measurements
VectorInt16 aaReal; // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector
float euler[3]; // [psi, theta, phi]    Euler angle container
float ypr[3]; // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

//=============================================================================
// ===                      define motors & servos                         ====
//=============================================================================
#define PIN_FL 40
#define PIN_RL 41
#define PIN_RR 24
#define PIN_FR 25

#define PIN_SERVO_FL 37
#define PIN_SERVO_RL 15
#define PIN_SERVO_RR 9
#define PIN_SERVO_FR 28

#define PIN_SERVO_AL 36
#define PIN_SERVO_AR 29
#define PIN_SERVO_D 6
#define PIN_SERVO_P 5

Servo ESC_FL;
Servo ESC_RL;
Servo ESC_RR;
Servo ESC_FR;

int PWM_FL = 1000;
int PWM_RL = 1000;
int PWM_RR = 1000;
int PWM_FR = 1000;

Servo servo_FL;
Servo servo_RL;
Servo servo_RR;
Servo servo_FR;

int servo_pwm_FL = 0;
int servo_pwm_RL = 0;
int servo_pwm_RR = 0;
int servo_pwm_FR = 0;

Servo servo_AL;
Servo servo_AR;
Servo servo_D;
Servo servo_P;

int pwm_AL = 0;
int pwm_AR = 0;
int pwm_D = 0;
int pwm_P = 0;

int throtle = 1000;
int pitch = 0;
int roll = 0;
int yaw = 0;

// ================================================================
// ===            NRF24L01 Variable declaration                ====
// ================================================================
/*
connection Teensy 4 a NRF24L01
GND  -> GND
VCC  -> 3.3V (mettre une capa de 10uF)
CE   -> 31 select RX/TX mode (non lie a SPI donc peut etre changee)
CSN  -> 10 chip select SPI
SCK  -> 13
MOSI -> 11
MISO -> 12
IRQ  -> 32  (non lie a SPI donc peut etre changee)
*/
RF24 radio(31, 10); // CE, CSN
const byte address[6] = "00001"; // must be the same on both NRF
volatile bool messageAvailable = false; // will be true if we recieved something

// structure data to send or recieve
struct Data {
    int throtle;
    int roll;
    int pitch;
    int yaw;
};
Data dataToSend = { 0, 0, 0, 0 };
Data dataReceived;
//*********************** FIN NRF ********************************

// ================================================================
// ===              PID Variable declaration                   ====
// ================================================================
//////////////////////////////PID FOR ROLL///////////////////////////
float roll_PID, pwm_L_F, pwm_L_B, pwm_R_F, pwm_R_B, roll_error, roll_previous_error;
float roll_pid_p = 0;
float roll_pid_i = 0;
float roll_pid_d = 0;
///////////////////////////////ROLL PID CONSTANTS////////////////////
double roll_kp = 2; // 0.72       // 3.55
double roll_ki = 0.0; // 0.006;       // 0.003
double roll_kd = 0.0; // 1.2;         // 2.05
float roll_desired_angle = 0; // This is the angle in which we whant the

//////////////////////////////PID FOR PITCH//////////////////////////
float pitch_PID, pitch_error, pitch_previous_error;
float pitch_pid_p = 0;
float pitch_pid_i = 0;
float pitch_pid_d = 0;
///////////////////////////////PITCH PID CONSTANTS///////////////////
double pitch_kp = 2; // 0.72;        // 3.55
double pitch_ki = 0.0; // 0.006;       // 0.003
double pitch_kd = 0.0; // 1.22;        // 2.05
float pitch_desired_angle = 0; // This is the angle in which we whant the

// ================================================================
// ===               fonction declaration                      ====
// ================================================================
void init_esc_cal(void);
void init_esc(void);
void init_servos_esc(void);
void init_servos_avion(void);
void pid_simple(void);
void NRF24L01_IRQ(void);

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady() { mpuInterrupt = true; }

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup()
{
    Serial.begin(115200);
    // --------- init the ESCs, servos -------------------------------
    init_esc(); // sans calibration en premier et on met le Throtle a 1000 (zero) 4 moteurs stop
    init_servos_esc(); // les 4 servos pour les 4 moteurs
    delay(500);
    // on ajuste pour etre perpendiculaires
    servo_pwm_FL = 90;
    servo_pwm_FR = 90;
    servo_pwm_RL = 85; 
    servo_pwm_RR = 85;
    servo_FL.write(servo_pwm_FL);
    servo_RL.write(servo_pwm_RL);
    servo_RR.write(servo_pwm_RR);
    servo_FR.write(servo_pwm_FR);
    delay(500);
    init_servos_avion(); // pour les servos mode avion
    delay(500);

    // ************************ Setup  MPU6050 **************************************************
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock.
    // ----- initialize device -------------------------
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    // ------ load and configure the DMP --------------
    delay(100);
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;
        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    //********************* SET UP NRF **********************************************
    // set interrupt
    pinMode(32, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(32), NRF24L01_IRQ, FALLING);
    Serial.println("checking SPI pins....");
    
    radio.begin();
    radio.setChannel(52); // we define the optimal channel
    radio.setPALevel(RF24_PA_MIN); // set the TX power output
    radio.setDataRate(RF24_250KBPS); // set kbits/second : the lowest => max distance
    radio.maskIRQ(1, 1, 0); // enable only IRQ on RX event

    radio.openWritingPipe(address);
    radio.openReadingPipe(1, address);

    radio.startListening();
    Serial.println(radio.available());
    Serial.println(radio.getPALevel());
    Serial.println(radio.isChipConnected());
    radio.printDetails();
    delay(10);
    //********************* FIN SET UP NRF ****************
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop()
{
    //************************ NRF *************************
    if (messageAvailable) { // if we recieved something
        messageAvailable = false;
        if (radio.available()) { // we check if data recieved are available
            radio.read(&dataReceived, sizeof(Data)); // get the data 
            throtle = dataReceived.throtle;
            pitch = dataReceived.pitch;
            roll = dataReceived.roll;
            if (throtle < 1000) {
                throtle = 1000;
            }
            if (throtle > 2000) {
                throtle = 2000;
            }

            if (pitch < -200) {
                pitch = -200;
            }
            if (pitch > 200) {
                pitch = 200;
            }

            if (roll < -200) {
                roll = -200;
            }
            if (roll > 200) {
                roll = 200;
            }
        }
    }
    //************************ FIN NRF *********************

    // **************** set motors speed *****************
    // see the doc mpu_capteurs.docx page 6
    PWM_FR = throtle - roll - pitch; // motor 1
    PWM_RR = throtle - roll + pitch; // motor 2
    PWM_RL = throtle + roll + pitch; // motor 3
    PWM_FL = throtle + roll - pitch; // motor 4

    if (PWM_FR < 1000) {
        PWM_FR = 1000;
    }
    if (PWM_FR > 2000) {
        PWM_FR = 2000;
    }

    if (PWM_RR < 1000) {
        PWM_RR = 1000;
    }
    if (PWM_RR > 2000) {
        PWM_RR = 2000;
    }
    if (PWM_RL < 1000) {
        PWM_RL = 1000;
    }
    if (PWM_RL > 2000) {
        PWM_RL = 2000;
    }

    if (PWM_FL < 1000) {
        PWM_FL = 1000;
    }
    if (PWM_FL > 2000) {
        PWM_FL = 2000;
    }

    ESC_FL.writeMicroseconds(PWM_FL);
    ESC_RL.writeMicroseconds(PWM_RL);
    ESC_RR.writeMicroseconds(PWM_RR);
    ESC_FR.writeMicroseconds(PWM_FR);

    // read a packet from FIFO
    /*  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet

         // display Euler angles in degrees
         mpu.dmpGetQuaternion(&q, fifoBuffer);
         mpu.dmpGetGravity(&gravity, &q);
         mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
         ypr[0] = ypr[0] * 180.0 / M_PI;
         ypr[1] = ypr[1] * 180.0 / M_PI;
         ypr[2] = ypr[2] * 180.0 / M_PI;

          pid_simple();
         //  pwm_L_F, pwm_L_B, pwm_R_F, pwm_R_B
         if (throtle > 1050) {
             ESC_FL.writeMicroseconds((int)pwm_L_F);
             ESC_RL.writeMicroseconds((int)pwm_L_B);
             ESC_RR.writeMicroseconds((int)pwm_R_B);
             ESC_FR.writeMicroseconds((int)pwm_R_F);
         } else {
             ESC_FL.writeMicroseconds(1000);
             ESC_RL.writeMicroseconds(1000);
             ESC_RR.writeMicroseconds(1000);
             ESC_FR.writeMicroseconds(1000);
         }
     } */
    delay(20);
}

void pid_simple(void)
{
    /*///////////////////////////P I D///////////////////////////////////*/
    roll_desired_angle = 0; // map(input_ROLL,1000,2000,-10,10);
    pitch_desired_angle = 0; // map(input_PITCH,1000,2000,-10,10);

    /*First calculate the error between the desired angle and
     *the real measured angle*/
    if (abs(ypr[2]) < 0.5)
        ypr[2] = 0.0;
    if (abs(ypr[1]) < 0.5)
        ypr[1] = 0.0;
    roll_error = ypr[2] - roll_desired_angle;
    pitch_error = ypr[1] - pitch_desired_angle;
    /*Next the proportional value of the PID is just a proportional constant
     *multiplied by the error*/
    roll_pid_p = roll_kp * roll_error;
    pitch_pid_p = pitch_kp * pitch_error;
    /*The integral part should only act if we are close to the
    desired position but we want to fine tune the error. That's
    why I've made a if operation for an error between -2 and 2 degree.
    To integrate we just sum the previous integral value with the
    error multiplied by  the integral constant. This will integrate (increase)
    the value each loop till we reach the 0 point*/
    /*
    if (abs(roll_error) < 3)
    {
      roll_pid_i = roll_pid_i + (roll_ki * roll_error);
    }
    if (abs(pitch_error) < 3)
    {
      pitch_pid_i = pitch_pid_i + (pitch_ki * pitch_error);
    }
  */
    roll_pid_i = roll_pid_i + (roll_ki * roll_error);
    pitch_pid_i = pitch_pid_i + (pitch_ki * pitch_error);

    /*The last part is the derivate. The derivate acts upon the speed of the
    error. As we know the speed is the amount of error that produced in a certain
    amount of time divided by that time. For taht we will use a variable called
    previous_error. We substract that value from the actual error and divide all
    by the elapsed time. Finnaly we multiply the result by the derivate constant*/
    roll_pid_d = roll_kd * ((roll_error - roll_previous_error)); // elapsedTime);
    pitch_pid_d = pitch_kd * ((pitch_error - pitch_previous_error)); //  elapsedTime);
    /*The final PID values is the sum of each of this 3 parts*/
    roll_PID = roll_pid_p + roll_pid_i + roll_pid_d;
    pitch_PID = pitch_pid_p + pitch_pid_i + pitch_pid_d;

    /*We know taht the min value of PWM signal is 1000us and the max is 2000. So
    that tells us that the PID value can/s oscilate more than -1000 and 1000
    because when we have a value of 2000us the maximum value taht we could
    substract is 1000 and when we have a value of 1000us for the PWM signal, the
    maximum value that we could add is 1000 to reach the maximum 2000us. But we
    don't want to act over the entire range so -+400 should be enough*/

    if (roll_PID < -400) {
        roll_PID = -400;
    }
    if (roll_PID > 400) {
        roll_PID = 400;
    }
    if (pitch_PID < -400) {
        pitch_PID = -400;
    }
    if (pitch_PID > 400) {
        pitch_PID = 400;
    }

    /*Finnaly we calculate the PWM width. We sum the desired throttle and the PID
     * value*/
    pwm_R_F = throtle - roll_PID - pitch_PID;
    pwm_R_B = throtle - roll_PID + pitch_PID;
    pwm_L_B = throtle + roll_PID + pitch_PID;
    pwm_L_F = throtle + roll_PID - pitch_PID;

    /*Once again we map the PWM values to be sure that we won't pass the min
    and max values. Yes, we've already maped the PID values. But for example, for
    throttle value of 1300, if we sum the max PID value we would have 2300us and
    that will mess up the ESC.*/

    if (pwm_R_F < 1100) {
        pwm_R_F = 1100;
    }
    if (pwm_R_F > 2000) {
        pwm_R_F = 2000;
    }

    // Left front
    if (pwm_L_F < 1100) {
        pwm_L_F = 1100;
    }
    if (pwm_L_F > 2000) {
        pwm_L_F = 2000;
    }

    // Right back
    if (pwm_R_B < 1100) {
        pwm_R_B = 1100;
    }
    if (pwm_R_B > 2000) {
        pwm_R_B = 2000;
    }

    // Left back
    if (pwm_L_B < 1100) {
        pwm_L_B = 1100;
    }
    if (pwm_L_B > 2000) {
        pwm_L_B = 2000;
    }

    roll_previous_error = roll_error; // Remember to store the previous error.
    pitch_previous_error = pitch_error; // Remember to store the previous error.
}

void init_esc_cal(void)
{
    /* while (1)
    {
      throtle = analogRead(A3);
      Serial.println(throtle);
      delay(250);
    } */
    // Servo for the ESC
    ESC_FL.attach(
        PIN_FL, 1000,
        2000); // FL range 1000 usec =full stop to 2000 usec =>full speed

    ESC_RL.attach(
        PIN_RL, 1000,
        2000); // RL range 1000 usec =full stop to 2000 usec =>full speed

    ESC_RR.attach(
        PIN_RR, 1000,
        2000); // RR range 1000 usec =full stop to 2000 usec =>full speed

    ESC_FR.attach(
        PIN_FR, 1000,
        2000); // FR range 1000 usec =full stop to 2000 usec =>full speed

    PWM_FL = PWM_RL = PWM_RR = PWM_FR = 2000;
    ESC_FL.writeMicroseconds(PWM_FL);
    ESC_RL.writeMicroseconds(PWM_RL);
    ESC_RR.writeMicroseconds(PWM_RR);
    ESC_FR.writeMicroseconds(PWM_FR);

    delay(100);
    Serial.println(
        " mettre la battery et attendre les bips et ensuite appuez sur une "
        "touche");
    // int ch = Serial.read();  // get the first char.
    /*  while (Serial.available()) {
       Serial.read();
     }
     while (!Serial.available()) {
     } */
    // while (Serial.read() != -1);
    delay(5000);

    Serial.println(" mise a 0 et attente de 5 sec pour un autre bip");
    PWM_FL = PWM_RL = PWM_RR = PWM_FR = 1000;
    ESC_FL.writeMicroseconds(PWM_FL);
    ESC_RL.writeMicroseconds(PWM_RL);
    ESC_RR.writeMicroseconds(PWM_RR);
    ESC_FR.writeMicroseconds(PWM_FR);
    delay(5000);

    Serial.println("Setup et calibration finis ");
}

void init_esc(void)
{
    // Servo for the ESC
    ESC_FL.attach(PIN_FL, 1000, 2000); // FL range 1000 usec =full stop to 2000 usec =>full speed
    ESC_RL.attach(PIN_RL, 1000, 2000); // RL range 1000 usec =full stop to 2000 usec =>full speed
    ESC_RR.attach(PIN_RR, 1000, 2000); // RR range 1000 usec =full stop to 2000 usec =>full speed
    ESC_FR.attach(PIN_FR, 1000, 2000); // FR range 1000 usec =full stop to 2000 usec =>full speed

    PWM_FL = PWM_RL = PWM_RR = PWM_FR = 1000;
    ESC_FL.writeMicroseconds(PWM_FL);
    ESC_RL.writeMicroseconds(PWM_RL);
    ESC_RR.writeMicroseconds(PWM_RR);
    ESC_FR.writeMicroseconds(PWM_FR);

    delay(5000);
}

void init_servos_esc(void)
{
    servo_FL.attach(PIN_SERVO_FL);
    servo_RL.attach(PIN_SERVO_RL);
    servo_RR.attach(PIN_SERVO_RR);
    servo_FR.attach(PIN_SERVO_FR);

    servo_pwm_FL = servo_pwm_RL = servo_pwm_RR = servo_pwm_FR = 90;
    servo_FL.write(servo_pwm_FL);
    servo_RL.write(servo_pwm_RL);
    servo_RR.write(servo_pwm_RR);
    servo_FR.write(servo_pwm_FR);

    delay(5);
}

void init_servos_avion(void)
{
    servo_AL.attach(PIN_SERVO_AL);
    servo_AR.attach(PIN_SERVO_AR);
    servo_D.attach(PIN_SERVO_D);
    servo_P.attach(PIN_SERVO_P);

    pwm_AL = pwm_AR = pwm_D = pwm_P = 90;
    servo_AL.write(pwm_AL);
    servo_AR.write(pwm_AR);
    servo_D.write(pwm_D);
    servo_P.write(pwm_P);

    delay(5);
}

// *********************** pour NRF et fonction appeller uniquement quand le NRF
// a recu un truc *****************
void NRF24L01_IRQ() { messageAvailable = true; }
