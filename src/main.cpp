#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <ESP32Servo.h>
#include <math.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
#define INTERRUPT_PIN 4 

MPU6050 mpu;
Servo bldcR, bldcL, servoR, servoL;
float RollAngle;
float prev_rollError = 0;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// Interrupt detection
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}
void armMotors (){
    pinMode(LED_BUILTIN, OUTPUT);

    digitalWrite(LED_BUILTIN, HIGH);
    bldcL.writeMicroseconds(2000);
    bldcR.writeMicroseconds(2000);
    delay(3000);
    bldcL.writeMicroseconds(1000);
    bldcR.writeMicroseconds(1000);
    digitalWrite(LED_BUILTIN, LOW);
    delay(3000);
}
void throttle(int throttleValue){
    bldcL.writeMicroseconds(throttleValue);
    bldcR.writeMicroseconds(throttleValue);
}
void pitch(float pitchAngle){
    float PitchPID[3];
    float servoAngle;
    float prev_pitchError;
    float pitchError = (ypr[0]* 180/M_PI) - pitchAngle;
    servoAngle = (PitchPID[0] * pitchError) + (PitchPID[1] * (prev_pitchError + pitchError)) + (PitchPID[2] * (prev_pitchError - pitchError));
    prev_pitchError = pitchError;
    } 

    void roll(int throttleValue, float rollAngle) {
        
        float RollPID[3] = {1, 0.05, 0.1};
        // float K_scale = static_cast<float>(2000 - 1000) / 180; // Assuming linear mapping between angle and speed
        float Motorspeed;
        

        float rollError = (ypr[2] * 180 / M_PI) - rollAngle;

        Motorspeed =((RollPID[0] * rollError) + (RollPID[1] * (prev_rollError + rollError)) + (RollPID[2] * (prev_rollError - rollError)));
        Serial.println(throttleValue + (int)Motorspeed);
        Serial.println(throttleValue - (int)Motorspeed);

        // Optional clipping
        Motorspeed = fmax(1000.0f, fmin(2000.0f, Motorspeed));

        prev_rollError = rollError;
        bldcL.writeMicroseconds(throttleValue + (int)Motorspeed);
        bldcR.writeMicroseconds(throttleValue - (int)Motorspeed);
} 

void setup() {
    

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    // Initializing serial communication
    Serial.begin(115200);
    while (!Serial);
    // initializing devices
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(-260.00000);
    mpu.setYGyroOffset(-105.00000);
    mpu.setZGyroOffset(-22.00000);
    mpu.setZAccelOffset(990.00000);


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
        Serial.print(F("Enabling interrupt detection (ESP32 external interrupt "));
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
    bldcL.attach(5,1000,2000);
    bldcR.attach(4,1000,2000);

    armMotors();

    
}



void loop() {
  // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
    mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            roll(1100,(float)0.0);
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
            // delay(3000);
            
            delay(200);
        }
}

