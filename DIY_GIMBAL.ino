/*
 * DIY-GIMBAL
 * Use : 
 * 1. 3-Servo motor for each axis
 * 2. Gyro sensor : MPU6050 9-degree of freedom
 * 3. Arduino UNO
 * 4. All part injection from 3D printer
 * Coding by : Isaranu janthong from Founder of http://www.iottweet.com
 * Code updated : 2016.Oct.2nd
 * This is maker show up from THAILAND.
 * 
*/

#include "I2Cdev.h"
#include <Servo.h>
#include <Wire.h>

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

Servo SvRoll, SvPitch, SvYaw, Stest;

#define OUTPUT_READABLE_YAWPITCHROLL

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
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

int SdriveRoll, SdrivePitch, SdriveYaw;
int RollOffset = 0, PitchOffset = -2, YawOffset = 0;
int YawInitial = 0, YawSetInit = 0;

// For interrupt detection on port D2
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() 
{
    mpuInterrupt = true;
}

void setup() 
{
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz). Comment this line if having compilation difficulties with TWBR.
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);

    // initialize device
    Serial.println(F("Initializing Gyro sensor devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Gyro device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
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

    //Configuration servo drive port.
    SvRoll.attach(3);
    SvPitch.attach(4);
    SvYaw.attach(5);

    //Start-up initial position setting.
    SvRoll.write(90);
    SvPitch.write(90);
    SvYaw.write(90+YawOffset);

    //Case of Yaw controlling, wait for z euler angle stable
    //Serial.print("\n Waiting for Z euler angle stable..");
    //delay(3500);
}

void loop() 
{
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) 
    {
      //Do something    
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            
            while(YawSetInit = 0)
            {
                YawInitial = int(ypr[0] * 180/M_PI);
                YawSetInit = 1; // Already set.
            }

            //Feedback angle, final drive Servo
            SdriveRoll = int(ypr[2] * 180/M_PI);
            SdrivePitch = int(ypr[1] * 180/M_PI);
            SdriveYaw = (int(ypr[0] * 180/M_PI)) - YawInitial;
            
            SvRoll.write(1.00*(90-SdriveRoll+RollOffset));
            SvPitch.write(1.00*(90-SdrivePitch+PitchOffset));
            //SvYaw.write(90-SdriveYaw);  >> Pending YAW-Servo drive (still have bug)

            Serial.println(SdriveRoll);
            Serial.print("\t");
            Serial.print(SdrivePitch);
            Serial.print("\t");
            Serial.print(SdriveYaw);
            Serial.print("\n");
  
        #endif
    }
}
