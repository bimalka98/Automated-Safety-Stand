/* Copyright notice for MPU6050 lib by Jeff Rowberg
  ============================================
  I2Cdev device library code is placed under the MIT license
  Copyright (c) 2012 Jeff Rowberg

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
  ===============================================
*/

// including I2Cdev and MPU6050 libraries
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h" //"MPU6050.h" is included in this header file
#include <Stepper.h>

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define stepA A0
#define stepB A1
#define stepC A2
#define stepD A3

#define powerSwitch 0
#define irInput 2
#define redLED 5
#define greenLED 6
#define buzzer 7


const int stepsPerRevolution = 200;  // change this to fit the number of steps per revolution
// for your motor


// initialize the stepper library on pins 8 through 11:
Stepper myStepper(stepsPerRevolution, stepA, stepB, stepC,stepD);

bool stepperDownFlag = false;

int stepCount = 0;  // number of steps the motor has taken
/* class default I2C address is 0x68
  specific I2C addresses may be passed as a parameter here
  AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
  AD0 high = 0x69 */
MPU6050 mpu; // creating an object of the class MPU6050
// MPU6050 mpu(0x69); // <-- use for AD0 high

/* define "OUTPUT_READABLE_YAWPITCHROLL" to see the yaw/
  pitch/roll angles (in degrees) calculated from the quaternions coming
  from the FIFO. Note this also requires gravity vector calculations.
  Also note that yaw/pitch/roll angles suffer from gimbal lock (for
  more info, see: http://en.wikipedia.org/wiki/Gimbal_lock) */
#define OUTPUT_READABLE_YAWPITCHROLL

// *****************************************************************************
/* *********************   VARIABLE DECLARATION    *********************      */
// *****************************************************************************

/* MPU control/status vars */
bool dmpReady = false;        // set true if DMP init was successful
uint8_t mpuIntStatus;         // holds actual interrupt status byte from MPU
uint8_t devStatus;            // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;          // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;           // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];       // FIFO storage buffer
uint8_t calibrationLoops = 6; // calibration loops

/* orientation/motion vars */
Quaternion q;                 // [w, x, y, z]         quaternion container
VectorInt16 aa;               // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;           // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;          // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;          // [x, y, z]            gravity vector
float euler[3];               // [psi, theta, phi]    Euler angle container
float ypr[3];                 // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

/* program specific variables */
float setAngleOfBike = 45; // set angle of bike: when it is in rest position
int buttonState = 0;         // current state of the button
int lastButtonState = 0;     // previous state of the button

// *****************************************************************************
/* **************      USER DEFINED FUNCTIONS DECLARATION **************      */
// *****************************************************************************

void initializeMPU6050();       // fucntion to initialize the MPU6050
float getCurrentangleOfBike();  // function to get the current angle of the bike

// *****************************************************************************
/* **************     SET UP FUCNTION RUNS ONCE AT STARTUP **************      */
// *****************************************************************************

void setup() { // setup function runs once at startupS

  // set the speed at 60 rpm:
  myStepper.setSpeed(60);

  pinMode (irInput, INPUT); // IR sensor pin INPUT
  pinMode (powerSwitch, INPUT); //  bike power pin INPUT
  pinMode (redLED , OUTPUT);
  pinMode (greenLED , OUTPUT);
  pinMode (buzzer , OUTPUT);

  

  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialization of MPU6050 IMU
  initializeMPU6050();

}

// *****************************************************************************
/* *****************     LOOP FUCNTION RUNS REPEATEDLY *****************      */
// *****************************************************************************

void loop() { // loop function runs repeatedly

  // get current angle of the bike
  float _currentangle = getCurrentangleOfBike();


  if ( _currentangle < setAngleOfBike && digitalRead(powerSwitch) == HIGH){
    // should check whether an IR sensor is actually needed.
    while (digitalRead(irInput) != HIGH){
      turnClockwise();
    }
  }

  buttonState = digitalRead(powerSwitch);

  if (buttonState != lastButtonState) {
    if (buttonState == LOW && lastButtonState == HIGH) {
      turnAntiClockwise();
    }
    lastButtonState = buttonState;
  }

  antiTheft();
}

// *****************************************************************************
/* **************      USER DEFINED FUNCTIONS DEFINITIONS **************      */
// *****************************************************************************
void turnClockwise(){
  myStepper.step(1);
  digitalWrite(redLED , HIGH);
  delay(50);
  digitalWrite(redLED , LOW);
  delay(50);
  stepCount++;
}

void turnAntiClockwise(){
  while(stepCount >0){
    myStepper.step(-1);
    digitalWrite(redLED , HIGH);
    delay(50);
    digitalWrite(redLED , LOW);
    delay(50);
    stepCount--;
  }
}

void antiTheft(){
  if (digitalRead(irInput) == HIGH && digitalRead(powerSwitch) == LOW){
    digitalWrite(buzzer , HIGH);
  }
}

// fucntion to initialize the MPU6050
void initializeMPU6050() {

  // initialize device
  mpu.initialize();
  //pinMode(INTERRUPT_PIN, INPUT);

  // load and configure the DMP : internal digital motion processor
  devStatus = mpu.dmpInitialize();

  // set accelerometer and gyroscope offsets to zero
  mpu.setXAccelOffset(0); mpu.setYAccelOffset(0); mpu.setZAccelOffset(0);
  mpu.setXGyroOffset(0); mpu.setYGyroOffset(0); mpu.setZGyroOffset(0);


  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(calibrationLoops);
    mpu.CalibrateGyro(calibrationLoops);

    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    //attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    //mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();

  }

}

// function to get the current angle of the bike
float getCurrentangleOfBike() {

  // doble check if MPU6050 is ready
  if (!dmpReady) {
    initializeMPU6050(); // try to initialize again
  }

  // clear the buffer to take the new data: https://mjwhite8119.github.io/Robots/mpu6050
  mpu.resetFIFO();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // wait for correct available data length, should be a VERY short wait
  while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

  // declaring variables to hold yaw, pitch, roll
  float _yaw, _pitch, _roll;

  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {

    // calculate Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    // calcuate ya/pitch/roll angles
    _yaw = ypr[0] * 180 / M_PI; // Yaw angle: rotation around the z-axis
    _pitch = ypr[1] * 180 / M_PI; // Pitch angle: rotation around the y-axis
    _roll = ypr[2] * 180 / M_PI; // Roll angle: rotation around the x-axis
  }

  // return the angle: rotation around y-axis
  return _pitch;
}
