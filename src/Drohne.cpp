/*
 Name:		Drohne.ino
 Created:	10.01.2018 16:22:22
 Author:	Micha    ...und Maxi!, halloooo?! :D
*/
#include <Servo.h>

#include <math.h>
#include <helper_3dmath.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include "TiltController.h"
#include "SPI.h"
#include "SD.h"
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;

//#define OUTPUT_READABLE_YAWPITCHROLL

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;		// return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;	// expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;		// count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
// orientation/motion vars
Quaternion q;		 // [w, x, y, z]         quaternion container
VectorFloat gravity; // [x, y, z]            gravity vector

#ifdef OUTPUT_READABLE_YAWPITCHROLL
	float ypr[3];		 // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
#endif


Servo engineServo;

Servo rollServo;
const int rollSign = -1;

Servo pitchServos[2];
const int pitchSigns[] = {1, -1};

#define IDLE_WHEN_REMOTE_CONTROLLED

#ifdef IDLE_WHEN_REMOTE_CONTROLLED
#define REMOTE_CONTROL_PIN 8
#define SD_PIN SS
#endif

#define SERVO_PIN_PITCH_1 6
#define SERVO_PIN_PITCH_2 9
#define SERVO_PIN_ROLL 5
#define SERVO_PIN_ENGINE 3

#define ROLL_DEGREE_0 81
#define ROLL_DEGREE_RANGE 40
#define MAX_ROLL_ADJUST 25

#define PITCH_DEGREE_0 96
#define PITCH_DEGREE_RANGE 30
#define MAX_PITCH_ADJUST 15

float mpuData[3];
void getMpuValues(float *data);

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
	mpuInterrupt = true;
}

TiltController *pitchController;
TiltController *rollController;

File file;

#define LOG_INTERVAL 10
#define FILE_BASE_NAME "log"

const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;
const uint8_t FILE_NAME_DIM  = BASE_NAME_SIZE + 7;
char fileName[FILE_NAME_DIM] = FILE_BASE_NAME "00.csv";

uint32_t startMillis;

void openNextLogfile() {
    Serial.println("Opening next file");
    while (SD.exists(fileName)) {
        Serial.print(fileName);
        Serial.println(" already exists");
        if (fileName[BASE_NAME_SIZE + 1] != '9') {
            fileName[BASE_NAME_SIZE + 1]++;
        } else {
            fileName[BASE_NAME_SIZE + 1] = '0';
            if (fileName[BASE_NAME_SIZE] == '9') {
                Serial.println("Can't create file name");
            }
            fileName[BASE_NAME_SIZE]++;
        }
    }
    Serial.print("Opening ");
    Serial.println(fileName);
    file = SD.open(fileName, FILE_WRITE);
    startMillis = 0;
    file.println(F("time (ms), yaw, pitch, roll, pitchPercent, rollPercent"));
}

void setup()
{
	Serial.begin(115200);
	while (!Serial); // wait for Leonardo enumeration, otherwise continue immediately

	pitchServos[0].attach(SERVO_PIN_PITCH_1);
	pitchServos[1].attach(SERVO_PIN_PITCH_2);
	rollServo.attach(SERVO_PIN_ROLL);
	engineServo.attach(SERVO_PIN_ENGINE);
    pitchController = new TiltController(2, pitchServos, pitchSigns, PITCH_DEGREE_0, PITCH_DEGREE_RANGE, MAX_PITCH_ADJUST);
	rollController = new TiltController(1, &rollServo, &rollSign, ROLL_DEGREE_0, ROLL_DEGREE_RANGE, MAX_ROLL_ADJUST);
    pitchController->targetDegree(2);
    rollController->targetDegree(-0.4);

    Serial.print("Initializing SD card...");

	if (!SD.begin(SD_PIN)) {
		Serial.println("initialization failed!");
		while (true);
	}
	Serial.println("initialization done.");

    openNextLogfile();

	// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
	Wire.begin();
	TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
	Fastwire::setup(400, true);
#endif

	// initialize device
	Serial.println(F("Initializing I2C devices..."));
	mpu.initialize();

	// verify connection
	Serial.println(F("Testing device connections..."));
	Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

	// wait for ready
	//Serial.println(F("\nSend any character to begin DMP programming and demo: "));
	//while (Serial.available() && Serial.read()); // empty buffer
	//while (!Serial.available());                 // wait for data
	//while (Serial.available() && Serial.read()); // empty buffer again

	// load and configure the DMP
	Serial.println(F("Initializing DMP..."));
	devStatus = mpu.dmpInitialize();

	// supply your own gyro offsets here, scaled for min sensitivity
	mpu.setXGyroOffset(220);
	mpu.setYGyroOffset(76);
	mpu.setZGyroOffset(-85);
	mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

	// make sure it worked (returns 0 if so)
	if (devStatus == 0)
	{
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
	}
	else
	{
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)
		Serial.print(F("DMP Initialization failed (code "));
		Serial.print(devStatus);
		Serial.println(F(")"));
	}
}

#ifdef IDLE_WHEN_REMOTE_CONTROLLED
bool isRemoteControlled() { return digitalRead(REMOTE_CONTROL_PIN) == HIGH; }
#endif

uint32_t lastMillis;

void loop()
{
#ifdef IDLE_WHEN_REMOTE_CONTROLLED
    if (isRemoteControlled()) {
        Serial.println("Stopped");
        file.close();

        pitchController->reset();
        rollController->reset();
        while (isRemoteControlled()) {}
        openNextLogfile();
        Serial.println("Continuing with next file");
    }
#endif

	engineServo.write(30); //30 für Motor = aus

	getMpuValues(mpuData);
    pitchController->supply(mpuData[1]);
    rollController->supply(mpuData[2]);

    if (millis() - lastMillis > LOG_INTERVAL - 1) {
        lastMillis = millis();
        if (startMillis == 0) {
            startMillis = lastMillis;
        }
        file.print(millis() - startMillis);
        for (float value : mpuData) {
            file.write(';');
            file.print(value * 180 / M_PI);
        }
        file.write(',');
        file.print(((float) (pitchController->getServoValue() - PITCH_DEGREE_0)) / ((float) PITCH_DEGREE_RANGE));
        file.write(',');
        file.println(((float) (rollController->getServoValue() - ROLL_DEGREE_0)) / ((float) ROLL_DEGREE_RANGE));
    }
}


void getMpuValues(float *data)
{

	// wait for MPU interrupt or extra packet(s) available
	while (!mpuInterrupt && fifoCount < packetSize)
	{
		// other program behavior stuff here
		// .
		// if you are really paranoid you can frequently test in between other
		// stuff to see if mpuInterrupt is true, and if so, "break;" from the
		// while() loop to immediately process the MPU data
	}

	// reset interrupt flag and get INT_STATUS byte
	mpuInterrupt = false;
	mpuIntStatus = mpu.getIntStatus();

	// get current FIFO count
	fifoCount = mpu.getFIFOCount();

	// check for overflow (this should never happen unless our code is too inefficient)
	if ((mpuIntStatus & 0x10) || fifoCount == 1024)
	{
		// reset so we can continue cleanly
		mpu.resetFIFO();
		Serial.println(F("FIFO overflow!"));

		// otherwise, check for DMP data ready interrupt (this should happen frequently)
	}
	else if (mpuIntStatus & 0x02)
	{
		// wait for correct available data length, should be a VERY short wait
		while (fifoCount < packetSize)
			fifoCount = mpu.getFIFOCount();

		// read a packet from FIFO
		mpu.getFIFOBytes(fifoBuffer, packetSize);
		//data[3] = millis();

		// track FIFO count here in case there is > 1 packet available
		// (this lets us immediately read more without waiting for an interrupt)
		fifoCount -= packetSize;

		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetYawPitchRoll(data, &q, &gravity);

		#ifdef OUTPUT_READABLE_YAWPITCHROLL
			// display Euler angles in degrees

			Serial.print("ypr\t");
			Serial.print(data[0] * 180 / M_PI);
			Serial.print("\t");
			Serial.print(data[1] * 180 / M_PI);
			Serial.print("\t");
			Serial.println(data[2] * 180 / M_PI);

		#endif
	}
}

