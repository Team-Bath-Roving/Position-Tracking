// Full orientation sensing using NXP/Madgwick/Mahony and a range of 9-DoF
// sensor sets.
// You *must* perform a magnetic calibration before this code will work.
//
// To view this data, use the Arduino Serial Monitor to watch the
// scrolling angles, or run the OrientationVisualiser example in Processing.
// Based on  https://github.com/PaulStoffregen/NXPMotionSense with adjustments
// to Adafruit Unified Sensor interface

/* -------------------------------- Includes -------------------------------- */

#include <Arduino.h>
// AHRS calibration stored in EEPROM
#include "EEPROM.h"
// I2C scan to check devices connected
#include "i2cscan.h"

// Flow Sensor
#include "Bitcraze_PMW3901.h"
// Adafruit libs
#define ADAFRUIT_SENSOR_CALIBRATION_USE_EEPROM
#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_AHRS.h>
// Laser altimeter
#include <SparkFun_VL6180X.h>

Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;
// IMU selection
#include "LSM6DS_LIS3MDL.h"  // can adjust to LSM6DS33, LSM6DS3U, LSM6DSOX...
//#include "LSM9DS.h"           // LSM9DS1 or LSM9DS0
//#include "NXP_FXOS_FXAS.h"  // NXP 9-DoF breakout

/* ------------------------------- AHRS config ------------------------------ */

#define IMU_I2C Wire1

// pick your filter! slower == better quality output
Adafruit_NXPSensorFusion filter; // slowest
// Adafruit_Madgwick filter;  // faster than NXP
// Adafruit_Mahony filter;  // fastest/smalleset

#if defined(ADAFRUIT_SENSOR_CALIBRATION_USE_EEPROM) // SDFat broken on rp2040
	Adafruit_Sensor_Calibration_EEPROM cal;
#else
	Adafruit_Sensor_Calibration_SDFat cal;
#endif

#define FILTER_UPDATE_RATE_HZ 100
#define PRINT_EVERY_N_UPDATES 10
//#define AHRS_DEBUG_OUTPUT

/* ------------------------- Laser Altimeter Config ------------------------- */

#define ALT_ADDR 0x29
#define ALT_I2C Wire

/* --------------------------- Optical Flow Config -------------------------- */

#define FLOW_SPI SPI
#define FLOW_CS 1

/* ------------------------- Physical configuration ------------------------- */

// displacement in mm from the centre of rotation of the vehicle to the IMU centre (x axis is left to right, y axis is back to front)
#define CENTRE_IMU_X	-20
#define CENTRE_IMU_Y	-90
// displacement in mm from the IMU centre to the optical flow sensor centre
#define IMU_FLOW_X	0
#define IMU_FLOW_Y	0
// displacement in mm from the IMU centre to the laser altimeter centre
#define IMU_ALT_X	54
#define IMU_ALT_Y	-10
// height above ground in mm of IMU, optical flow sensor and altimeter (when vehicle on flat ground)
#define IMU_Z	73
#define FLOW_Z	71
#define ALT_Z	70

#define CENTRE_FLOW_X (CENTRE_IMU_X + IMU_FLOW_X)
#define CENTRE_FLOW_Y (CENTRE_IMU_Y + IMU_FLOW_Y)
#define CENTRE_ALT_X (CENTRE_IMU_X + IMU_ALT_X)
#define CENTRE_ALT_Y (CENTRE_IMU_Y + IMU_ALT_Y)

/* --------------------------- I2C and SPI pinout --------------------------- */

#define WIRE_SDA	12
#define WIRE_SCL	13
#define WIRE_CLK	400000
#define WIRE1_SDA	6
#define WIRE1_SCL	7
#define WIRE1_CLK	400000

#define SPI_MOSI_TX	3
#define SPI_MISO_RX	4
#define SPI_SCK		2

/* ------------------ Global variables / object declaration ----------------- */



VL6180x altimeter(ALT_ADDR);
Bitcraze_PMW3901 flow(FLOW_CS); // pin 1 for CS

float R, P, H; // roll pitch and heading
float dR,dP,dH; // change in roll,pitch,heading
int16_t dX,dY;
int32_t x,y,xpos,ypos,xpos2,ypos2;

uint32_t timestamp;

void setup() {
	
/* --------------------------------- Init IO -------------------------------- */

	Serial.begin(115200);

	Wire.setSDA(WIRE_SDA);
	Wire.setSCL(WIRE_SCL);
	Wire.setClock(WIRE_CLK);
	Wire.begin();

	Wire1.setSDA(WIRE1_SDA);
	Wire1.setSCL(WIRE1_SCL);
	Wire1.setClock(WIRE1_CLK);
	Wire1.begin();
	
	i2cscan(&Wire);
	i2cscan(&Wire1);

	SPI.setSCK(SPI_SCK);
	SPI.setTX(SPI_MOSI_TX);
	SPI.setRX(SPI_MISO_RX);

  	// while (!Serial) yield(); // wait until serial monitor starts
	delay(2000);
	Serial.println("Hi");

/* ---------------------------- Load Calibration ---------------------------- */

	EEPROM.begin(4096); // open EEPROM to fetch calibration data
	if (!cal.begin()) {
		Serial.println("Failed to initialize calibration helper");
	} else if (! cal.loadCalibration()) {
		Serial.println("No calibration loaded/found");
	} 
  	EEPROM.end(); // close EEPROM

/* ------------------------------ Start Sensors ----------------------------- */

	if (!init_sensors(&Wire1)) {
		Serial.println("Failed to find sensors");
		while (1) delay(10);
	}
	if (!flow.begin()) {
		Serial.println("Initialization of the flow sensor failed");
	}
	if (!altimeter.VL6180xInit()) {
		Serial.println("Initialization of the distance sensor failed");
	}

/* ------------------------------- AHRS setup ------------------------------- */

	accelerometer->printSensorDetails();
	gyroscope->printSensorDetails();
	magnetometer->printSensorDetails();

	setup_sensors();
	filter.begin(FILTER_UPDATE_RATE_HZ);
	timestamp = millis();

/* -------------------------- Laser Altimeter Setup ------------------------- */

	altimeter.VL6180xDefautSettings();

/* --------------------------- Optical Flow Setup --------------------------- */

	flow.setLed(true);


	// Serial.print(">3D|mySimpleCube:S:cube\n");
}

void loop() {

/* -------------------------------- Read AHRS ------------------------------- */

	
	float gx, gy, gz;
	static uint8_t counter = 0;

	if ((millis() - timestamp) < (1000 / FILTER_UPDATE_RATE_HZ)) {
		return;
	}
	timestamp = millis();
	// Read the motion sensors
	sensors_event_t accel, gyro, mag;
	accelerometer->getEvent(&accel);
	gyroscope->getEvent(&gyro);
	magnetometer->getEvent(&mag);
	#if defined(AHRS_DEBUG_OUTPUT)
		Serial.print("I2C took "); Serial.print(millis()-timestamp); Serial.println(" ms");
	#endif

	cal.calibrate(mag);
	cal.calibrate(accel);
	cal.calibrate(gyro);
	// Gyroscope needs to be converted from Rad/s to Degree/s
	// the rest are not unit-important
	gx = gyro.gyro.x * SENSORS_RADS_TO_DPS;
	gy = gyro.gyro.y * SENSORS_RADS_TO_DPS;
	gz = gyro.gyro.z * SENSORS_RADS_TO_DPS;

	// Update the SensorFusion filter
	filter.update(gx, gy, gz, 
                accel.acceleration.x, accel.acceleration.y, accel.acceleration.z, 
                mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);
	#if defined(AHRS_DEBUG_OUTPUT)
		Serial.print("Update took "); Serial.print(millis()-timestamp); Serial.println(" ms");
	#endif


	#if defined(AHRS_DEBUG_OUTPUT)
		Serial.print("Raw: ");
		Serial.print(accel.acceleration.x, 4); Serial.print(", ");
		Serial.print(accel.acceleration.y, 4); Serial.print(", ");
		Serial.print(accel.acceleration.z, 4); Serial.print(", ");
		Serial.print(gx, 4); Serial.print(", ");
		Serial.print(gy, 4); Serial.print(", ");
		Serial.print(gz, 4); Serial.print(", ");
		Serial.print(mag.magnetic.x, 4); Serial.print(", ");
		Serial.print(mag.magnetic.y, 4); Serial.print(", ");
		Serial.print(mag.magnetic.z, 4); Serial.println("");
	#endif

	
	dR = filter.getRoll	() 	- R;
	dP = filter.getPitch() 	- P;
	dH = filter.getYaw	() 	- H;
	R  = filter.getRoll	();
	P  = filter.getPitch();
	H  = filter.getYaw	();
	

	float qw, qx, qy, qz;
	filter.getQuaternion(&qw, &qx, &qy, &qz);

/* -------------------------- Read Laser Altimeter -------------------------- */

	int16_t altitude = altimeter.getDistance()-ALT_Z;


/* ---------------------------- Read Optical Flow --------------------------- */

	int16_t dXneg;
	flow.enableFrameBuffer();
	flow.readMotionCount(&dY, &dX);

	/* ------------------------------ Combine data ------------------------------ */

	x+=dX;
	y+=dY;

	xpos+=dX*sin(radians(H));
	ypos+=dY*cos(radians(H));


	/* ------------------------------ Print Results ----------------------------- */

	// only print the calculated output once in a while
	if (counter++ <= PRINT_EVERY_N_UPDATES) {
		return;
	}
	counter=0;

	Serial.print(">dxdy:");Serial.print(dX); Serial.print(":");Serial.print(dX);Serial.println("|xy");
	Serial.print(">XY:");Serial.print(x); Serial.print(":");Serial.print(y);Serial.println("|xy");
	Serial.print(">XposYpos:");Serial.print(xpos); Serial.print(":");Serial.print(ypos);Serial.println("|xy");


	// Serial.printf(">3D|mySimpleCube:S:cube:R:%f:%f:%f:P:%f:%f:%f\n",radians(P),-radians(H),radians(R), xpos2/10.0,altitude/10.0,ypos2/10.0);

	// print the heading, pitch and roll
	Serial.print(">heading:"); 
	Serial.println(H);
	Serial.print(">pitch:");
	Serial.println(P);
	Serial.print(">roll:");
	Serial.println(R);
}