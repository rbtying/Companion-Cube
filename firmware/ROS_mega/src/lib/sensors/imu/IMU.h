/*
 * IMU.h
 *
 *  Created on: Feb 2, 2012
 *      Author: rbtying
 */

#ifndef IMU_H_
#define IMU_H_

#include <libraries/Wire/Wire.h>
#include "L3G4200D.h"
#include "LSM303.h"

namespace IMU {

int SENSOR_SIGN[9] = { -1, 1, 1, 1, -1, -1, -1, 1, 1 };
// LSM303 accelerometer: 8 g sensitivity
// 3.8 mg/digit; 1 g = 256
#define GRAVITY 256  //this equivalent to 1G in the raw data coming from the accelerometer
#define ToRad(x) ((x)*0.01745329252)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi
// L3G4200D gyro: 2000 dps full scale
// 70 mdps/digit; 1 dps = 0.07
#define Gyro_Gain_X 0.07 //X axis Gyro gain
#define Gyro_Gain_Y 0.07 //Y axis Gyro gain
#define Gyro_Gain_Z 0.07 //Z axis Gyro gain
#define Gyro_Scaled_X(x) ((x)*ToRad(Gyro_Gain_X)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Y(x) ((x)*ToRad(Gyro_Gain_Y)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Z(x) ((x)*ToRad(Gyro_Gain_Z)) //Return the scaled ADC raw data of the gyro in radians for second
// LSM303 magnetometer calibration constants; use the Calibrate example from
// the Pololu LSM303 library to find the right values for your board
#define M_X_MIN -796
#define M_Y_MIN -457
#define M_Z_MIN -424
#define M_X_MAX 197
#define M_Y_MAX 535
#define M_Z_MAX 397

#define Kp_ROLLPITCH 0.02
#define Ki_ROLLPITCH 0.00002
#define Kp_YAW 1.2
#define Ki_YAW 0.00002

/*For debugging purposes*/
//OUTPUTMODE=1 will print the corrected data,
//OUTPUTMODE=0 will print uncorrected data of the gyros (with drift)
#define OUTPUTMODE 1

//#define PRINT_DCM 0     //Will print the whole direction cosine matrix
#define PRINT_ANALOGS 0 //Will print the analog raw data
#define PRINT_EULER 1   //Will print the Euler angles Roll, Pitch and Yaw
#define STATUS_LED 13

float G_Dt = 0.02; // Integration time (DCM algorithm)  We will run the integration loop at 50Hz if possible

long timer = 0; //general purpuse timer
long timer_old;
long timer24 = 0; //Second timer used to print values
int AN[6]; //array that stores the gyro and accelerometer data
int AN_OFFSET[6] = { 0, 0, 0, 0, 0, 0 }; //Array that stores the Offset of the sensors

int gyro_x;
int gyro_y;
int gyro_z;
int accel_x;
int accel_y;
int accel_z;
int magnetom_x;
int magnetom_y;
int magnetom_z;
float c_magnetom_x;
float c_magnetom_y;
float c_magnetom_z;
float MAG_Heading;

float Accel_Vector[3] = { 0, 0, 0 }; //Store the acceleration in a vector
float Gyro_Vector[3] = { 0, 0, 0 };//Store the gyros turn rate in a vector
float Omega_Vector[3] = { 0, 0, 0 }; //Corrected Gyro_Vector data
float Omega_P[3] = { 0, 0, 0 };//Omega Proportional correction
float Omega_I[3] = { 0, 0, 0 };//Omega Integrator
float Omega[3] = { 0, 0, 0 };

// Euler angles
float roll;
float pitch;
float yaw;

float errorRollPitch[3] = { 0, 0, 0 };
float errorYaw[3] = { 0, 0, 0 };

unsigned int counter = 0;
byte gyro_sat = 0;

float DCM_Matrix[3][3] = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
float Update_Matrix[3][3] = { { 0, 1, 2 }, { 3, 4, 5 }, { 6, 7, 8 } }; //Gyros here


float Temporary_Matrix[3][3] = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };

// function prototypes

void initI2C();
void initAccel();
void initCompass();
void initGyro();
void readGyro();
void readAccel();
void readCompass();
void Compass_Heading();
void Normalize();
void Drift_correction();
void Euler_angles();
void printdata();
void Matrix_update();
void matrixMultiply(float a[3][3], float b[3][3], float mat[3][3]);
void vectorAdd(float vectorOut[3], float vectorIn1[3], float vectorIn2[3]);
void vectorCrossProduct(float vectorOut[3], float v1[3], float v2[3]);
float vectorDotProduct(float vector1[3], float vector2[3]);
void vectorScale(float vectorOut[3], float vectorIn[3], float scale2);

void initIMU() {
	Serial3.begin(115200);
	pinMode(STATUS_LED, OUTPUT); // Status LED

	initI2C();

	Serial3.println("Pololu MinIMU-9 + Arduino AHRS");

	digitalWrite(STATUS_LED, LOW);
	delay(1500);

	initAccel();
	initCompass();
	initGyro();

	delay(20);

	for (int i = 0; i < 32; i++) // We take some readings...
	{
		readGyro();
		readAccel();
		for (int y = 0; y < 6; y++) // Cumulate values
			AN_OFFSET[y] += AN[y];
		delay(20);
	}

	for (int y = 0; y < 6; y++)
		AN_OFFSET[y] = AN_OFFSET[y] / 32;

	AN_OFFSET[5] -= GRAVITY * SENSOR_SIGN[5];

	//Serial3.println("Offset:");
	for (int y = 0; y < 6; y++)
		Serial3.println(AN_OFFSET[y]);

	delay(2000);
	digitalWrite(STATUS_LED, HIGH);

	timer = millis();
	delay(20);
	counter = 0;
}

bool updateIMU() //Main Loop
{
	if ((millis() - timer) >= 20) // Main loop runs at 50Hz
	{
		counter++;
		timer_old = timer;
		timer = millis();
		if (timer > timer_old)
			G_Dt = (timer - timer_old) / 1000.0; // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
		else
			G_Dt = 0;

		// *** DCM algorithm
		// Data adquisition
		readGyro(); // This read gyro data
		readAccel(); // Read I2C accelerometer

		if (counter > 5) // Read compass data at 10Hz... (5 loop runs)
		{
			counter = 0;
			readCompass(); // Read I2C magnetometer
			Compass_Heading(); // Calculate magnetic heading
		}

		// Calculations...
		Matrix_update();
		Normalize();
		Drift_correction();
		Euler_angles();
		// ***
		return true;
	}
	return false;
}

void Compass_Heading() {
	float MAG_X;
	float MAG_Y;
	float cos_roll;
	float sin_roll;
	float cos_pitch;
	float sin_pitch;

	cos_roll = cos(roll);
	sin_roll = sin(roll);
	cos_pitch = cos(pitch);
	sin_pitch = sin(pitch);

	// adjust for LSM303 compass axis offsets/sensitivity differences by scaling to +/-0.5 range
	c_magnetom_x = (float) (magnetom_x - SENSOR_SIGN[6] * M_X_MIN) / (M_X_MAX
			- M_X_MIN) - SENSOR_SIGN[6] * 0.5;
	c_magnetom_y = (float) (magnetom_y - SENSOR_SIGN[7] * M_Y_MIN) / (M_Y_MAX
			- M_Y_MIN) - SENSOR_SIGN[7] * 0.5;
	c_magnetom_z = (float) (magnetom_z - SENSOR_SIGN[8] * M_Z_MIN) / (M_Z_MAX
			- M_Z_MIN) - SENSOR_SIGN[8] * 0.5;

	// Tilt compensated Magnetic field X:
	MAG_X = c_magnetom_x * cos_pitch + c_magnetom_y * sin_roll * sin_pitch
			+ c_magnetom_z * cos_roll * sin_pitch;
	// Tilt compensated Magnetic field Y:
	MAG_Y = c_magnetom_y * cos_roll - c_magnetom_z * sin_roll;
	// Magnetic Heading
	MAG_Heading = atan2(-MAG_Y, MAG_X);
}

void Normalize(void) {
	float error = 0;
	float temporary[3][3];
	float renorm = 0;

	error = -vectorDotProduct(&DCM_Matrix[0][0], &DCM_Matrix[1][0]) * .5; //eq.19

	vectorScale(&temporary[0][0], &DCM_Matrix[1][0], error); //eq.19
	vectorScale(&temporary[1][0], &DCM_Matrix[0][0], error); //eq.19

	vectorAdd(&temporary[0][0], &temporary[0][0], &DCM_Matrix[0][0]);//eq.19
	vectorAdd(&temporary[1][0], &temporary[1][0], &DCM_Matrix[1][0]);//eq.19

	vectorCrossProduct(&temporary[2][0], &temporary[0][0], &temporary[1][0]); // c= a x b //eq.20

	renorm = .5 * (3 - vectorDotProduct(&temporary[0][0], &temporary[0][0])); //eq.21
	vectorScale(&DCM_Matrix[0][0], &temporary[0][0], renorm);

	renorm = .5 * (3 - vectorDotProduct(&temporary[1][0], &temporary[1][0])); //eq.21
	vectorScale(&DCM_Matrix[1][0], &temporary[1][0], renorm);

	renorm = .5 * (3 - vectorDotProduct(&temporary[2][0], &temporary[2][0])); //eq.21
	vectorScale(&DCM_Matrix[2][0], &temporary[2][0], renorm);
}

/**************************************************/
void Drift_correction(void) {
	float mag_heading_x;
	float mag_heading_y;
	float errorCourse;
	//Compensation the Roll, Pitch and Yaw drift.
	static float Scaled_Omega_P[3];
	static float Scaled_Omega_I[3];
	float Accel_magnitude;
	float Accel_weight;

	//*****Roll and Pitch***************

	// Calculate the magnitude of the accelerometer vector
	Accel_magnitude = sqrt(
			Accel_Vector[0] * Accel_Vector[0] + Accel_Vector[1]
					* Accel_Vector[1] + Accel_Vector[2] * Accel_Vector[2]);
	Accel_magnitude = Accel_magnitude / GRAVITY; // Scale to gravity.
	// Dynamic weighting of accelerometer info (reliability filter)
	// Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
	Accel_weight = constrain(1 - 2 * abs(1 - Accel_magnitude), 0, 1); //

	vectorCrossProduct(&errorRollPitch[0], &Accel_Vector[0], &DCM_Matrix[2][0]); //adjust the ground of reference
	vectorScale(&Omega_P[0], &errorRollPitch[0], Kp_ROLLPITCH * Accel_weight);

	vectorScale(&Scaled_Omega_I[0], &errorRollPitch[0],
			Ki_ROLLPITCH * Accel_weight);
	vectorAdd(Omega_I, Omega_I, Scaled_Omega_I);

	//*****YAW***************
	// We make the gyro YAW drift correction based on compass magnetic heading

	mag_heading_x = cos(MAG_Heading);
	mag_heading_y = sin(MAG_Heading);
	errorCourse = (DCM_Matrix[0][0] * mag_heading_y) - (DCM_Matrix[1][0]
			* mag_heading_x); //Calculating YAW error
	vectorScale(errorYaw, &DCM_Matrix[2][0], errorCourse); //Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.

	vectorScale(&Scaled_Omega_P[0], &errorYaw[0], Kp_YAW);//.01proportional of YAW.
	vectorAdd(Omega_P, Omega_P, Scaled_Omega_P);//Adding  Proportional.

	vectorScale(&Scaled_Omega_I[0], &errorYaw[0], Ki_YAW);//.00001Integrator
	vectorAdd(Omega_I, Omega_I, Scaled_Omega_I);//adding integrator to the Omega_I
}
/**************************************************/
/*
 void Accel_adjust(void)
 {
 Accel_Vector[1] += Accel_Scale(speed_3d*Omega[2]);  // Centrifugal force on Acc_y = GPS_speed*GyroZ
 Accel_Vector[2] -= Accel_Scale(speed_3d*Omega[1]);  // Centrifugal force on Acc_z = GPS_speed*GyroY
 }
 */
/**************************************************/

void Matrix_update(void) {
	Gyro_Vector[0] = Gyro_Scaled_X(gyro_x); //gyro x roll
	Gyro_Vector[1] = Gyro_Scaled_Y(gyro_y); //gyro y pitch
	Gyro_Vector[2] = Gyro_Scaled_Z(gyro_z); //gyro Z yaw

	Accel_Vector[0] = accel_x;
	Accel_Vector[1] = accel_y;
	Accel_Vector[2] = accel_z;

	vectorAdd(&Omega[0], &Gyro_Vector[0], &Omega_I[0]); //adding proportional term
	vectorAdd(&Omega_Vector[0], &Omega[0], &Omega_P[0]); //adding Integrator term

	//Accel_adjust();    //Remove centrifugal acceleration.   We are not using this function in this version - we have no speed measureme


#if OUTPUTMODE==1
	Update_Matrix[0][0] = 0;
	Update_Matrix[0][1] = -G_Dt * Omega_Vector[2];//-z
	Update_Matrix[0][2] = G_Dt * Omega_Vector[1];//y
	Update_Matrix[1][0] = G_Dt * Omega_Vector[2];//z
	Update_Matrix[1][1] = 0;
	Update_Matrix[1][2] = -G_Dt * Omega_Vector[0];//-x
	Update_Matrix[2][0] = -G_Dt * Omega_Vector[1];//-y
	Update_Matrix[2][1] = G_Dt * Omega_Vector[0];//x
	Update_Matrix[2][2] = 0;
#else                    // Uncorrected data (no drift correction)
	Update_Matrix[0][0]=0;
	Update_Matrix[0][1]=-G_Dt*Gyro_Vector[2];//-z
	Update_Matrix[0][2]=G_Dt*Gyro_Vector[1];//y
	Update_Matrix[1][0]=G_Dt*Gyro_Vector[2];//z
	Update_Matrix[1][1]=0;
	Update_Matrix[1][2]=-G_Dt*Gyro_Vector[0];
	Update_Matrix[2][0]=-G_Dt*Gyro_Vector[1];
	Update_Matrix[2][1]=G_Dt*Gyro_Vector[0];
	Update_Matrix[2][2]=0;
#endif

	matrixMultiply(DCM_Matrix, Update_Matrix, Temporary_Matrix); //a*b=c

	for (int x = 0; x < 3; x++) //Matrix Addition (update)
	{
		for (int y = 0; y < 3; y++) {
			DCM_Matrix[x][y] += Temporary_Matrix[x][y];
		}
	}
}

void Euler_angles(void) {
	pitch = -asin(DCM_Matrix[2][0]);
	roll = atan2(DCM_Matrix[2][1], DCM_Matrix[2][2]);
	yaw = atan2(DCM_Matrix[1][0], DCM_Matrix[0][0]);
}

L3G4200D gyro;
LSM303 compass;

void initI2C() {
	Wire.begin();
}

void initGyro() {
	gyro.writeReg(L3G4200D_CTRL_REG1, 0x0F); // normal power mode, all axes enabled, 100 Hz
	gyro.writeReg(L3G4200D_CTRL_REG4, 0x20); // 2000 dps full scale
}

void readGyro() {
	gyro.read();

	AN[0] = gyro.g.x;
	AN[1] = gyro.g.y;
	AN[2] = gyro.g.z;
	gyro_x = SENSOR_SIGN[0] * (AN[0] - AN_OFFSET[0]);
	gyro_y = SENSOR_SIGN[1] * (AN[1] - AN_OFFSET[1]);
	gyro_z = SENSOR_SIGN[2] * (AN[2] - AN_OFFSET[2]);
}

void initAccel() {
	compass.writeAccReg(LSM303_CTRL_REG1_A, 0x27); // normal power mode, all axes enabled, 50 Hz
	compass.writeAccReg(LSM303_CTRL_REG4_A, 0x30); // 8 g full scale
}

// Reads x,y and z accelerometer registers
void readAccel() {
	compass.readAcc();

	AN[3] = compass.a.x;
	AN[4] = compass.a.y;
	AN[5] = compass.a.z;
	accel_x = SENSOR_SIGN[3] * (AN[3] - AN_OFFSET[3]);
	accel_y = SENSOR_SIGN[4] * (AN[4] - AN_OFFSET[4]);
	accel_z = SENSOR_SIGN[5] * (AN[5] - AN_OFFSET[5]);
}

void initCompass() {
	compass.init();
	compass.writeMagReg(LSM303_MR_REG_M, 0x00); // continuous conversion mode
	// 15 Hz default
}

void readCompass() {
	compass.readMag();

	magnetom_x = SENSOR_SIGN[6] * compass.m.x;
	magnetom_y = SENSOR_SIGN[7] * compass.m.y;
	magnetom_z = SENSOR_SIGN[8] * compass.m.z;
}

void printdata(void) {
	Serial3.print("!");

#if PRINT_EULER == 1
	Serial3.print("ANG:");
	Serial3.print(ToDeg(roll));
	Serial3.print(",");
	Serial3.print(ToDeg(pitch));
	Serial3.print(",");
	Serial3.print(ToDeg(yaw));
#endif
#if PRINT_ANALOGS==1
	Serial3.print(",AN:");
	Serial3.print(AN[0]); //(int)read_adc(0)
	Serial3.print(",");
	Serial3.print(AN[1]);
	Serial3.print(",");
	Serial3.print(AN[2]);
	Serial3.print(",");
	Serial3.print(AN[3]);
	Serial3.print (",");
	Serial3.print(AN[4]);
	Serial3.print (",");
	Serial3.print(AN[5]);
	Serial3.print(",");
	Serial3.print(c_magnetom_x);
	Serial3.print (",");
	Serial3.print(c_magnetom_y);
	Serial3.print (",");
	Serial3.print(c_magnetom_z);
#endif
	/*#if PRINT_DCM == 1
	 Serial3.print (",DCM:");
	 Serial3.print(convert_to_dec(DCM_Matrix[0][0]));
	 Serial3.print (",");
	 Serial3.print(convert_to_dec(DCM_Matrix[0][1]));
	 Serial3.print (",");
	 Serial3.print(convert_to_dec(DCM_Matrix[0][2]));
	 Serial3.print (",");
	 Serial3.print(convert_to_dec(DCM_Matrix[1][0]));
	 Serial3.print (",");
	 Serial3.print(convert_to_dec(DCM_Matrix[1][1]));
	 Serial3.print (",");
	 Serial3.print(convert_to_dec(DCM_Matrix[1][2]));
	 Serial3.print (",");
	 Serial3.print(convert_to_dec(DCM_Matrix[2][0]));
	 Serial3.print (",");
	 Serial3.print(convert_to_dec(DCM_Matrix[2][1]));
	 Serial3.print (",");
	 Serial3.print(convert_to_dec(DCM_Matrix[2][2]));
	 #endif*/
	Serial3.println();

}

long convertToDec(float x) {
	return x * 10000000;
}

//Computes the dot product of two vectors
float vectorDotProduct(float vector1[3], float vector2[3]) {
	float op = 0;

	for (int c = 0; c < 3; c++) {
		op += vector1[c] * vector2[c];
	}

	return op;
}

//Computes the cross product of two vectors
void vectorCrossProduct(float vectorOut[3], float v1[3], float v2[3]) {
	vectorOut[0] = (v1[1] * v2[2]) - (v1[2] * v2[1]);
	vectorOut[1] = (v1[2] * v2[0]) - (v1[0] * v2[2]);
	vectorOut[2] = (v1[0] * v2[1]) - (v1[1] * v2[0]);
}

//Multiply the vector by a scalar.
void vectorScale(float vectorOut[3], float vectorIn[3], float scale2) {
	for (int c = 0; c < 3; c++) {
		vectorOut[c] = vectorIn[c] * scale2;
	}
}

void vectorAdd(float vectorOut[3], float vectorIn1[3], float vectorIn2[3]) {
	for (int c = 0; c < 3; c++) {
		vectorOut[c] = vectorIn1[c] + vectorIn2[c];
	}
}

//Multiply two 3x3 matrixs. This function developed by Jordi can be easily adapted to multiple n*n matrix's. (Pero me da flojera!).
void matrixMultiply(float a[3][3], float b[3][3], float mat[3][3]) {
	float op[3];
	for (int x = 0; x < 3; x++) {
		for (int y = 0; y < 3; y++) {
			for (int w = 0; w < 3; w++) {
				op[w] = a[x][w] * b[w][y];
			}
			mat[x][y] = 0;
			mat[x][y] = op[0] + op[1] + op[2];

			//			float test = mat[x][y];
		}
	}
}
}

#endif /* IMU_H_ */
