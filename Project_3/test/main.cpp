#include<iostream>
#include<stdio.h>
#include<fcntl.h>
#include<sys/ioctl.h>
#include<linux/i2c.h>
#include<linux/i2c-dev.h>
#include<iomanip>
#include<unistd.h>


#ifdef __cplusplus
extern "C"
{
#endif

#include <math.h>

#define sampleFreq	512.0f		// sample frequency in H
float beta = 0.1f;								// 2 * proportional gain (Kp)
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	// quaternion of sensor frame relative to auxiliary frame

float invSqrt(float x);

void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;   

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * (1.0f / sampleFreq);
	q1 += qDot2 * (1.0f / sampleFreq);
	q2 += qDot3 * (1.0f / sampleFreq);
	q3 += qDot4 * (1.0f / sampleFreq);

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}


void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);
		return;
	}

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;   

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0mx = 2.0f * q0 * mx;
		_2q0my = 2.0f * q0 * my;
		_2q0mz = 2.0f * q0 * mz;
		_2q1mx = 2.0f * q1 * mx;
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_2q0q2 = 2.0f * q0 * q2;
		_2q2q3 = 2.0f * q2 * q3;
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3;

		// Reference direction of Earth's magnetic field
		hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
		hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
		_2bx = sqrt(hx * hx + hy * hy);
		_2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		// Gradient decent algorithm corrective step
		s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * (1.0f / sampleFreq);
	q1 += qDot2 * (1.0f / sampleFreq);
	q2 += qDot3 * (1.0f / sampleFreq);
	q3 += qDot4 * (1.0f / sampleFreq);

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update


float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}



#ifdef __cplusplus
}
#endif



using namespace std;

// Small macro to display value in hexadecimal with 2 places
#define HEX(x) setw(2) << setfill('0') << hex << (int)(x)

// The ADXL345 Resisters required for this example
#define DEVID                    0x0f
#define BUFFER_SIZE              0x40

#define L3GD20H                  0x6b
#define LSM303D                  0x1d

#define L3GD20H_REG_CTRL1	      0x20
#define L3GD20H_REG_CTRL2	      0x21

#define L3GD20H_REG_CTRL1_MODE   0x5f
#define L3GD20H_REG_CTRL2_MODE   0x20    


#define LSM303D_REG_CTRL1        0x20
#define LSM303D_REG_CTRL2        0x21
#define LSM303D_REG_CTRL5        0x24
#define LSM303D_REG_CTRL6        0x25
#define LSM303D_REG_CTRL7        0x26

#define LSM303D_REG_CTRL1_MODE   0x57
#define LSM303D_REG_CTRL2_MODE   0x00    
#define LSM303D_REG_CTRL5_MODE   0x64
#define LSM303D_REG_CTRL6_MODE   0x20    
#define LSM303D_REG_CTRL7_MODE   0x00

#define gyro_start               0x28
#define acc_start                0x28
#define mag_start                0x08

#define gyro_scale               0.00875
#define accelScale               0.000061
#define magScale                 0.00016



int writeRegister(int file, unsigned char sensor ,unsigned char address, char value){
   if (ioctl(file, I2C_SLAVE, sensor) < 0){
      cout << "I2C_SLAVE address " << sensor << " failed..." << endl;
      return(2);
   }
   unsigned char buffer[2];
   buffer[0] = address;
   buffer[1] = value;
   if (write(file, buffer, 2)!=2){
      cout << "Failed write to the device" << endl;
      return 1;
   }
   return 0;
}

int readRegisters(int file,unsigned char sensor ,char addresses, unsigned char data[], int size){
   if (ioctl(file, I2C_SLAVE, sensor) < 0){
      cout << "I2C_SLAVE address " << sensor << " failed..." << endl;
      return(2);
   }
   
   char temp = addresses;
   if(size > 1) temp |= 0b10000000;
   
   char buf[1] = { temp };
   if(write(file, buf, 1) !=1){
      cout << "Failed to set address to read from in readFullSensorState() " << endl;
   }
   if ( read(file, data, size) != size) {
      cout << "Failure to read value from I2C Device address." << endl;
   }
   return 0;
}

float combineValues(unsigned char msb, unsigned char lsb, float scale){
   short temp = msb;
   temp = (msb<<8) | lsb;
   // cout << "HEX: " << HEX(temp) << endl;
   return ((float)temp * scale); 
}

float *getXYZ(int file, unsigned char sensor, unsigned char start, float scale){

	float *q = new float[3];
		if (ioctl(file, I2C_SLAVE, sensor) < 0){
		cout << "I2C_SLAVE address " << sensor << " failed..." << endl;
		return q;
	}
	
	unsigned char coords[6];
	readRegisters(file, sensor, start, coords, 6);
	cout << "X_L: " << HEX(coords[0]) << " X_H: " << HEX(coords[1]) << " Y_L: " << HEX(coords[2]) << " Y_H: " << HEX(coords[3]) << " Z_L: " << HEX(coords[4]) << " Z_H: " << HEX(coords[5]) << endl;
	q[0] = combineValues( coords[1], coords[0], scale);
	q[1] = combineValues(  coords[3], coords[2], scale);
	q[2] = combineValues(  coords[5], coords[4], scale);
	return q;
}

void normalModeSet(int file){

   writeRegister(file, L3GD20H, L3GD20H_REG_CTRL1, L3GD20H_REG_CTRL1_MODE);
   writeRegister(file, L3GD20H, L3GD20H_REG_CTRL2, L3GD20H_REG_CTRL2_MODE);

   writeRegister(file, LSM303D, LSM303D_REG_CTRL1, LSM303D_REG_CTRL1_MODE);
   writeRegister(file, LSM303D, LSM303D_REG_CTRL2, LSM303D_REG_CTRL2_MODE);
   writeRegister(file, LSM303D, LSM303D_REG_CTRL5, LSM303D_REG_CTRL5_MODE);
   writeRegister(file, LSM303D, LSM303D_REG_CTRL6, LSM303D_REG_CTRL6_MODE);
   writeRegister(file, LSM303D, LSM303D_REG_CTRL7, LSM303D_REG_CTRL7_MODE);

}

void readMainRegisters(int file){
   cout << "_______" << endl;
   unsigned char who_am_i[1];
   readRegisters(file, L3GD20H, DEVID, who_am_i, 1);

   cout << "WHO AM I L3GD20H:" << HEX(who_am_i[0]) << endl;

   unsigned char ctrl_reg_L3GD20H[2];
   readRegisters(file, L3GD20H, L3GD20H_REG_CTRL1, ctrl_reg_L3GD20H, 2);
   cout << "CTRL1 L3GD20H:" << HEX(ctrl_reg_L3GD20H[0]) << endl;
   cout << "CTRL2 L3GD20H:" << HEX(ctrl_reg_L3GD20H[1]) << endl;

   cout << "-------" << endl;
   // unsigned char who_am_i[1];
   readRegisters(file, LSM303D, DEVID, who_am_i, 1);
   cout << "WHO AM I LSM303D:" << HEX(who_am_i[0]) << endl;


   unsigned char ctrl_reg_LSM303D[8];
   readRegisters(file, LSM303D, LSM303D_REG_CTRL1, ctrl_reg_LSM303D, 8);
   cout << "CTRL1 LSM303D:" << HEX(ctrl_reg_LSM303D[0]) << endl;
   cout << "CTRL2 LSM303D:" << HEX(ctrl_reg_LSM303D[1]) << endl;
   cout << "CTRL5 LSM303D:" << HEX(ctrl_reg_LSM303D[4]) << endl;
   cout << "CTRL6 LSM303D:" << HEX(ctrl_reg_LSM303D[5]) << endl;
   cout << "CTRL7 LSM303D:" << HEX(ctrl_reg_LSM303D[6]) << endl;
   cout << "_______" << endl;

}

void streamQuaternion(int file, int iter){

	
	for(int i = 0; i < iter; i++){

		cout << "gyroscope raw data: "<< endl;
		float* gyroXYZ = getXYZ(file,L3GD20H, gyro_start, gyro_scale);
		cout << "gyroX:" << gyroXYZ[0] <<" dps gyroY:"<< gyroXYZ[1] << " dps gyroZ:" << gyroXYZ[2]<< " dps"  << endl;
		cout<<endl;

		cout << "accelerometer raw data: "<< endl;
		float* accXYZ = getXYZ(file,LSM303D, acc_start , accelScale);
		cout << "accX:" << accXYZ[0] <<" g accY:"<< accXYZ[1] << " g accZ:" << accXYZ[2]  << " g" << endl;
		cout<<endl;
		
		cout << "magnetometer raw data: "<< endl;
		float* magXYZ = getXYZ(file,LSM303D, mag_start , magScale);
		cout << "magX:" << magXYZ[0] <<" gauss magY:"<< magXYZ[1] << " gauss magZ:" << magXYZ[2]  << " gauss" << endl;
		cout<<endl;
		
		

		MadgwickAHRSupdate(gyroXYZ[0], gyroXYZ[1], gyroXYZ[2], accXYZ[0], accXYZ[1], accXYZ[2], magXYZ[0], magXYZ[1], magXYZ[2]);
		

		cout << "q0: " << q0 <<" q1: "<< q1 << " q2: " << q2 << " q3: " << q3 << endl;
		q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
		free(gyroXYZ);
		free(accXYZ);
		free(magXYZ);
		cout<<endl;
		usleep(1000000);
	}
	
}

int main(){
	int file;
	cout << "Starting the ADXL345 sensor application" << endl;
	if((file=open("/dev/i2c-2", O_RDWR)) < 0){
		cout << "failed to open the bus" << endl;
		return 1;
	}

	if(ioctl(file, I2C_SLAVE, LSM303D) < 0){
		cout << "Failed to connect to the sensor LSM303D" << endl;
		return 1;
	}

	if(ioctl(file, I2C_SLAVE, L3GD20H) < 0){
		cout << "Failed to connect to the sensor L3GD20H" << endl;
		return 1;
	}



	normalModeSet(file);
   	

	
	readMainRegisters(file);
	
	cout<<endl;
	streamQuaternion(file, 20);
	close(file);
}
