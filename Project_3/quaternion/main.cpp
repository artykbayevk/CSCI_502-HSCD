#include<iostream>
#include<stdio.h>
#include<fcntl.h>
#include<sys/ioctl.h>
#include<linux/i2c.h>
#include<linux/i2c-dev.h>
#include<iomanip>
#include<unistd.h>
#include<math.h>
#include <iomanip>
#include "MadgwickAHRS.h"

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

//#define gyro_scale               0.00875
#define accelScale               1.0
#define magScale                 1.0

#define gyro_scale               0.000130494896
//#define accelScale               0.000598754908
//#define magScale                 0.000000012207

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

float combineValues(unsigned char msb, unsigned char lsb){
   short temp = msb;
   temp = (msb<<8) | lsb;
   return (float)temp;
}

float *getXYZ(int file, unsigned char sensor, unsigned char start){

	float *q = new float[3];
		if (ioctl(file, I2C_SLAVE, sensor) < 0){
		cout << "I2C_SLAVE address " << sensor << " failed..." << endl;
		return q;
	}
	
	unsigned char coords[6];
	readRegisters(file, sensor, start, coords, 6);
	printf("raw data: X_L %x X_H %x Y_L %x Y_H %x Z_L %x Z_H %x\n\n", coords[0], coords[1], coords[2], coords[3], coords[4], coords[5]);
	q[0] = combineValues(coords[1], coords[0]);
	q[1] = combineValues(coords[3], coords[2]);
	q[2] = combineValues(coords[5], coords[4]);
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

void streamQuaternion(int file){

	float* gyroXYZ = getXYZ(file, L3GD20H, gyro_start);
    printf("gyro X: %.5f gyro Y: %.5f gyro Z: %.5f\n\n", gyroXYZ[0], gyroXYZ[1], gyroXYZ[2]);
    float* accXYZ = getXYZ(file, LSM303D, acc_start );
    printf("acc X: %.5f acc Y: %.5f acc Z: %.5f\n\n", accXYZ[0], accXYZ[1], accXYZ[2]);
	float* magXYZ = getXYZ(file, LSM303D, mag_start );
    printf("mag X: %.5f mag Y: %.5f mag Z: %.5f\n\n", magXYZ[0], magXYZ[1], magXYZ[2]);

	Madgwick madgwick;
//	madgwick.update(gyroXYZ[0]*gyro_scale, gyroXYZ[1]*gyro_scale, gyroXYZ[2]*gyro_scale, accXYZ[0], accXYZ[1], accXYZ[2], magXYZ[0], magXYZ[1], magXYZ[2]);
    madgwick.updateIMU(gyroXYZ[0]*gyro_scale, gyroXYZ[1]*gyro_scale, gyroXYZ[2]*gyro_scale, accXYZ[0], accXYZ[1], accXYZ[2]);
	printf("q0: %.5f q1: %.5f q2: %.5f q3: %.5f\n\n", madgwick.q0, madgwick.q1, madgwick.q2, madgwick.q3);

//	free(gyroXYZ);
//	free(accXYZ);
//	free(magXYZ);
}



int main(){
	int file;
	cout << "Starting the ADXL345 sensor application\n" << endl;
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

//	normalModeSet(file);

//    reading from 0x10 to 0x1F
//    unsigned char values[16];
//    readRegisters(file, L3GD20H, 0x10, values, 16);

//    for(int i = 0; i < 16; i++){
//        cout << i << " "<<HEX(values[i]) << endl ;
//    }

	
	// reading main ctrl registers
//	readMainRegisters(file);
	streamQuaternion(file);
	
	close(file);
}
