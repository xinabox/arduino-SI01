/*
	This is a library for the SI01 
	3D Accelerometer, 3D gyroscope, 3D magnetometer

	The board uses I2C for communication.
	
	The board communicates with two I2C devices:
	* 	LSM9DS1   
	
	Data Sheets:
	LSM9DS1 - http://www.st.com/content/ccc/resource/technical/document/datasheet/1e/3f/2a/d6/25/eb/48/46/DM00103319.pdf/files/DM00103319.pdf/jcr:content/translations/en.DM00103319.pdf
*/

#ifndef xSI01_h
#define xSI01_h

// System Includes
#include <inttypes.h>
#include "xCore.h"
#include "xSI01_CONFIG.h"

// Status Registers
#define STATUS_REG_A		0x17
#define STATUS_REG_T		0x27
#define STATUS_REG_G		0x17
#define STATUS_REG_M		0x27

// LSM9DS1 Magneto Registers
#define OFFSET_X_REG_L_M	0x05
#define OFFSET_X_REG_H_M	0x06
#define OFFSET_Y_REG_L_M	0x07
#define OFFSET_Y_REG_H_M	0x08
#define OFFSET_Z_REG_L_M	0x09
#define OFFSET_Z_REG_H_M	0x0A
#define WHO_AM_I_M			0x0F
#define CTRL_REG1_M			0x20
#define CTRL_REG2_M			0x21
#define CTRL_REG3_M			0x22
#define CTRL_REG4_M			0x23
#define CTRL_REG5_M			0x24
#define STATUS_REG			0x27
#define INT_CFG_M			0x30
#define INT_SRC_M			0x31
#define INT_THS_L_M			0x32
#define INT_THS_H_M			0x33

// LSM9DS1 Gyro and Accel Shared Registers
#define WHO_AM_I_AG			0x0F
#define CTRL_REG6_XL		0x20
#define CTRL_REG1_AG		0x10

// LSM9DS1 New Data Comparison
#define GYRO_READY			0x02
#define MAG_READY			0x08
#define ACC_READY			0x01
#define TEMP_READY			0x03

// LSM9DS1 Gyro Specific Registers
#define CTRL_REG2_G			0x11
#define CTRL_REG3_G			0x12
#define ORIENT_CFG_G		0x13
#define INT_GEN_SRC_G		0x14

// LSM9DS1 Accel Specific Registers
#define ACT_THS				0x04
#define ACT_DUR				0x05
#define INT_GEN_CFG_XL		0x06
#define INT_GEN_THS_X_XL	0x07
#define INT_GEN_THS_Y_XL	0x08
#define INT_GEN_THS_Z_XL	0x09
#define INT_GEN_DUR_XL		0x0A
#define REFERENCE_G			0x0B
#define INT1_CTRL			0x0C
#define INT2_CTRL			0x0D
#define CTRL_REG4			0x1E
#define CTRL_REG5_XL		0x1F
#define CTRL_REG7_XL		0x21
#define CTRL_REG8			0x22
#define CTRL_REG9			0x23
#define CTRL_REG10			0x24
#define INT_GEN_SRC_XL		0x26
#define OUT_X_L_XL			0x28
#define OUT_X_H_XL			0x29
#define OUT_Y_L_XL			0x2A
#define OUT_Y_H_XL			0x2B
#define OUT_Z_L_XL			0x2C
#define OUT_Z_H_XL			0x2D
#define FIFO_CTRL			0x2E
#define FIFO_SRC			0x2F

//LSM9DS1 Gyro and Accel Interrupt Registers
#define INT_GEN_CFG_G		0x30
#define INT_GEN_THS_XH_G	0x31
#define INT_GEN_THS_XL_G	0x32
#define INT_GEN_THS_YH_G	0x33
#define INT_GEN_THS_YL_G	0x34
#define INT_GEN_THS_ZH_G	0x35
#define INT_GEN_THS_ZL_G	0x36
#define INT_GEN_DUR_G		0x37

//LSM9DS1 Temperature OUTPUT Registers 
#define OUT_TEMP_L			0x15
#define OUT_TEMP_H			0x16

//LSM9DS1 Gyro OUTPUT Registers 
#define OUT_X_L_G			0x18
#define OUT_X_H_G			0x19
#define OUT_Y_L_G			0x1A
#define OUT_Y_H_G			0x1B
#define OUT_Z_L_G			0x1C
#define OUT_Z_H_G			0x1D

//LSM9DS1 Accel OUTPUT Registers 
#define OUT_X_L_A			0x28
#define OUT_X_H_A			0x29
#define OUT_Y_L_A			0x2A
#define OUT_Y_H_A			0x2B
#define OUT_Z_L_A			0x2C
#define OUT_Z_H_A			0x2D

//LSM9DS1 Mag OUTPUT Registers 
#define OUT_X_L_M			0x28
#define OUT_X_H_M			0x29
#define OUT_Y_L_M			0x2A
#define OUT_Y_H_M			0x2B
#define OUT_Z_L_M			0x2C
#define OUT_Z_H_M			0x2D

// Linear Acceleration: mg per LSB
#define LSM9DS1_ACCEL_MG_LSB_2G 		0.000061
#define LSM9DS1_ACCEL_MG_LSB_4G 		0.000122
#define LSM9DS1_ACCEL_MG_LSB_8G 		0.000244
#define LSM9DS1_ACCEL_MG_LSB_16G 		0.000732

// Magnetic Field Strength: gauss range
#define LSM9DS1_MAG_MGAUSS_4GAUSS      		0.00014
#define LSM9DS1_MAG_MGAUSS_8GAUSS      		0.00029
#define LSM9DS1_MAG_MGAUSS_12GAUSS    		0.00043
#define LSM9DS1_MAG_MGAUSS_16GAUSS    		0.00058

// Angular Rate: dps per LSB
#define LSM9DS1_GYRO_DPS_DIGIT_245DPS      	0.00875
#define LSM9DS1_GYRO_DPS_DIGIT_500DPS      	0.01750
#define LSM9DS1_GYRO_DPS_DIGIT_2000DPS     	0.07000

// WHO_AM_I ID reply
#define AG_CHIP_ID	0x68
#define M_CHIP_ID	0x3D


/*---------------------------------------------------------------*/
class xSI01: public xCoreClass
{
	public:
		/**
		* Class stucture for setup
		*/		
		structSI01 setup;

		/**
		* Constructor
		* Creates a new instance of Sensor class.
		*/	
		xSI01();
		xSI01(uint8_t I2C_AG, uint8_t I2C_M);

		/*
		* Runs the setup of the sensor for all three parts 
		* Magnotometer, Accelerometer and GyroScope.
		*
		* Call this in setup(), before reading any sensor data.
		* @return true if setup was successful.
		*/
		bool begin(void);

		/*
		* Request for the sensor to take a reading from
		* Magnotometer, Accelerometer and GyroScope.
		* Store data in private variables.
		*
		* Call this in loop(), before requesting sensor values.
		* @return none
		*/
		void poll(void);

		/*
		* Request for the sensor Accelerometer data.  
		*
		* Call this in loop(), to get Accelerometer X.
		* @return Accelerometer X.
		*/		
		float getAX(void);
		
		/*
		* Request for the sensor Accelerometer data.
		*
		* Call this in loop(), to get Accelerometer Y.
		* @return Accelerometer Y.
		*/
		float getAY(void);
		
		/*
		* Request for the sensor Accelerometer data.
		*
		* Call this in loop(), to get Accelerometer Z.
		* @return Accelerometer Z.
		*/
		float getAZ(void);
		
		/*
		* Request for the sensor Magnotometer data.
		*
		* Call this in loop(), to get Magnotometer X.
		* @return Magnotometer X
		*/			
		float getMX(void);
		
		/*
		* Request for the sensor Magnotometer data.
		*
		* Call this in loop(), to get Magnotometer Y.
		* @return Magnotometer Y
		*/		
		float getMY(void);
		
		/*
		* Request for the sensor Magnotometer data.
		*
		* Call this in loop(), to get Magnotometer Z.
		* @return Magnotometer Y
		*/
		float getMZ(void);
		
		/*
		* Request for the sensor GyroScope data.
		*
		* Call this in loop(), to get GyroScope Z.
		* @return Magnotometer X
		*/
		float getGX(void);
		
		/*
		* Request for the sensor GyroScope data.
		*
		* Call this in loop(), to get GyroScope Z.
		* @return Magnotometer X
		*/
		float getGY(void);
		
		/*
		* Request for the sensor GyroScope data.
		*
		* Call this in loop(), to get GyroScope Z.
		* @return Magnotometer X
		*/
		float getGZ(void);	

		/*
		* Request for Calculated G-Force
		*
		* Call this in loop(), to get chip Temperature.
		* @return gForce
		*/
		float getGForce(void);

		/*
		* Request for Calculated Sensor Roll.
		*
		* Call this in loop()
		* @return roll
		*/
		float getRoll(void);

		/*
		* Request for Calculated Pitch.
		*
		* Call this in loop()
		* @return pitch
		*/
		float getPitch(void);

		/*
		* Request for the Calculated YAW
		*
		* Call this in loop()
		* @return heading
		*/				
		float getYaw(void);

		/*
		* Sets the sensitivity of the three sensors.
		*
		* @param sensitivityACC, the sensitivity of the Accelerometer.
		* @param sensitivityGYRO, the sensitivity of the GyroScope.
		* @param sensitivityMAG, the sensitivity of the Magnotometer.
		*
		* @return none
		*/			
		void setSensitivity(float sensitivityACC, float sensitivityGYRO, float sensitivityMAG);


	private:
		/*
		* Set the I2C Address 
		*
		* @return WHO_AM_I, combined ID of AG and MAG
		*/
		uint16_t WHO_AM_I(void);

		/*
		* Setup Magnotometer 
		*
		* @return true if successful
		*/
		void setupMag(void);			
		
		/*
		* Setup GyroScope
		*
		* @return true if successful
		*/
		void setupGyro(void);
		
		/*
		* Setup Accelerometer
		*
		* @return true if successful
		*/
		void setupAccel(void);
		
		/*
		* Set defaults value for sensor
		* Magnotometer, Accelerometer and GyroScope.
		*
		* @return none
		*/
		void initSensor(void);

		/*
		* Read X, Y and Z of GyroScope.
		* Store in gX, gY and gZ.
		*
		* @return none
		*/
		void readGyro();
		
		/*
		* Read X, Y and Z of Accelerometer.
		* Store in aX, Ay and aZ.
		*
		* @return none
		*/		
		void readAccel();
		
		/*
		* Read X, Y and Z of Magnotometer
		* Store in mX, mY and mZ.
		*
		* @return none
		*/
		void readMag();	

		// Device I2C Addresses
		uint8_t LSM9DS1_AG_I2C_ADDRESS;
		uint8_t LSM9DS1_M_I2C_ADDRESS;

		// Variables to store sensor data
		float mX, aX, gX;
		float mY, aY, gY;
		float mZ, aZ, gZ;
		float roll, pitch, yaw;
		float gForce;
};

#endif
