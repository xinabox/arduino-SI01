/*
	This is a library for the SI01 
	3D Accelerometer, 3D gyroscope, 3D magnetometer

	The board uses I2C for communication.
	
	The board communicates with two I2C devices:
	* 	LSM9DS1   
	
	Data Sheets:
	LSM9DS1 - http://www.st.com/content/ccc/resource/technical/document/datasheet/1e/3f/2a/d6/25/eb/48/46/DM00103319.pdf/files/DM00103319.pdf/jcr:content/translations/en.DM00103319.pdf
*/

#include <xSI01.h>
#include <xSI01_CONFIG.h>
#include <math.h>

/********************************************************
	xSI01 Class
*********************************************************/
/*-----------------Public Class Function----------------*/
// Constructor
xSI01::xSI01(void){
	// Initialize values to zero
	gX = mX = aX = 0;
	gY = mY = aY = 0;
	gZ = mZ = aZ = 0;
	roll = pitch = yaw = 0;
	gForce = 0;
	LSM9DS1_AG_I2C_ADDRESS = 0x6A;
	LSM9DS1_M_I2C_ADDRESS = 0x1C;
	initSensor();
}

/* Constructor
 *
 * @param I2C_AG, Address if the Accelerometer and Gyroscope
 * @param I2C_M, Address of the magnetometer 
*/
xSI01::xSI01(uint8_t I2C_AG, uint8_t I2C_M){
	// Initialize values to zero
	gX = mX = aX = 0;
	gY = mY = aY = 0;
	gZ = mZ = aZ = 0;
	roll = pitch = yaw = 0;
	gForce = 0;	
	LSM9DS1_AG_I2C_ADDRESS = I2C_AG;
	LSM9DS1_M_I2C_ADDRESS = I2C_M;
	initSensor();	
}
// Poll Sensor for Data
void xSI01::poll(void){
	readAccel();
	readGyro();
	readMag();
}

// Begin Sensor Setup with default parameters
bool xSI01::begin(void){
	uint16_t ID;
	setSensitivity(LSM9DS1_ACCEL_MG_LSB_2G, LSM9DS1_GYRO_DPS_DIGIT_245DPS, LSM9DS1_MAG_MGAUSS_4GAUSS);
	ID = WHO_AM_I();
	if(ID != 0x683D){
		return 0;
	}
	setupMag();
	setupGyro();
	setupAccel();
	return ID;
}

// Set sensor sensitivity list
void xSI01::setSensitivity(float sensitivityACC, float sensitivityGYRO, float sensitivityMAG){
	setup.sensitivity.accel = sensitivityACC;
	setup.sensitivity.gyro = sensitivityGYRO;
	setup.sensitivity.mag = sensitivityMAG;
}

/*-----------------Private Class Function----------------*/
// WHO_AM_I check if sensor is present
uint16_t xSI01::WHO_AM_I(void){
	uint8_t AG_ID, M_ID;
	uint16_t COMBINED_ID = 0;
	AG_ID = M_ID = 0;

	AG_ID = xCore.read8(LSM9DS1_AG_I2C_ADDRESS, WHO_AM_I_AG);
	M_ID = xCore.read8(LSM9DS1_M_I2C_ADDRESS, WHO_AM_I_M);

	COMBINED_ID = ((AG_ID << 8)|M_ID);

	return COMBINED_ID;
}

// initialize sensor setup parameters
void xSI01::initSensor(void){
	setup.gyro.scale 	= 0x00; 
	setup.gyro.bandwidth 	= 0x00; 
	setup.gyro.sampleRate 	= 0xC0; 
	setup.gyro.lowpower	= false;
	setup.gyro.HPF		= false;
	setup.gyro.HPF_CutOFF	= false;
	setup.gyro.enable_axis	= 0x38;
	setup.gyro.latchedIRQ 	= 0x02;

	setup.accel.scale	= 0x00;
	setup.accel.bandwidth	= 0x00;
	setup.accel.sampleRate	= 0xC0;
	setup.accel.enable_axis	= 0x38;
	setup.accel.lowpower	= false;

	setup.mag.scale		= 0x00;
	setup.mag.bandwidth	= 0x80;
	setup.mag.sampleRate	= 0x1C;
	setup.mag.performanceXY = 0x60;
	setup.mag.performanceZ 	= 0x0C;
	setup.mag.mode		= 0x00;
	setup.mag.lowpower	= false;
	setup.mag.enable_axis	= 0x00;
	setup.mag.fast_read	= 0x00;
	setup.mag.BDU		= 0x00;

	setup.temp.enabled	= true;
}

/********************************************************
	Accelerometer 
*********************************************************/
/*-----------------Public Class Function----------------*/
// get Accelerometer X
float xSI01::getAX(void){
	return aX;
}

// get Accelerometer Y
float xSI01::getAY(void){
	return aY;
}

// get Accelerometer Z
float xSI01::getAZ(void){
	return aZ;
}

/*----------------Private Class Function----------------*/
// Begin Accelerometer
void xSI01::setupAccel(void){
	uint8_t tempData = 0;
	
	// enable X,Y,Z axis
	tempData = setup.accel.enable_axis;
	xCore.write8(LSM9DS1_AG_I2C_ADDRESS, CTRL_REG5_XL, tempData);
	tempData = 0;

	tempData = setup.accel.sampleRate;
	tempData |= setup.accel.bandwidth;
	tempData |= setup.accel.scale;
	xCore.write8(LSM9DS1_AG_I2C_ADDRESS, CTRL_REG6_XL, tempData);
	tempData = 0;

	tempData = 0x00;
	xCore.write8(LSM9DS1_AG_I2C_ADDRESS, CTRL_REG7_XL, tempData);
}


// Read Accelerometer Data
void xSI01::readAccel(void){
	int16_t data = 0;
	uint8_t newData = 0;
	
	newData = xCore.read8(LSM9DS1_AG_I2C_ADDRESS, STATUS_REG_A);
	
	if (newData & ACC_READY){	
		// Get X data
		data = xCore.read8(LSM9DS1_AG_I2C_ADDRESS, OUT_X_H_XL);
		data <<= 8;
		data |=  xCore.read8(LSM9DS1_AG_I2C_ADDRESS, OUT_X_L_XL);
		aX = data*setup.sensitivity.accel;
		data = 0;
		
		// get Y data
		data = xCore.read8(LSM9DS1_AG_I2C_ADDRESS, OUT_Y_H_XL);
		data <<= 8;
		data |=  xCore.read8(LSM9DS1_AG_I2C_ADDRESS, OUT_Y_L_XL);
		aY = data*setup.sensitivity.accel;
		data = 0;
		
		// get Z data
		data = xCore.read8(LSM9DS1_AG_I2C_ADDRESS, OUT_Z_H_XL);
		data <<= 8;
		data |= xCore.read8(LSM9DS1_AG_I2C_ADDRESS, OUT_Z_L_XL);
		aZ = data*setup.sensitivity.accel;
	}
}	

/********************************************************
 	Magnotometer 
*********************************************************/
/*-----------------Public Class Function----------------*/
// get Magnotometer X
float xSI01::getMX(void){
	return mX;
}

// get Magnotometer Y
float xSI01::getMY(void){
	return mY;
}

// get Magnotometer Z
float xSI01::getMZ(void){
	return mZ;
}

/*----------------Private Class Function----------------*/
// Begin Magnotometer
void xSI01::setupMag(void){
	uint8_t tempData = 0;
	
	tempData = setup.mag.performanceXY;
	tempData |= setup.mag.sampleRate;
	xCore.write8(LSM9DS1_M_I2C_ADDRESS, CTRL_REG1_M, tempData);
	tempData = 0;

	tempData = setup.mag.scale;
	xCore.write8(LSM9DS1_M_I2C_ADDRESS, CTRL_REG2_M, tempData);
	tempData = 0;

	tempData = setup.mag.mode;
	xCore.write8(LSM9DS1_M_I2C_ADDRESS, CTRL_REG3_M, tempData);	 
	tempData = 0;
	
	tempData = setup.mag.performanceZ;
	xCore.write8(LSM9DS1_M_I2C_ADDRESS, CTRL_REG4_M, tempData);	 
	tempData = 0;

	tempData = setup.mag.fast_read;
	tempData |= setup.mag.BDU;
	xCore.write8(LSM9DS1_M_I2C_ADDRESS, CTRL_REG5_M, tempData);	 
	tempData = 0;
}

//Read Magnotomer Data
void xSI01::readMag(void){
	int16_t data = 0;
	uint8_t newData = 0;
	
	newData = xCore.read8(LSM9DS1_M_I2C_ADDRESS, STATUS_REG_M);
	
	if (newData & MAG_READY){
		data = xCore.read8(LSM9DS1_M_I2C_ADDRESS, OUT_X_H_M);
		data <<= 8;
		data |=  xCore.read8(LSM9DS1_M_I2C_ADDRESS, OUT_X_L_M);
		mX = data*setup.sensitivity.mag;
		data = 0;
		
		data = xCore.read8(LSM9DS1_M_I2C_ADDRESS, OUT_Y_H_M);
		data <<= 8;
		data |= xCore.read8(LSM9DS1_M_I2C_ADDRESS, OUT_Y_L_M);		
		mY = data*setup.sensitivity.mag;
		data = 0;
		
		data = xCore.read8(LSM9DS1_M_I2C_ADDRESS, OUT_Z_H_M);
		data <<= 8;
		data |= xCore.read8(LSM9DS1_M_I2C_ADDRESS, OUT_Z_L_M);		
		mZ = data*setup.sensitivity.mag;
	}
}	

/********************************************************
 	Gyroscope 
*********************************************************/
/*-----------------Public Class Function----------------*/
// get GyroScope X
float xSI01::getGX(void){
	return gX;
}

// get GyroScope Y
float xSI01::getGY(void){
	return gY;
}

// get GyroScope Z
float xSI01::getGZ(void){
	return gZ;
}

/*----------------Private Class Function----------------*/
//Begin Gyroscope
void xSI01::setupGyro(void){
	uint8_t tempData = 0;

	tempData = setup.gyro.sampleRate;
	tempData |= setup.gyro.scale;
	tempData |= setup.gyro.bandwidth;
	xCore.write8(LSM9DS1_AG_I2C_ADDRESS, CTRL_REG1_AG, tempData);
	tempData = 0;

	tempData = 0x00;
	xCore.write8(LSM9DS1_AG_I2C_ADDRESS, CTRL_REG2_G, tempData);

	tempData = 0x00;
	xCore.write8(LSM9DS1_AG_I2C_ADDRESS, CTRL_REG3_G, tempData);

	tempData = setup.gyro.enable_axis;
	tempData |= setup.gyro.latchedIRQ;
	xCore.write8(LSM9DS1_AG_I2C_ADDRESS, CTRL_REG4, tempData);
	tempData = 0;

	tempData = 0x00;
	xCore.write8(LSM9DS1_AG_I2C_ADDRESS, ORIENT_CFG_G, tempData);
	tempData = 0;
}

//Read Gyroscope Data
void xSI01::readGyro(void){
	int16_t data = 0;

	uint8_t newData = 0;
	
	newData = xCore.read8(LSM9DS1_AG_I2C_ADDRESS, STATUS_REG_G);
	if (newData & GYRO_READY){
		data = xCore.read8(LSM9DS1_AG_I2C_ADDRESS, OUT_X_H_G);
		data <<= 8;
		data |=  xCore.read8(LSM9DS1_AG_I2C_ADDRESS, OUT_X_L_G);
		gX = data*setup.sensitivity.gyro;
		data = 0;
		
		data = xCore.read8(LSM9DS1_AG_I2C_ADDRESS, OUT_Y_H_G);
		data <<= 8;
		data |= xCore.read8(LSM9DS1_AG_I2C_ADDRESS, OUT_Y_L_G);
		gY = data*setup.sensitivity.gyro;
		data = 0;
		
		data = xCore.read8(LSM9DS1_AG_I2C_ADDRESS, OUT_Z_H_G);
		data <<= 8;
		data |= xCore.read8(LSM9DS1_AG_I2C_ADDRESS, OUT_Z_L_G);
		gZ = data*setup.sensitivity.gyro;
	}
}	

/********************************************************
 	Calculated Data 
*********************************************************/
float xSI01::getGForce(void){
	gForce = sqrt( (aX*aX) + (aY*aY) + (aZ*aZ) );
	return gForce;
}

float xSI01::getRoll(void){
	roll = ((atan(aY/(sqrt((aX*aX)+(aZ*aZ)))))*180.0)/M_PI;
	return roll;
}

float xSI01::getPitch(void){
	pitch = ((atan(aX/(sqrt((aY*aY)+(aZ*aZ)))))*180.0)/M_PI;
	return pitch;
}

float xSI01::getYaw(void){
	float magNorm, mX_Norm, mY_Norm, mZ_Norm;
	magNorm = sqrt((mX*mX)+(mY*mY)+(mZ*mZ));
	mX_Norm = mX/magNorm;
	mY_Norm = mY/magNorm;
	mZ_Norm = mZ/magNorm;
	
	yaw = atan2((mZ_Norm * sin(getRoll()) - mY_Norm * cos(getRoll())),((mX_Norm * cos(getPitch())) + (mY_Norm * sin(getPitch())* sin(getRoll())) + (mZ_Norm * sin(getPitch()) * cos(getRoll()))));
	yaw *= 180.0/M_PI;
	return yaw;
}
