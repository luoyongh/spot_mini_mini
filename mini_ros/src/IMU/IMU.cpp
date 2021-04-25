#include "IMU/IMU.hpp"
#include <math.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/i2c-dev.h>
#ifdef __cplusplus 
extern "C" { 
#endif
#include <i2c/smbus.h>
#ifdef __cplusplus 
}
#endif
#include "ros/ros.h"
#define millis() (ros::Time::now().toNSec() / 1000000.0)
#define micros() (ros::Time::now().toNSec() / 1000.0)

/* Wrap an angle in the range [-limit,+limit] (special thanks to Edgar Bonet!) */
static float wrap(float angle,float limit){
  while (angle >  limit) angle -= 2*limit;
  while (angle < -limit) angle += 2*limit;
  return angle;
}

/* INIT and BASIC FUNCTIONS */

MPU6050::MPU6050(){
  setFilterGyroCoef(DEFAULT_GYRO_COEFF);
  setGyroOffsets(0,0,0);
  setAccOffsets(0,0,0);
}

uint8_t MPU6050::begin(const char * filename, int gyro_config_num, int acc_config_num){
  if ((mpu6050IoHandle = open(filename, O_RDWR)) < 0) {
	  ROS_FATAL("Failed to open I2C bus %s for MPU-6050", filename);
	  return 1;
  }
  ROS_INFO("I2C bus opened on %s for MPU-6050", filename);
  if (ioctl(mpu6050IoHandle, I2C_SLAVE, MPU6050_ADDR) < 0) {
	  ROS_FATAL("Failed to acquire bus access and/or talk to I2C slave at address 0x%02X", MPU6050_ADDR);
	  return 2;
  }
  ROS_INFO ("I2C bus access acquired at address 0x%02X", MPU6050_ADDR);
  // changed calling register sequence [https://github.com/rfetick/MPU6050_light/issues/1] -> thanks to augustosc
  uint8_t status = writeData(MPU6050_PWR_MGMT_1_REGISTER, 0x01); // check only the first connection with status
  writeData(MPU6050_SMPLRT_DIV_REGISTER, 0x00);
  writeData(MPU6050_CONFIG_REGISTER, 0x00);
  setGyroConfig(gyro_config_num);
  setAccConfig(acc_config_num);
  
  this->update();
  angleX = this->getAccAngleX();
  angleY = this->getAccAngleY();
  preInterval = millis(); // may cause issue if begin() is much before the first update()
  isAvailable = true;
  return status;
}

uint8_t MPU6050::writeData(uint8_t reg, uint8_t data){
  if (i2c_smbus_write_byte_data(mpu6050IoHandle, reg, data) < 0) {
	  ROS_ERROR("Failed to write %u at %u", data, reg);
	  return 3;
  }
  return 0; // 0 if success
}

// This method is not used internaly, maybe by user...
uint8_t MPU6050::readData(uint8_t reg) {
  return (uint8_t)i2c_smbus_read_byte_data(mpu6050IoHandle, reg);
}

/* SETTER */

uint8_t MPU6050::setGyroConfig(int config_num){
  uint8_t status;
  switch(config_num){
    case 0: // range = +- 250 °/s
	  gyro_lsb_to_degsec = 131.0;
	  status = writeData(MPU6050_GYRO_CONFIG_REGISTER, 0x00);
	  break;
	case 1: // range = +- 500 °/s
	  gyro_lsb_to_degsec = 65.5;
	  status = writeData(MPU6050_GYRO_CONFIG_REGISTER, 0x08);
	  break;
	case 2: // range = +- 1000 °/s
	  gyro_lsb_to_degsec = 32.8;
	  status = writeData(MPU6050_GYRO_CONFIG_REGISTER, 0x10);
	  break;
	case 3: // range = +- 2000 °/s
	  gyro_lsb_to_degsec = 16.4;
	  status = writeData(MPU6050_GYRO_CONFIG_REGISTER, 0x18);
	  break;
	default: // error
	  status = 1;
	  break;
  }
  return status;
}

uint8_t MPU6050::setAccConfig(int config_num){
  uint8_t status;
  switch(config_num){
    case 0: // range = +- 2 g
	  acc_lsb_to_g = 16384.0;
	  status = writeData(MPU6050_ACCEL_CONFIG_REGISTER, 0x00);
	  break;
	case 1: // range = +- 4 g
	  acc_lsb_to_g = 8192.0;
	  status = writeData(MPU6050_ACCEL_CONFIG_REGISTER, 0x08);
	  break;
	case 2: // range = +- 8 g
	  acc_lsb_to_g = 4096.0;
	  status = writeData(MPU6050_ACCEL_CONFIG_REGISTER, 0x10);
	  break;
	case 3: // range = +- 16 g
	  acc_lsb_to_g = 2048.0;
	  status = writeData(MPU6050_ACCEL_CONFIG_REGISTER, 0x18);
	  break;
	default: // error
	  status = 1;
	  break;
  }
  return status;
}

void MPU6050::setGyroOffsets(float x, float y, float z){
  gyroXoffset = x;
  gyroYoffset = y;
  gyroZoffset = z;
}

void MPU6050::setAccOffsets(float x, float y, float z){
  accXoffset = x;
  accYoffset = y;
  accZoffset = z;
}

void MPU6050::setFilterGyroCoef(float gyro_coeff){
  if ((gyro_coeff<0) or (gyro_coeff>1)){ gyro_coeff = DEFAULT_GYRO_COEFF; } // prevent bad gyro coeff, should throw an error...
  filterGyroCoef = gyro_coeff;
}

void MPU6050::setFilterAccCoef(float acc_coeff){
  setFilterGyroCoef(1.0-acc_coeff);
}

/* CALC OFFSET */

void MPU6050::calcOffsets(bool is_calc_gyro, bool is_calc_acc){
  if(is_calc_gyro){ setGyroOffsets(0,0,0); }
  if(is_calc_acc){ setAccOffsets(0,0,0); }
  float ag[6] = {0,0,0,0,0,0}; // 3*acc, 3*gyro
  
  for(int i = 0; i < CALIB_OFFSET_NB_MES; i++){
    this->fetchData();
	ag[0] += accX;
	ag[1] += accY;
	ag[2] += (accZ-1.0);
	ag[3] += gyroX;
	ag[4] += gyroY;
	ag[5] += gyroZ;
	ros::Duration(0.001).sleep();
  }
  
  if(is_calc_acc){
    accXoffset = ag[0] / CALIB_OFFSET_NB_MES;
    accYoffset = ag[1] / CALIB_OFFSET_NB_MES;
    accZoffset = ag[2] / CALIB_OFFSET_NB_MES;
  }
  
  if(is_calc_gyro){
    gyroXoffset = ag[3] / CALIB_OFFSET_NB_MES;
    gyroYoffset = ag[4] / CALIB_OFFSET_NB_MES;
    gyroZoffset = ag[5] / CALIB_OFFSET_NB_MES;
  }
}

/* UPDATE */

void MPU6050::fetchData(){
  int16_t rawData[7]; // [ax,ay,az,temp,gx,gy,gz]

  i2c_smbus_read_block_data(mpu6050IoHandle, MPU6050_ACCEL_OUT_REGISTER, (uint8_t *)&rawData[0]);

  accX = ((float)rawData[0]) / acc_lsb_to_g - accXoffset;
  accY = ((float)rawData[1]) / acc_lsb_to_g - accYoffset;
  accZ = (!upsideDownMounting - upsideDownMounting) * ((float)rawData[2]) / acc_lsb_to_g - accZoffset;
  temp = (rawData[3] + TEMP_LSB_OFFSET) / TEMP_LSB_2_DEGREE;
  gyroX = ((float)rawData[4]) / gyro_lsb_to_degsec - gyroXoffset;
  gyroY = ((float)rawData[5]) / gyro_lsb_to_degsec - gyroYoffset;
  gyroZ = ((float)rawData[6]) / gyro_lsb_to_degsec - gyroZoffset;
}

void MPU6050::update(){
  // retrieve raw data
  this->fetchData();
  
  // estimate tilt angles: this is an approximation for small angles!
  float sgZ = (accZ>=0)-(accZ<0); // allow one angle to go from -180° to +180°
  angleAccX =   atan2(accY, sgZ*sqrt(accZ*accZ + accX*accX)) * RAD_2_DEG; // [-180°,+180°]
  angleAccY = - atan2(accX,     sqrt(accZ*accZ + accY*accY)) * RAD_2_DEG; // [- 90°,+ 90°]

  unsigned long Tnew = millis();
  float dt = (Tnew - preInterval) * 1e-3;
  preInterval = Tnew;

  // Correctly wrap X and Y angles (special thanks to Edgar Bonet!)
  // https://github.com/gabriel-milan/TinyMPU6050/issues/6
  angleX = wrap(filterGyroCoef*(angleAccX + wrap(angleX +     gyroX*dt - angleAccX,180)) + (1.0-filterGyroCoef)*angleAccX,180);
  angleY = wrap(filterGyroCoef*(angleAccY + wrap(angleY + sgZ*gyroY*dt - angleAccY, 90)) + (1.0-filterGyroCoef)*angleAccY, 90);
  angleZ += gyroZ*dt; // not wrapped (to do???)
}