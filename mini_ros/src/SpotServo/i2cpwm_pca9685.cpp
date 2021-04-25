
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
#include <ros/ros.h>
#include "SpotServo/i2cpwm_pca9685.h"

/// @cond PRIVATE_NO_PUBLIC DOC

#define _BASE_ADDR   0x40
#ifndef _PI
#define _PI 3.14159265358979323846
#endif
#define _CONST(s) ((char*)(s))

enum pwm_regs {
  // Registers/etc.
  __MODE1              = 0x00,
  __MODE2              = 0x01,
  __SUBADR1            = 0x02,      // enable sub address 1 support
  __SUBADR2            = 0x03,      // enable sub address 2 support
  __SUBADR3            = 0x04,      // enable sub address 2 support
  __PRESCALE           = 0xFE,
  __CHANNEL_ON_L       = 0x06,
  __CHANNEL_ON_H       = 0x07,
  __CHANNEL_OFF_L      = 0x08,
  __CHANNEL_OFF_H      = 0x09,
  __ALL_CHANNELS_ON_L  = 0xFA,
  __ALL_CHANNELS_ON_H  = 0xFB,
  __ALL_CHANNELS_OFF_L = 0xFC,
  __ALL_CHANNELS_OFF_H = 0xFD,
  __RESTART            = 0x80,
  __SLEEP              = 0x10,      // enable low power mode
  __ALLCALL            = 0x01,
  __INVRT              = 0x10,      // invert the output control logic
  __OUTDRV             = 0x04
};

#define MAX_BOARDS 62
#define MAX_SERVOS (16*MAX_BOARDS)

int _last_servo = -1;

int _pwm_boards[MAX_BOARDS];                // we can support up to 62 boards (1..62)
int _active_board = 0;                      // used to determine if I2C SLAVE change is needed
int _pca9685_io_handle;                  // linux file handle for I2C

int _pwm_frequency = 50;                    // frequency determines the size of a pulse width; higher numbers make RC servos buzz


/// @endcond PRIVATE_NO_PUBLIC DOC



//* ------------------------------------------------------------------------------------------------------------------------------------
// local private methods
//* ------------------------------------------------------------------------------------------------------------------------------------
/*
static float _abs (float v1)
{
	if (v1 < 0)
		return (0 - v1);
	return v1;
}

static float _min (float v1, float v2)
{
	if (v1 > v2)
		return v2;
	return v1;
}

static float _max (float v1, float v2)
{
	if (v1 < v2)
		return v2;
	return v1;
}

static float _absmin (float v1, float v2)
{
	float a1, a2;
	float sign = 1.0;
	//	if (v1 < 0)
	//		sign = -1.0;
	a1 = _abs(v1);
	a2 = _abs(v2);
	if (a1 > a2)
		return (sign * a2);
	return v1;
}

static float _absmax (float v1, float v2)
{
	float a1, a2;
	float sign = 1.0;
	//	if (v1 < 0)
	//		sign = -1.0;
	a1 = _abs(v1);
	a2 = _abs(v2);
	if (a1 < a2)
		return (sign * a2);
	return v1;
}
*/


/**
 * \private method to set a pulse frequency
 *
 *The pulse defined by start/stop will be active on all channels until any subsequent call changes it.
 *@param frequency an int value (1..15000) indicating the pulse frequency where 50 is typical for RC servos
 *Example _set_frequency (68)  // set the pulse frequency to 68Hz
 */
void set_pwm_frequency (int freq)
{
    int prescale;
    char oldmode, newmode;

    _pwm_frequency = freq;   // save to global
    
	ROS_DEBUG("set_pwm_frequency prescale");
    float prescaleval = 25000000.0; // 25MHz
    prescaleval /= 4096.0;
    prescaleval /= (float)freq;
    prescaleval -= 1.0;
    //ROS_INFO("Estimated pre-scale: %6.4f", prescaleval);
    prescale = floor(prescaleval + 0.5);
    // ROS_INFO("Final pre-scale: %d", prescale);


	ROS_INFO("Setting PWM frequency to %d Hz", freq);

    nanosleep ((const struct timespec[]){{1, 000000L}}, NULL); 


    oldmode = i2c_smbus_read_byte_data (_pca9685_io_handle, __MODE1);
    newmode = (oldmode & 0x7F) | 0x10; // sleep

    if (0 > i2c_smbus_write_byte_data (_pca9685_io_handle, __MODE1, newmode)) // go to sleep
        ROS_ERROR("Unable to set PWM controller to sleep mode"); 

    if (0 >  i2c_smbus_write_byte_data(_pca9685_io_handle, __PRESCALE, (int)(floor(prescale))))
        ROS_ERROR("Unable to set PWM controller prescale"); 

    if (0 > i2c_smbus_write_byte_data(_pca9685_io_handle, __MODE1, oldmode))
        ROS_ERROR("Unable to set PWM controller to active mode"); 

    nanosleep((const struct timespec[]){{0, 5000000L}}, NULL);   //sleep 5microsec,

    if (0 > i2c_smbus_write_byte_data(_pca9685_io_handle, __MODE1, oldmode | 0x80))
        ROS_ERROR("Unable to restore PWM controller to active mode");
}



/**
 * \private method to set a common value for all PWM channels on the active board
 *
 *The pulse defined by start/stop will be active on all channels until any subsequent call changes it.
 *@param start an int value (0..4096) indicating when the pulse will go high sending power to each channel.
 *@param end an int value (0..4096) indicating when the pulse will go low stoping power to each channel.
 *Example set_pwm_interval_all (0, 108)   // set all servos with a pulse width of 105
 */
void set_pwm_interval_all (int start, int end)
{
    // the public API is ONE based and hardware is ZERO based
    if ((_active_board<1) || (_active_board>62)) {
        ROS_ERROR("Internal error - invalid active board number %d :: PWM board numbers must be between 1 and 62", _active_board);
        return;
    }

    if (0 > i2c_smbus_write_byte_data (_pca9685_io_handle, __ALL_CHANNELS_ON_L, start & 0xFF))
        ROS_ERROR ("Error setting PWM start low byte for all servos on board %d", _active_board);
    if (0 >  i2c_smbus_write_byte_data (_pca9685_io_handle, __ALL_CHANNELS_ON_H, start  >> 8))
        ROS_ERROR ("Error setting PWM start high byte for all servos on board %d", _active_board);
    if (0 > i2c_smbus_write_byte_data (_pca9685_io_handle, __ALL_CHANNELS_OFF_L, end & 0xFF))
        ROS_ERROR ("Error setting PWM end low byte for all servos on board %d", _active_board);
    if (0 > i2c_smbus_write_byte_data (_pca9685_io_handle, __ALL_CHANNELS_OFF_H, end >> 8))
        ROS_ERROR ("Error setting PWM end high byte for all servos on board %d", _active_board);
}



/**
 * \private method to set the active board
 *
 *@param board an int value (1..62) indicating which board to activate for subsequent service and topic subscription activity where 1 coresponds to the default board address of 0x40 and value increment up
 *Example set_active_board (68)   // set the pulse frequency to 68Hz
 */
void set_active_board (int board)
{
	char mode1res;

	if ((board<1) || (board>62)) {
        ROS_ERROR("Internal error :: invalid board number %d :: board numbers must be between 1 and 62", board);
        return;
    }
    if (_active_board != board) {
        _active_board = board;   // save to global
        
        // the public API is ONE based and hardware is ZERO based
        board--;
        
        if (0 > ioctl (_pca9685_io_handle, I2C_SLAVE, (_BASE_ADDR+(board)))) {
            ROS_FATAL ("Failed to acquire bus access and/or talk to I2C slave at address 0x%02X", (_BASE_ADDR+board));
            return; /* exit(1) */   /* additional ERROR HANDLING information is available with 'errno' */
        }
        ROS_INFO ("I2C bus access acquired at address 0x%02X", (_BASE_ADDR+board));

        if (_pwm_boards[board]<0) {
            _pwm_boards[board] = 1;

            /* this is guess but I believe the following needs to be done on each board only once */

            if (0 > i2c_smbus_write_byte_data (_pca9685_io_handle, __MODE2, __OUTDRV))
                ROS_ERROR ("Failed to enable PWM outputs for totem-pole structure");

            if (0 > i2c_smbus_write_byte_data (_pca9685_io_handle, __MODE1, __ALLCALL))
                ROS_ERROR ("Failed to enable ALLCALL for PWM channels");

            nanosleep ((const struct timespec[]){{0, 5000000L}}, NULL);   //sleep 5microsec, wait for osci


            mode1res = i2c_smbus_read_byte_data (_pca9685_io_handle, __MODE1);
            mode1res = mode1res & ~__SLEEP; //                 # wake up (reset sleep)

            if (0 > i2c_smbus_write_byte_data (_pca9685_io_handle, __MODE1, mode1res))
                ROS_ERROR ("Failed to recover from low power mode");

            nanosleep((const struct timespec[]){{0, 5000000L}}, NULL);   //sleep 5microsec, wait for osci

            // the first time we activate a board, we mark it and set all of its servo channels to 0
            set_pwm_interval_all (0, 0);
        }
    }
}



/**
 * \private method to set a value for a PWM channel on the active board
 *
 *The pulse defined by start/stop will be active on the specified servo channel until any subsequent call changes it.
 *@param servo an int value (1..16) indicating which channel to change power
 *@param start an int value (0..4096) indicating when the pulse will go high sending power to each channel.
 *@param end an int value (0..4096) indicating when the pulse will go low stoping power to each channel.
 *Example set_pwm_interval (3, 0, 350)    // set servo #3 (fourth position on the hardware board) with a pulse of 350
 */
void set_pwm_interval (int servo, int start, int end)
{
	ROS_DEBUG("set_pwm_interval enter");

    if ((servo<1) || (servo>(MAX_SERVOS))) {
        ROS_ERROR("Invalid servo number %d :: servo numbers must be between 1 and %d", servo, MAX_BOARDS);
        return;
    }

	int board = ((int)((servo-1)/16))+1;	// servo 1..16 is board #1, servo 17..32 is board #2, etc.
	set_active_board(board);

	servo = ((servo-1) % 16) + 1;			// servo numbers are 1..16


    // the public API is ONE based and hardware is ZERO based
    board = _active_board - 1;				// the hardware enumerates boards as 0..61
    int channel = servo - 1;				// the hardware enumerates servos as 0..15
	ROS_DEBUG("set_pwm_interval board=%d servo=%d", board, servo);
    
    if (0 > i2c_smbus_write_byte_data (_pca9685_io_handle, __CHANNEL_ON_L+4*channel, start & 0xFF))
        ROS_ERROR ("Error setting PWM start low byte on servo %d on board %d", servo, _active_board);
    if (0 >  i2c_smbus_write_byte_data (_pca9685_io_handle, __CHANNEL_ON_H+4*channel, start  >> 8))
        ROS_ERROR ("Error setting PWM start high byte on servo %d on board %d", servo, _active_board);
    if (0 > i2c_smbus_write_byte_data (_pca9685_io_handle, __CHANNEL_OFF_L+4*channel, end & 0xFF))
        ROS_ERROR ("Error setting PWM end low byte on servo %d on board %d", servo, _active_board);
    if (0 > i2c_smbus_write_byte_data (_pca9685_io_handle, __CHANNEL_OFF_H+4*channel, end >> 8))
        ROS_ERROR ("Error setting PWM end high byte on servo %d on board %d", servo, _active_board);
}


/**
 \private method to initialize private internal data structures at startup

@param devicename a string value indicating the linux I2C device

Example pca9685_init ("/dev/i2c-1");  // default I2C device on RPi2 and RPi3 = "/dev/i2c-1"
 */
int pca9685_init (const char* filename)
{
    /* initialize all of the global data objects */
    
    for (int i=0; i<MAX_BOARDS;i++)
        _pwm_boards[i] = -1;
    _active_board = -1;
	_last_servo = -1;	
	
    if ((_pca9685_io_handle = open (filename, O_RDWR)) < 0) {
        ROS_FATAL ("Failed to open I2C bus %s for PCA9685", filename);
        return 1; /* exit(1) */   /* additional ERROR HANDLING information is available with 'errno' */
    }
	ROS_INFO ("I2C bus opened on %s for PCA9685", filename);
    return 0;
}

