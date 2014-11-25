/*******************************************************************************
 * Copyright (c) 2009-2014, MAV'RIC Development Team
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, 
 * this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 * this list of conditions and the following disclaimer in the documentation 
 * and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

/*******************************************************************************
 * \file conf_platform.h
 * 
 * \author MAV'RIC Team
 * 
 * \brief  This file configures the imu for the rev 4 of the maveric autopilot
 *   
 ******************************************************************************/


#ifndef CONF_PLATFORM_H_
#define CONF_PLATFORM_H_

#ifdef __cplusplus
	extern "C" {
#endif

//#include "conf_imu_rev3.h"
//#include "conf_imu_rev4.h"

#define NATIVE_BIG_ENDIAN  

#define MAVLINK_SYS_ID 23
#define MAVLINK_BASE_STATION_ID 255

#define CONF_DIAG
//#define CONF_CROSS

#define RC_INPUT_SCALE 0.8
///< Thrust compensation for hover (relative to center position)
#define THRUST_HOVER_POINT (-0.3f)

///< Define which configuration of the imu to use, depending on the autopilot ID
#if MAVLINK_SYS_ID == 1
#include "MAVsettings/MAV001_conf_imu_rev4.h"
#elif MAVLINK_SYS_ID == 2
#include "MAVsettings/MAV002_conf_imu_rev4.h"
#elif MAVLINK_SYS_ID == 3
#include "MAVsettings/MAV003_conf_imu_rev4.h"
#elif MAVLINK_SYS_ID == 4
#include "MAVsettings/MAV004_conf_imu_rev4.h"
#elif MAVLINK_SYS_ID == 5
#include "MAVsettings/MAV005_conf_imu_rev4.h"
#elif MAVLINK_SYS_ID == 6
#include "MAVsettings/MAV006_conf_imu_rev4.h"
#elif MAVLINK_SYS_ID == 7
#include "MAVsettings/MAV007_conf_imu_rev4.h"
#elif MAVLINK_SYS_ID == 8
#include "MAVsettings/MAV008_conf_imu_rev4.h"
#elif MAVLINK_SYS_ID == 9
#include "MAVsettings/MAV009_conf_imu_rev4.h"
#elif MAVLINK_SYS_ID == 10
#include "MAVsettings/MAV010_conf_imu_rev4.h"
#elif MAVLINK_SYS_ID == 11
#include "MAVsettings/MAV011_conf_imu_rev4.h"
#elif MAVLINK_SYS_ID == 21
#include "MAVsettings/MAV021_conf_imu_rev4.h"
#elif MAVLINK_SYS_ID == 22
#include "MAVsettings/MAV022_conf_imu_rev4.h"
#elif MAVLINK_SYS_ID == 23
#include "MAVsettings/MAV023_conf_imu_rev4.h"
#elif MAVLINK_SYS_ID == 24
#include "MAVsettings/MAV024_conf_imu_rev4.h"
#elif MAVLINK_SYS_ID == 51
#include "MAVsettings/MAV051_conf_imu_rev4.h"
#elif MAVLINK_SYS_ID == 101
#include "MAVsettings/MAV101_conf_imu_rev4.h"
#elif MAVLINK_SYS_ID == 102
#include "MAVsettings/MAV102_conf_imu_rev4.h"
#elif MAVLINK_SYS_ID == 201
#include "MAVsettings/MAV201_conf_imu_rev4.h"
#elif MAVLINK_SYS_ID == 202
#include "MAVsettings/MAV202_conf_imu_rev4.h"
#elif MAVLINK_SYS_ID == 203
#include "MAVsettings/MAV203_conf_imu_rev4.h"
#elif MAVLINK_SYS_ID == 204
#include "MAVsettings/MAV204_conf_imu_rev4.h"
#else
#include "conf_imu_rev4.h"
#endif

#define IMU_X 0				///< Define the index for the IMU array
#define IMU_Y 1				///< Define the index for the IMU array
#define IMU_Z 2				///< Define the index for the IMU array

#define ROLL 0				///< Define the index for the IMU array
#define PITCH 1				///< Define the index for the IMU array
#define YAW 2				///< Define the index for the IMU array

#define X			0		///< Define the index for the IMU array
#define Y			1		///< Define the index for the IMU array
#define Z			2		///< Define the index for the IMU array

#define GYRO_OFFSET 0		///< Define the index for the IMU array
#define ACC_OFFSET 3		///< Define the index for the IMU array
#define MAG_OFFSET 6		///< Define the index for the IMU array
							
#define UPVECTOR_X  0		///< Define the index for the IMU array
#define UPVECTOR_Y  0		///< Define the index for the IMU array
#define UPVECTOR_Z -1		///< Define the index for the IMU array
							
#define FRONTVECTOR_X 1		///< Define the index for the IMU array
#define FRONTVECTOR_Y 0		///< Define the index for the IMU array
#define FRONTVECTOR_Z 0		///< Define the index for the IMU array

//#define FRONTVECTOR_Z -1.3846f	///< Inside value
//#define FRONTVECTOR_Z -0.8985f	///< Outside value

///< Definitions of Platform configuration
#define ROTORCOUNT 4		///< Define number of motors

#define M_REAR_LEFT 0		///< Define the index for the control
#define M_FRONT_LEFT 1		///< Define the index for the control
#define M_FRONT_RIGHT 2		///< Define the index for the control
#define M_REAR_RIGHT 3		///< Define the index for the control

#define M_FR_DIR ( 1)		///< Define the front right motor turn direction
#define M_FL_DIR (-1)		///< Define the front left motor turn direction
#define M_RR_DIR (-1)		///< Define the motor turn direction
#define M_RL_DIR ( 1)		///< Define the motor turn direction

#define M_FRONT 0			///< Define the index for the movement control to go front
#define M_RIGHT 1			///< Define the index for the movement control to go right
#define M_REAR 2			///< Define the index for the movement control to go backward
#define M_LEFT 3			///< Define the index for the movement control to go left

#define M_FRONT_DIR ( 1)	///< Define the direction of control
#define M_RIGHT_DIR (-1)	///< Define the direction of control
#define M_REAR_DIR  ( 1)	///< Define the direction of control
#define M_LEFT_DIR  (-1)	///< Define the direction of control

#define MIN_THRUST -0.9f	///< Define the minimum thrust to apply
#define MAX_THRUST 1.0f		///< Define the maximum thrust to apply
#define SERVO_SCALE 500		///< Define the scale factor for the servos

///< define GPS type
#define GPS_TYPE_UBX

///< GLE: define CS for SPI
#define CS_ON_SERVO_7_8 false

///< define type of remote controller
//#define SPEKTRUM_REMOTE
#define TURNIGY_REMOTE

#ifdef __cplusplus
}
#endif

#endif /* CONF_PLATFORM_H_ */
