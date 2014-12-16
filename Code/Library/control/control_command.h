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
 * \file control_command.h
 * 
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *   
 * \brief Data structures to hold torque, thrust, rate, attitude and 
 * velocity commands
 *
 ******************************************************************************/


#include "quaternions.h"

#ifndef CONTROL_COMMAND_H_
#define CONTROL_COMMAND_H_

#ifdef __cplusplus
	extern "C" {
#endif


/**
 * \brief 	Control command mode enum
 */
typedef enum
{
	TORQUE_COMMAND   = 0,
	RATE_COMMAND     = 1,
	ATTITUDE_COMMAND = 2,
	VELOCITY_COMMAND = 3
} control_command_mode_t;


/**
 * \brief	Velocity command mode: either in local or global frame
 */
typedef enum
{
	VELOCITY_COMMAND_MODE_LOCAL	 = 0,
	VELOCITY_COMMAND_MODE_GLOBAL = 1
} velocity_command_mode_t;


/**
 * \brief	Attitude command structure
 */
typedef struct 
{	
	float xyz[3];					///<	Velocity on X, Y and Z axis 
 	velocity_command_mode_t mode;	///< 	Command mode, defines whether the velocity is given in local or global frame
} velocity_command_t;


/**
 * \brief	Attitude command mode, either quaternion, either roll pitch and yaw angles
 */
typedef enum
{
	ATTITUDE_COMMAND_MODE_QUATERNION	= 0,
	ATTITUDE_COMMAND_MODE_RPY			= 1
} attitude_command_mode_t;


/**
 * \brief	Attitude command structure
 */
typedef struct 
{	
	quat_t quat;					///<	Attitude quaternion
	float rpy[3];					///<	Roll, pitch and yaw angles 
 	attitude_command_mode_t mode;	///< 	Command mode, defines whether the quaternion or the rpy angles should be used
} attitude_command_t;


/**
 * \brief	Rate command structure
 */
typedef struct 
{	
	float xyz[3];					///<	Rate on X, Y and Z axis (in rad/s)
} rate_command_t;


/**
 * \brief	Torque command structure
 */
typedef struct 
{	
	float xyz[3];					///<	Torque on X, Y and Z axis 
} torque_command_t;


/**
 * \brief	Thrust command type
 */
typedef struct 
{
	float thrust;
} thrust_command_t;


/**
 * \brief 	Global command structure
 */
typedef struct
{
	control_command_mode_t  mode;
	thrust_command_t        thrust;
	torque_command_t        torque;
	rate_command_t          rate;
	attitude_command_t      attitude;
	velocity_command_t      velocity;
} command_t;


#ifdef __cplusplus
	}
#endif

#endif