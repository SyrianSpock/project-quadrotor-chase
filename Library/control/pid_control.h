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
 * \file pid_control.h
 * 
 * \author MAV'RIC Team
 * \author Felix Schill
 *   
 * \brief PID controller
 *
 ******************************************************************************/


#ifndef PID_CONTROL_H_
#define PID_CONTROL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <math.h>

typedef struct {
	float pregain, postgain;			///< pregain and postgain 
	float accumulator;					///< accumulator
	float maths_clip;							///< clipping value
	float leakiness;					///< leakiness
} integrator_t;							///< integrator parameters

typedef struct {
	float gain;							///< gain 
	float previous;						///< Previous input to the differentiator 
	float LPF;							///< Low pass filter
	float maths_clip;							///< clipping value
} differentiator_t;						///< differentiator parameters

typedef struct {
	float p_gain;						///< proportional gain
	float clip_min, clip_max;			///< min and max clipping values
	integrator_t integrator;			///< integrator parameters
	differentiator_t differentiator;	///< differentiator parameters
	float output;						///< output
	float error;						///< error
	uint32_t last_update;				///< last update time in timer tick
	float dt;							///< time step
	float soft_zone_width;				///< approximate width of a "soft zone" on the error input, i.e. a region of low gain around the target point. Value 0 -> switched off
} pid_controller_t;						///< PID controller parameters

/**
 * \brief	Passing through the controller
 *
 * \return	A PID controller structure
 */
pid_controller_t pid_control_pass_through_controller(void);

/**
 * \brief	Reset integrator
 *
 * \param	integrator	The pointer to the integrator structure
 */
void pid_control_reset_integrator(integrator_t *integrator);

/**
 * \brief				Update the PID controller
 *
 * \param	controller	Pointer to the PID controller structure
 * \param	error		Error in the controlled variable
 *
 * \return				The controller output
 */
float pid_control_update(pid_controller_t* controller, float error);

/**
 * \brief				Update the PID controller for a given time step
 *
 * \param	controller	Pointer to the PID controller structure
 * \param	error		Error in the controlled variable
 * \param	dt			Timestep
 *
 * \return				The controller output
 */
float pid_control_update_dt(pid_controller_t* controller, float error, float dt);

#ifdef __cplusplus
}
#endif

#endif /* PID_CONTROL_H_ */
