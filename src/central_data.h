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
 * \file central_data.h
 * 
 * \author MAV'RIC Team
 *   
 * \brief Place where the central data is stored and initialized
 *
 ******************************************************************************/


#ifndef CENTRAL_DATA_H_
#define CENTRAL_DATA_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "stdbool.h"

#include "time_keeper.h"
#include "qfilter.h"
#include "imu.h"
#include "ahrs.h"
#include "stabilisation_copter.h"

#include "pid_control.h"
#include "streams.h"
#include "buffer.h"
#include "print_util.h"

#include "mavlink_stream.h"
#include "mavlink_communication.h"
#include "coord_conventions.h"
#include "onboard_parameters.h"
#include "gps_ublox.h"
#include "mavlink_waypoint_handler.h"
#include "simulation.h"
#include "bmp085.h"
#include "position_estimation.h"

#include "analog_monitor.h"
#include "navigation.h"
#include "state.h"
#include "stabilisation.h"

#include "hud_telemetry.h"
#include "sd_spi.h"

#include "servos.h"
#include "pwm_servos.h"
#include "servos_mix_quadcopter_diag.h"
#include "remote.h"

#include "state_machine.h"
#include "sd_spi.h"
#include "joystick_parsing.h"
#include "data_logging.h"

#include "neighbor_selection.h"
#include "track_following.h"
#include "simu_gps_track.h"

// TODO : update documentation

/**
 * \brief The central data structure
 */
typedef struct  {
	scheduler_t	scheduler;										///< The scheduler structure
	mavlink_communication_t mavlink_communication;				///< The MAVLink communication structure
	command_t command;											///< The command structure
	servos_t servos;											///< The servos structure
	servo_mix_quadcotper_diag_t servo_mix;						///< The servos mix structure
	remote_t remote;											///< The remote controller structure
	remote_mode_t remote_mode;									///< The remote mode structure

	analog_monitor_t analog_monitor;							///< The analog to digital converter structure

	imu_t imu;													///< The IMU structure
	qfilter_t attitude_filter;									///< The qfilter structure
	ahrs_t ahrs;												///< The attitude estimation structure
	control_command_t controls;									///< The control structure used for rate and attitude modes
	control_command_t controls_nav;								///< The control nav structure used for velocity modes
	control_command_t controls_joystick;						///< The control structure for the joystick

	joystick_parsing_t joystick_parsing;						///< The joystick parsing structure
	
	stabilise_copter_t stabilisation_copter;					///< The stabilisation structure for copter

	gps_t gps;													///< The GPS structure
	
	simulation_model_t sim_model;								///< The simulation model structure
	
	position_estimator_t position_estimator;					///< The position estimaton structure
	
	// aliases
	byte_stream_t *telemetry_down_stream;						///< The pointer to the downcoming telemetry byte stream
	byte_stream_t *telemetry_up_stream;							///< The pointer to the upcoming telemetry byte stream
	byte_stream_t *debug_out_stream;							///< The pointer to the outgoing debug byte stream
	byte_stream_t *debug_in_stream;								///< The pointer to the incoming debug byte stream
	
	mavlink_waypoint_handler_t waypoint_handler;
	
	navigation_t navigation;									///< The structure to perform GPS navigation
	
	state_t state;												///< The structure with all state information
	state_machine_t state_machine;								///< The structure for the state machine
	
	barometer_t pressure;										///< The pressure structure
	
	hud_telemetry_structure_t hud_telemetry_structure;			///< The HUD structure
	
	sd_spi_t sd_spi;											///< The sd_SPI driver structure
	
	data_logging_t data_logging;								///< The log data structure
	
	neighbors_t neighbor_selection;								///< The neighbor selection structure
	
	track_following_t track_following;							///< The track following structure
	simu_gps_track_t simu_gps_track;							///< The simulated gps track
	
} central_data_t;


/**
 * \brief	Initialization of the central data structure
 */
void central_data_init(void);


/**
 * \brief	Get a pointer to the central data
 *
 * \return	A pointer to the structure central data
*/
central_data_t* central_data_get_pointer_to_struct(void);

#ifdef __cplusplus
}
#endif

#endif /* CENTRAL_DATA_H_ */
