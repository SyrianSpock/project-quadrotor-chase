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
 * \file simu_gps_track.h
 * 
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 * 
 * \brief  This file simulates a GPS navigation track
 *   
 ******************************************************************************/


#ifndef SIMU_GPS_TRACK_H__
#define SIMU_GPS_TRACK_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "mavlink_stream.h"
#include "mavlink_message_handler.h"
#include "neighbor_selection.h"
#include "tasks.h"

#define MSG_PERIOD_SEC 4 ///< The period in seconds at which the target MAV sends messages, use only 0.25, 0.5, 0.75, 1, 2, 3, 4

/**
 * \brief	Structure of the simulated GPS track.
 */
typedef struct  
{
	int32_t i;									///< The index in the table of waypoints
	
	neighbors_t* neighbors;						///< The pointer to the neighbors module
	const mavlink_stream_t* mavlink_stream;		///< The pointer to the MAVLink stream module
	const state_t* state;						///< The pointer to the state module
}simu_gps_track_t;


/**
 * \brief	Simulates the reception of a message from the target
 *
 * \param	simu_gps_track			The pointer to the simulated GPS track structure
 */
task_return_t simu_gps_track_pack_msg(simu_gps_track_t* simu_gps_track);

/**
 * \brief	Sends the message on the communication channel to the neighbors
 *
 * \param	simu_gps_track			The pointer to the simulated GPS track structure
 * \param	mavlink_stream			The pointer to the MAVLink stream module
 * \param	msg						The pointer to the message that is going to be sent
 */
void simu_gps_track_send_msg(simu_gps_track_t* simu_gps_track, const mavlink_stream_t* mavlink_stream, mavlink_message_t* msg);

/**
 * \brief	Sends the emulated neighbor heartbeat (to show on QGroundControl)
 *
 * \param	simu_gps_track			The pointer to the simulated GPS track structure
 */
void simu_gps_track_send_neighbor_heartbeat(simu_gps_track_t* simu_gps_track);

/**
 * \brief	Initialise the simulated GPS track module
 *
 * \param	simu_gps_track			The pointer to the simulated GPS track structure
 * \param	neighbors				The pointer to the neighbors module
 * \param	mavlink_stream			The pointer to the MAVLink stream module
 * \param	state					The pointer to the state structure
 */
void simu_gps_track_init(simu_gps_track_t* simu_gps_track, neighbors_t* neighbors, const mavlink_stream_t* mavlink_stream, const state_t* state);

#ifdef __cplusplus
}
#endif

#endif /* CONF_PLATFORM_H_ */