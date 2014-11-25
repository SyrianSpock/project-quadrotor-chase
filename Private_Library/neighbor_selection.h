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
 * \file neighbor_selection.c
 * 
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 * 
 * \brief This file decodes the message from the neighbors and computes the relative position and velocity in local coordinates
 * 
 ******************************************************************************/


#ifndef NEIGHBOR_SEL_H__
#define NEIGHBOR_SEL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "mavlink_stream.h"
#include "position_estimation.h"
#include "mavlink_communication.h"
#include "state.h"
#include "gps_ublox.h"
#include "barometer.h"

#define MAX_NUM_NEIGHBORS 15										///< The maximum number of neighbors

#define SIZE_VHC_ORCA 3.0f											///< The safety size for the collision avoidance strategy

#define MAXSPEED 4.5f												///< The maximum speed of the vehicle

#define ORCA_TIME_STEP_MILLIS 10.0f									///< The time step for the task

#define NEIGHBOR_TIMEOUT_LIMIT_MS 4000								///< 4 seconds timeout limit

#define FREQ_LPF 0.2f

/**
 * \brief The track neighbor structure
 */
typedef struct
{
	uint8_t neighbor_ID;											///< The MAVLink ID of the vehicle
	float position[3];												///< The 3D position of the neighbor in m
	float velocity[3];												///< The 3D velocity of the neighbor in m/s
	float size;														///< The physical size of the neighbor in m
	uint32_t time_msg_received;										///< The time at which the message was received in ms
	float comm_frequency;											///< The frequency of message reception
	uint32_t msg_count;												///< The number of message received
	float extrapolated_position[3];									///< The 3D position of the neighbor
} track_neighbor_t;													///< The structure of information about a neighbor 

/**
 * \brief The collision log structure
 */
typedef struct
{
	bool near_miss_flag[MAX_NUM_NEIGHBORS];
	bool collision_flag[MAX_NUM_NEIGHBORS];
	uint8_t count_collision;
	uint8_t count_near_miss;
	
} collision_log_t;

/**
 * \brief The neighbor structure
 */
typedef struct
{
		uint8_t number_of_neighbors;								///< The actual number of neighbors at a given time step
		float safe_size;											///< The safe size for collision avoidance
		track_neighbor_t neighbors_list[MAX_NUM_NEIGHBORS];			///< The list of neighbors structure
		float alt_consensus;										///< The consensus altitude
		float LPF_alt;												///< The value of the gain of the low-pass filter for the altitude consensus
		float mean_comm_frequency;									///< The mean value of the communication frequency
		float variance_comm_frequency;								///< The variance of the communication frequency
		uint32_t previous_time;										///< The time of the previous loop
		uint32_t update_time_interval;								///< The time between two communication frequency update, in ms
		collision_log_t collision_log;								///< The collision log structure
		float collision_dist_sqr;									///< The square of the collision distance
		float near_miss_dist_sqr;									///< The square of the near-miss distance
		position_estimator_t* position_estimator;					///< The pointer to the position estimator structure
		gps_t* gps;
		barometer_t* barometer;
		const state_t* state;										///< The pointer to the state structure
		const mavlink_stream_t* mavlink_stream;
} neighbors_t;

/**
 * \brief	Initialize the neighbor selection module
 *
 * \param neighbors				The pointer to the neighbor struct
 * \param position_estimator	The pointer to the position estimator struct
 * \param message_handler		The pointer to the message handler structure
 */
void neighbors_selection_init(neighbors_t* neighbors, position_estimator_t *position_estimator, state_t* state, gps_t* gps, barometer_t* barometer, mavlink_message_handler_t *message_handler, const mavlink_stream_t* mavlink_stream);

/**
 * \brief	Decode the message and parse to the neighbor array
 *
 * \param	neighbors			The pointer to the neighbors struct
 * \param	sysid				The system ID
 * \param	msg					The pointer to the MAVLink message
 */
void neighbors_selection_read_message_from_neighbors(neighbors_t *neighbors, uint32_t sysid, mavlink_message_t* msg);

/**
 * \brief	Extrapolate the position of each UAS between two messages, deletes the message if time elapsed too long from last message
 *
 * \param	neighbors			The pointer to the neighbors struct
 */
void neighbors_selection_extrapolate_or_delete_position(neighbors_t *neighbors);

/**
 * \brief	Computes the consensus altitude between all neighbors
 *
 * \param	neighbors			The pointer to the neighbors struct
 * \param	reset_position		The flag that tells to set the consensus offsets 
 */
void neighbors_selection_update_consensus_altitude(neighbors_t *neighbors, bool const reset_position);

/**
 * \brief	Computes the mean and the variance of the communication frequency
 *
 * \param	neighbors			The pointer to the neighbors struct
 */
void neighbors_compute_communication_frequency(neighbors_t* neighbors);

/**
 * \brief	Update the log of the near miss and collisions
 *
 * \param	neighbors			The pointer to the neighbors struct
 */
void neighbors_collision_log(neighbors_t *neighbors);

#ifdef __cplusplus
}
#endif

#endif // NEIGHBOR_SEL_H__