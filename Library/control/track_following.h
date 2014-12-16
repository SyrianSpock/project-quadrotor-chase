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
 * \file track_following.h
 *
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 * \author Jonathan Arreguit
 * \author Dorian Konrad
 * \author Salah Missri
 * \author David Tauxe
 *
 * \brief This file implements a strategy to follow a GPS track
 *
 ******************************************************************************/

#ifndef TRACK_FOLLOWING_H__
#define TRACK_FOLLOWING_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "mavlink_waypoint_handler.h"
#include "neighbor_selection.h"
#include "position_estimation.h"
#include "mavlink_stream.h"
#include "pid_control.h"

typedef struct
{
	float dist2following;									///< The distance with the neighbor
	mavlink_waypoint_handler_t* waypoint_handler;			///< The pointer to the waypoint handler
	neighbors_t* neighbors;									///< The pointer to the neighbor structure
	position_estimator_t* position_estimator;				///< The pointer to the position estimation structure
}track_following_t;

/**
 * \brief	Initialisation of the track following module
 *
 * \param	track_following			The pointer to the structure of the track following
 * \param	waypoint_handler		The pointer to the structure of the MAVLink waypoint handler
 * \param	neighbors				The pointer to the neighbor selection module
 * \param	position_estimator		The pointer to the position estimation module
 */
void track_following_init(track_following_t* track_following, mavlink_waypoint_handler_t* waypoint_handler, neighbors_t* neighbors, position_estimator_t* position_estimator);


/**
 * \brief	Get the following waypoint
 *
 * \param	track_following			The pointer to the structure of the track following
 */
void track_following_get_waypoint(track_following_t* track_following);


/**
 * \brief	Improve the strategy
 *
 * \param	track_following			The pointer to the structure of the track following
 */
void track_following_improve_waypoint_following(track_following_t* track_following);


// KALMAN PREDICTOR //
// Calls the kalman predictor to get a new waypoint
void track_following_kalman_predictor(track_following_t* track_following);

// \brief   Check if there is a new measurement received
// returns 1 if there is a new message
// returns 0 otherwise
bool track_following_new_message_received(track_following_t* track_following);


// CONTROL //
void track_following_WP_control_PID(track_following_t* track_following);


// FUNCTIONS //
uint32_t track_following_WP_time_last_ms(track_following_t* track_following);

float track_following_WP_distance_XYZ(track_following_t* track_following, int i);


void track_following_send_dist(const track_following_t* track_following, const mavlink_stream_t* mavlink_stream, mavlink_message_t* msg);

#ifdef __cplusplus
}
#endif

#endif // TRACK_FOLLOWING_H__
