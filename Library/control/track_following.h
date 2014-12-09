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


typedef struct
{
	float dist2following;
	mavlink_waypoint_handler_t* waypoint_handler;
	neighbors_t* neighbors;
}track_following_t; 


/**
 * \brief	Initialisation of the track following module
 *
 * \param	track_following			The pointer to the structure of the track following
 * \param	waypoint_handler		The pointer to the structure of the MAVLink waypoint handler
 * \param	neighbors				The pointer to the neighbor selection module
 */
void track_following_init(track_following_t* track_following, mavlink_waypoint_handler_t* waypoint_handler, neighbors_t* neighbors);


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

							// ADDED STRATEGIES
							
// LINEAR STRATEGY
void track_following_linear_strategy(track_following_t* track_following);

							// ADDED FUNTIONS //

// time_last_WP_ms: Time since last waypoint was received
float time_last_WP_ms(track_following_t* track_following);

#ifdef __cplusplus
}
#endif

#endif // TRACK_FOLLOWING_H__