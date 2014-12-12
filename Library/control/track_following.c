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
 * \file track_following.c
 * 
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 *   
 * \brief This file implements a strategy to follow a GPS track
 *
 ******************************************************************************/


#include "track_following.h"
#include "print_util.h"
#include "maths.h"
#include "time_keeper.h"

void track_following_init(track_following_t* track_following, mavlink_waypoint_handler_t* waypoint_handler, neighbors_t* neighbors, position_estimator_t* position_estimator)
{
	track_following->waypoint_handler = waypoint_handler;
	track_following->neighbors = neighbors;
	track_following->position_estimator = position_estimator;

	track_following->dist2following = 0.0f;

	print_util_dbg_print("[TRACK FOLLOWING] Initialized\r\n");
} 


void track_following_get_waypoint(track_following_t* track_following)
{
	int16_t i;
	
	track_following->dist2following = 0.0f;
	
	for(i=0;i<3;++i)
	{
		track_following->waypoint_handler->waypoint_following.pos[i] = track_following->neighbors->neighbors_list[0].position[i];
		track_following->dist2following += SQR(track_following->neighbors->neighbors_list[0].position[i] - track_following->position_estimator->local_position.pos[i]);
	}
	
	track_following->dist2following = maths_fast_sqrt(track_following->dist2following);
}

void track_following_improve_waypoint_following(track_following_t* track_following)
{
	// Write your code here
	track_following_linear_strategy(track_following);
}

							// PREDICTION //
							
// LINEAR STRATEGY
void track_following_linear_strategy(track_following_t* track_following)
{
	uint32_t time_offset = time_last_WP_ms(track_following);
	
	float position_offset = 0;
	
	for(int i=0;i<2;i++)
	{
		// velocity in m/s and time_offset in ms
		position_offset = track_following->neighbors->neighbors_list[0].velocity[i]*((float)time_offset)/1000.0f;
		
		track_following->waypoint_handler->waypoint_following.pos[i] =
				track_following->neighbors->neighbors_list[0].position[i]
				+ position_offset;
	}
	// track_following->waypoint_handler->waypoint_following.pos[2] = track_following->neighbors->neighbors_list[0].position[2];
	control_WP_PID(track_following);
}
							
							// CONTROL //

// Implement PID for the waypoint following
void control_WP_PID(track_following_t* track_following)
{
	float KP = 1;
	float error = 0;
	
	for(int i=0;i<2; i++)
	{
		error = distance_WP_XYZ(track_following,i);
		track_following->waypoint_handler->waypoint_following.pos[i] = track_following->position_estimator->local_position.pos[i]
																	+ KP*error;
	}
}							
							
							// FUNCTIONS //
							
// time_last_WP_ms: Time since last waypoint was received
uint32_t time_last_WP_ms(track_following_t* track_following)
{
	uint32_t timeWP = track_following->neighbors->neighbors_list[0].time_msg_received; // Last waypoint time in ms
	uint32_t time_actual = time_keeper_get_millis(); // actual time in ms
	uint32_t time_offset = time_actual - timeWP; // time since last waypoint in ms
	
	return time_offset;
}

// calculate distance to waypoint according to an axis
float distance_WP_XYZ(track_following_t* track_following, int i)
{
	float distance = track_following->waypoint_handler->waypoint_following.pos[i]
					- track_following->position_estimator->local_position.pos[i];
	return distance;
}
							
void track_following_send_dist(const track_following_t* track_following, const mavlink_stream_t* mavlink_stream, mavlink_message_t* msg)
{
	mavlink_msg_named_value_float_pack(	mavlink_stream->sysid,
										mavlink_stream->compid,
										msg,
										time_keeper_get_millis(),
										"dist2follow",
										track_following->dist2following);
}