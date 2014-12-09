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
#include "time_keeper.h" // Added to obtain time information

void track_following_init(track_following_t* track_following, mavlink_waypoint_handler_t* waypoint_handler, neighbors_t* neighbors)
{
	track_following->waypoint_handler = waypoint_handler;
	track_following->neighbors = neighbors;

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
		track_following->dist2following += SQR(track_following->neighbors->neighbors_list[0].position[i] - track_following->neighbors->position_estimator->local_position.pos[i]);
	}
	
	track_following->dist2following = maths_fast_sqrt(track_following->dist2following);
}

							// IMPROVE WAYPOINT //

void track_following_improve_waypoint_following(track_following_t* track_following)
{
	// Write your code here	
	track_following_linear_strategy(track_following);
	//track_following_non_linear_strategy(track_following);
}

							// STRATEGIES //

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
	track_following->waypoint_handler->waypoint_following.pos[2] = track_following->neighbors->neighbors_list[0].position[2]*2.0;
}

/*
// CIRCLE STRATEGY
void track_following_non_linear_strategy(track_following_t* track_following, track_following_WP* previous_waypoints)
{
	// Position and velocities -> still need to implement
	float WP_sent_new_pos[3]; // Waypoint last received 
	float WP_sent_new_vel[3];
	float WP_sent_old_pos[3]; // Previous waypoint before last
	float WP_sent_old_vel[3];
	float WP_pos_predict[3]; // Current waypoint the quadrostalker must follow
	
	// Orthogonal velocity vectors to find the rotation centre
	float WP_sent_new_vel_ortho[3] = vector_orthogonal(WP_sent_new_vel);
	float WP_sent_old_vel_ortho[3] = vector_orthogonal(WP_sent_old_vel);
	
	float rotation_centre = vector_intersection(WP_sent_old_pos,
												WP_sent_old_vel_ortho,
												WP_sent_old_pos,
												WP_sent_new_vel_ortho);
												
	float rotation_angle_tot = vector_rotation_angle(WP_sent_old_vel_ortho, WP_sent_new_vel_ortho) 
	
	float t = time_last_WP_ms(track_following);
	// Angle with the last received coordinates viewed from centre :
	float rotation_angle_current = rotation_angle_tot*t/4000;

	WP_pos_predict = WP_rotation_pos(WP_sent_new_pos, rotation_centre, rotation_angle_current);
	
	write_WP_data(,,) // Save new and old waypoints
}
*/
							// Functions //
	
// time_last_WP_ms: Time since last waypoint was received
uint32_t time_last_WP_ms(track_following_t* track_following)
{
	uint32_t timeWP = track_following->neighbors->neighbors_list[0].time_msg_received; // Last waypoint time in ms
	uint32_t time_actual = time_keeper_get_millis(); // actual time in ms
	uint32_t time_offset = time_actual - timeWP; // time since last waypoint in ms
	
	return time_offset;
}

/*
// vector_intersection: returns the coordinates of the vector intersection
float vector_intersection(float v_pos[3], float v_dir[3], float u_pos[3], float u_dir[3])
{
	float intersection_pos[2]; // Intersection coordinates
	float w[2];
	
	for (int i=0; i<2; i++) {
		float w[i] = u_pos[i] - v_pos[i];
	}
	
	float s = (v_dir[1]*w[0] - v_dir[0]*w[1]) / (v_dir[0]*u_dir[1] - v_dir[1]*u_dir[0]);
	
	
	intersection_pos[0] = u_pos[0] + s*w[0];
	intersection_pos[1] = u_pos[1] + s*w[1];
	intersection_pos[2] = u_pos[2];
	
	return intersection_pos;
}

// vector_rotation_angle: returns the angle between the two vectors
float vector_rotation_angle(float v_dir[3], float u_dir[3])
{
	float angle;
	
	float uv_scalar_product = vectors_scalar_product(v_dir[3], u_dir[3]);
	float u_norm = vectors_norm(u_dir[3]);
	float v_norm = vectors_norm(v[3]);
	float angle = quick_trig_acos(uv_scalar_product/(u_norm*v_norm));
	
	return angle;
}

// vector_orthogonal: returns a vector orthogonal to the original
float vector_orthogonal(float v_dir[3])
{
	float v_dir_ortho();
	
	v_dir_ortho[0] = v_dir[1];
	v_dir_ortho[1] = v_dir[0];
	v_dir_ortho[2] = v_dir[2];
	
	return v_dir_ortho;
}

// WP_rotation_pos: Calculates the new waypoint according to the rotation angle
float WP_rotation_pos(float pos[3], float centre[3], float angle)
{
	float WP_new[3];
	
	WP_new[0] = pos[0] - centre[0];
	WP_new[1] = pos[1] - centre[1];
	WP_new[2] = pos[2];
	
	float angle_cos = quick_trig_cos(angle);
	float angle_sin = quick_trig_sin(angle);
	
	WP_new[0] = cos_angle*pos[0] - sin_angle*pos[1];
	WP_new[1] = sin_angle*pos[0] + cos_angle*pos[1];
	
	WP_new[0] = WP_new[0] + centre[0];
	WP_new[1] = WP_new[1] + centre[1];
	
	return WP_new;
}

// write_WP_data: Save the data for the next iteration
void write_WP_data()
{
	// Save previous waypoints
	// Write new waypoint
}
*/