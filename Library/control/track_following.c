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
#include "quick_trig.h"

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
	// Write your code here/Dave's code
	
	static Bool start = true;
	
	static Bool recalculSpeeds = true;
	
	static Bool update = true;
	
	int16_t i;
	
	uint32_t time_actual = time_keeper_get_millis(); // actual time in ms
	static uint32_t timeArtificialWP; // Last artificial waypoint time in ms
	uint32_t time_offset; // time since last Artificial waypoint in ms
	
	static float past_velocity[3];
	static float past_position[3];
	
	static float present_velocity[3];
	static float present_position[3];
	
	static float present_WP_position[3];
	static float present_WP_heading;
	
	static float past_speed;
	static float present_speed;
	
	static float past_heading;
	static float present_heading;
	
	static float dist;
	static float wRate;
	static float dist2WP;
	
	int speedfactor=2;
	int distfactor=2;
	
	uint32_t deltaT = 500; //deltaT in ms//see what happens with different values...
	
	
	
	if(start){
		for(i=0;i<3;++i){
			past_position[i] = track_following->neighbors->neighbors_list[0].position[i];
			past_velocity[i] = track_following->neighbors->neighbors_list[0].velocity[i];
			present_position[i] = track_following->neighbors->neighbors_list[0].position[i];
			present_velocity[i] = track_following->neighbors->neighbors_list[0].velocity[i];
			present_WP_position[i] = present_position[i];
		}
		timeArtificialWP=time_actual-deltaT;//added -deltaT
		start=false;
	}
	
	
	//check for new position and speed coming from the other quad
	for(i=0;i<3;++i){
		if(present_position[i] != track_following->neighbors->neighbors_list[0].position[i]){
			past_position[i]=present_position[i];
			present_position[i] =track_following->neighbors->neighbors_list[0].position[i];
			
			present_WP_position[i]=present_position[i];
			update=true;
		}
		
		if(past_velocity[i] != track_following->neighbors->neighbors_list[0].velocity[i]){
			past_velocity[i]=present_velocity[i];
			present_velocity[i]=track_following->neighbors->neighbors_list[0].velocity[i];
			recalculSpeeds=true;
			update=true;
		}
	}
	
	
	
	if(recalculSpeeds){
		past_speed=maths_fast_sqrt((past_velocity[0]*past_velocity[0])+(past_velocity[1]*past_velocity[1]));
		present_speed=maths_fast_sqrt((present_velocity[0]*present_velocity[0])+(present_velocity[1]*present_velocity[1]));
		
		past_heading=present_heading;
		
		if(past_velocity[1]>=0)
		present_heading = quick_trig_acos(present_velocity[0]/present_speed);
		else if(past_velocity[1]<0)
		present_heading = -(quick_trig_acos(present_velocity[0]/present_speed));
		
		
		dist=present_speed*deltaT/1000.0f*speedfactor;
		wRate=(present_heading-past_heading)/4000.0f;
		
		recalculSpeeds=false;
	}


	time_offset = time_actual - timeArtificialWP;

	//set the artificial WP
	if(time_offset>=deltaT){
		
		present_WP_heading=present_WP_heading+wRate*time_offset;
		
		present_WP_position[0] = present_WP_position[0]+dist*quick_trig_cos(present_heading);
		present_WP_position[1] = present_WP_position[1]+dist*quick_trig_sin(present_heading);
		present_WP_position[2] = present_WP_position[2]; //Assume constant altitude
		
		update=true;
	}
	
	
	
	//send the WP
	
	if(update){
		for(i=0;i<3;++i){
			track_following->waypoint_handler->waypoint_following.pos[i] = present_WP_position[i];
			dist2WP += SQR(present_WP_position[i] - track_following->position_estimator->local_position.pos[i]);
		}
		track_following->dist2following = maths_fast_sqrt(dist2WP)*distfactor;//maybe try adding a little something here to make the robot think the waypoint is further away that it actually is.
		timeArtificialWP=time_keeper_get_millis();
		update = false;
	}
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