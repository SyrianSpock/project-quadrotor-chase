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
{/*
	int16_t i;

	track_following->dist2following = 0.0f;

	for(i=0;i<3;++i)
	{
		track_following->waypoint_handler->waypoint_following.pos[i] = track_following->neighbors->neighbors_list[0].position[i];
		track_following->dist2following += SQR(track_following->neighbors->neighbors_list[0].position[i] - track_following->position_estimator->local_position.pos[i]);
	}

	track_following->dist2following = maths_fast_sqrt(track_following->dist2following);*/
}

void track_following_improve_waypoint_following(track_following_t* track_following)
{
	// Write your code here
	// Write your code here/Dave's code

	static Bool start = true;

	static int update = 1;

	uint32_t time_actual = time_keeper_get_millis(); // actual time in ms
	static uint32_t timeArtificialWP; // Last artificial waypoint time in ms
	uint32_t time_offset; // time since last Artificial waypoint in ms

	static uint32_t timeControl;
	uint32_t time_offset_control;

	static float past_velocity[3];
	static float past_position[3];

	static float present_velocity[3];
	static float present_position[3];

	static float present_WP_position[3];
	static float present_WP_heading;

	static float present_WP_position_control[3];

	static float past_speed;
	static float present_speed;

	static float past_heading;
	static float present_heading;

	static float dist;
	static float wRate;
	//static float dist2WP;

	static float P_factor = 0.5;
	static float I_factor = 0.25;
	static float D_factor = 1;
	static float heading_factor = 0.25;
	
	static float integral[2];

	uint32_t deltaT = 100; //deltaT in ms//see what happens with different values...



	if(start){
		start_init(track_following,past_position,present_position,past_velocity,present_velocity,present_WP_position);
		
		timeControl=time_keeper_get_millis();
		start=false;
	}

	
	
	//check for new position and speed coming from the other quad
	check_new_message(track_following, present_position, present_WP_position, past_position, past_velocity, present_velocity,&update);


	//update the speeds values
	if(update==1)
		recalcul_speed(&past_speed, &present_speed, present_velocity, past_velocity, &past_heading, &present_heading,&present_WP_heading, &dist, &wRate, deltaT);
	


	
	//set the artificial WP
	time_offset = time_actual - timeArtificialWP;

	if(time_offset>=deltaT||update==1){

		timeArtificialWP=set_artificial_waypoint(present_WP_heading, present_WP_position, present_WP_position_control, present_speed, time_offset, heading_factor, wRate);

		update =0;

		//reset the integral
		integral[0]=0;
		integral[1]=0;	
	}


	//cascade PID control
	time_offset_control = time_actual - timeControl;
	timeControl = PID_waypoin_control(track_following, present_WP_position, present_WP_position_control, time_offset_control, P_factor, I_factor, D_factor, integral);
	
	
	//send the WP
	send_update_waypoint(present_WP_position_control,track_following);


		/*for(i=0;i<3;i++)
			dist2WP += SQR(present_WP_position[i] - track_following->position_estimator->local_position.pos[i]);

		dist2WP = maths_fast_sqrt(dist2WP);
		track_following->dist2following = dist2WP;*/
	//}
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

void start_init(track_following_t* track_following,float* past_position, float* present_position, float* past_velocity, float* present_velocity, float* present_WP_position)
{
	for(int i=0;i<3;++i){
		past_position[i] = track_following->neighbors->neighbors_list[0].position[i];
		past_velocity[i] = track_following->neighbors->neighbors_list[0].velocity[i];
		present_position[i] = track_following->neighbors->neighbors_list[0].position[i];
		present_velocity[i] = track_following->neighbors->neighbors_list[0].velocity[i];
		present_WP_position[i] = present_position[i];
	}
}


void check_new_message(track_following_t* track_following, float* present_position, float* present_WP_position,float* past_position,float* past_velocity,float* present_velocity,int* update)
{

	for(int i=0;i<3;++i){
		if(present_position[i] != track_following->neighbors->neighbors_list[0].position[i]){
			past_position[i]=present_position[i];
			present_position[i] =track_following->neighbors->neighbors_list[0].position[i];
			
			present_WP_position[i]=present_position[i];

			*update=1;
			
		}

		if(past_velocity[i] != track_following->neighbors->neighbors_list[0].velocity[i]){
			past_velocity[i]=present_velocity[i];
			present_velocity[i]=track_following->neighbors->neighbors_list[0].velocity[i];
			
			*update=1;
		}
	}
}

void recalcul_speed(float* past_speed,float* present_speed,float* present_velocity, float* past_velocity,float* past_heading, float* present_heading, float* present_WP_heading, float* dist,float* wRate, uint32_t deltaT)
{
	*past_speed=*present_speed;
	*present_speed=maths_fast_sqrt((present_velocity[0]*present_velocity[0])+(present_velocity[1]*present_velocity[1]));

	*past_heading=*present_heading;

	if(*present_speed==0)
		*present_heading = 0;
	else if(past_velocity[1]>=0)
		*present_heading = quick_trig_acos(present_velocity[0]/ *present_speed);
	else if(past_velocity[1]<0)
		*present_heading = -(quick_trig_acos(present_velocity[0]/ *present_speed));

	*present_WP_heading=*present_heading;
	*dist=* present_speed*deltaT/1000.0f;
	*wRate=(*present_heading-*past_heading)/4000.0f;

}


uint32_t set_artificial_waypoint(float present_WP_heading, float* present_WP_position, float* present_WP_position_control,float present_speed, float time_offset, float heading_factor, float wRate)
{
	present_WP_heading=present_WP_heading+wRate*time_offset*heading_factor;

	present_WP_position[0] = present_WP_position[0]+((present_speed*time_offset)/1000.0f)*quick_trig_cos(present_WP_heading);
	present_WP_position[1] = present_WP_position[1]+((present_speed*time_offset)/1000.0f)*quick_trig_sin(present_WP_heading);
	present_WP_position[2] = present_WP_position[2]; //Assume constant altitude
	
	present_WP_position_control[2] = present_WP_position[2];
	
	return (time_keeper_get_millis());

}

uint32_t PID_waypoin_control(track_following_t* track_following, float* present_WP_position, float* present_WP_position_control, uint32_t time_offset_control, float P_factor, float I_factor, float D_factor, float* integral)
{
	static float error[2]={0,0};
	static float past_error[2]={0,0};
	static float derivate[2];
	for(int i=0;i<2;i++){
		
		past_error[i]=error[i];
		error[i]=(present_WP_position[i]-track_following->position_estimator->local_position.pos[i]);
		integral[i]=integral[i]+error[i]*((float)time_offset_control);
		
		if(time_offset_control!=0)
			derivate[i]=(error[i]-past_error[i])/((float)time_offset_control);
		
		else if(time_offset_control==0)
			derivate[i]=0;
		
		present_WP_position_control[i]=present_WP_position[i]+P_factor*error[i]+I_factor*integral[i]+D_factor*derivate[i];
	}
	
	return (time_keeper_get_millis());
	
}

void send_update_waypoint (float* waypoint_pos, track_following_t* track_following)
{
	for(int i=0;i<3;++i)
		track_following->waypoint_handler->waypoint_following.pos[i] = waypoint_pos[i];
}