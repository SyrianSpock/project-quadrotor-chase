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


#include "neighbor_selection.h"
#include "coord_conventions.h"
#include "conf_platform.h"
#include "print_util.h"
#include "time_keeper.h"
#include <stdbool.h>
#include "delay.h"

void neighbors_selection_init(neighbors_t* neighbors, position_estimator_t *position_estimator, state_t* state, gps_t* gps, barometer_t* barometer, mavlink_message_handler_t *message_handler, const mavlink_stream_t* mavlink_stream)
{
	neighbors->number_of_neighbors = 0;
	neighbors->position_estimator = position_estimator;
	neighbors->mavlink_stream = mavlink_stream;
	
	neighbors->state = state;
	
	neighbors->gps = gps;
	neighbors->barometer = barometer;
	
	neighbors->alt_consensus = position_estimator->local_position.origin.altitude;
	neighbors->LPF_alt = 0.9;
	
	neighbors->mean_comm_frequency = 0.0f;
	neighbors->variance_comm_frequency = 0.0f;
	
	neighbors->previous_time = time_keeper_get_millis();
	
	neighbors->update_time_interval = 1000.0f; // 1 sec
	
	uint8_t i;
	neighbors->collision_log.count_near_miss = 0;
	neighbors->collision_log.count_collision = 0;
	for (i = 0; i < MAX_NUM_NEIGHBORS; i++)
	{
		neighbors->collision_log.near_miss_flag[i] = false;
		neighbors->collision_log.collision_flag[i] = false;
	}
	neighbors->collision_dist_sqr = SQR(6.0f);
	neighbors->near_miss_dist_sqr = SQR(10.0f);
	
	// Add callbacks for onboard parameters requests
	mavlink_message_handler_msg_callback_t callback;

	callback.message_id 	= MAVLINK_MSG_ID_GLOBAL_POSITION_INT; // 33
	callback.sysid_filter 	= MAV_SYS_ID_ALL;
	callback.compid_filter 	= MAV_COMP_ID_ALL;
	callback.function 		= (mavlink_msg_callback_function_t)	&neighbors_selection_read_message_from_neighbors;
	callback.module_struct 	= (handling_module_struct_t)		neighbors;
	mavlink_message_handler_add_msg_callback( message_handler, &callback );
			
	print_util_dbg_print("Neighbor selection initialized.\r\n");
}

void neighbors_selection_read_message_from_neighbors(neighbors_t *neighbors, uint32_t sysid, mavlink_message_t* msg)
{
	
	 
	//Use this block for message debugging
	/*
	print_util_dbg_print("\n Received message with ID");
	print_util_dbg_print_num(msg->msgid, 10);
	print_util_dbg_print(" from system");
	print_util_dbg_print_num(msg->sysid, 10);
	print_util_dbg_print(" for component");
	print_util_dbg_print_num(msg->compid,10);
	print_util_dbg_print( "\r\n");
	*/
	
	
	uint8_t i;
	
	mavlink_global_position_int_t packet;

	mavlink_msg_global_position_int_decode(msg,&packet);
	//Check if coming from a neighbor
	
	if (msg->sysid != (uint8_t)sysid)
	{
		global_position_t global_pos_neighbor;
		local_coordinates_t local_pos_neighbor;
		uint8_t actual_neighbor;
		
		global_pos_neighbor.longitude = (double)packet.lon / 10000000.0f;
		global_pos_neighbor.latitude = (double)packet.lat / 10000000.0f;
		global_pos_neighbor.altitude = (float)packet.alt / 1000.0f;
		global_pos_neighbor.heading = (float)packet.hdg;
		
		local_pos_neighbor = coord_conventions_global_to_local_position(global_pos_neighbor,neighbors->position_estimator->local_position.origin);
		
		local_pos_neighbor.pos[2] = -packet.relative_alt / 1000.0f;
		
		bool ID_found = false;
		i = 0;
		while ((!ID_found)&&(i < neighbors->number_of_neighbors))
		{
			if (msg->sysid == neighbors->neighbors_list[i].neighbor_ID)
			{
				ID_found = true;
			}
			else
			{
				i++;
			}
		}
		
		if (i >= neighbors->number_of_neighbors)
		{
			if (neighbors->number_of_neighbors < MAX_NUM_NEIGHBORS)
			{
				actual_neighbor = neighbors->number_of_neighbors;
				neighbors->number_of_neighbors++;
				neighbors->neighbors_list[actual_neighbor].comm_frequency = 0.0f;
				neighbors->neighbors_list[actual_neighbor].msg_count = 0;
			}
			else
			{
				// This case shouldn't happen
				print_util_dbg_print("Error! There is more neighbors than planned!\n");
				actual_neighbor = neighbors->number_of_neighbors - 1;
			}
		}
		else
		{
			actual_neighbor = i;
		}
		
		if ( (neighbors->state->mav_state == MAV_STATE_STANDBY)||(neighbors->state->mav_state == MAV_STATE_BOOT) )
		{
			neighbors->alt_consensus = neighbors->LPF_alt * neighbors->alt_consensus + (1.0f - neighbors->LPF_alt) * global_pos_neighbor.altitude;
//			print_util_dbg_print("Alt neighbor (x1000):");
//			print_util_dbg_print_num(global_pos_neighbor.altitude*1000.0f,10);
//			print_util_dbg_print(", consensus alt (x100):");
//			print_util_dbg_print_num(neighbors->alt_consensus*100.0f,10);
//			print_util_dbg_print(", alt neighbor(x100):");
//			print_util_dbg_print_num(local_pos_neighbor.pos[2]*100.0f,10);
//			print_util_dbg_print(", own local alt(x100):");
//			print_util_dbg_print_num(neighbors->position_estimator->local_position.pos[2]*100.0f,10);
//			print_util_dbg_print("\r\n");
//			delay_ms(100);
		}

		neighbors->neighbors_list[actual_neighbor].neighbor_ID = msg->sysid;
		
		for (i = 0; i < 3; i++)
		{
			neighbors->neighbors_list[actual_neighbor].position[i] = local_pos_neighbor.pos[i];
		}
		neighbors->neighbors_list[actual_neighbor].velocity[X] = packet.vx / 100.0f;
		neighbors->neighbors_list[actual_neighbor].velocity[Y] = packet.vy / 100.0f;
		neighbors->neighbors_list[actual_neighbor].velocity[Z] = packet.vz / 100.0f;
		
		neighbors->neighbors_list[actual_neighbor].size = SIZE_VHC_ORCA;
		
		neighbors->neighbors_list[actual_neighbor].time_msg_received = time_keeper_get_millis();
		
		neighbors->neighbors_list[actual_neighbor].msg_count++;
		
//		print_util_dbg_print("Neighbor with ID ");
//		print_util_dbg_print_num(neighbors->neighbors_list[actual_neighbor].neighbor_ID,10);
//		print_util_dbg_print(" at position ");
//		print_util_dbg_print_vector(neighbors->neighbors_list[actual_neighbor].position,3);
//		print_util_dbg_print(" with velocity ");
//		delay_ms(100);
//		print_util_dbg_print_vector(neighbors->neighbors_list[actual_neighbor].velocity,3);
//		print_util_dbg_print(" with relative position ");
//		float rel_pos[3];
//		uint8_t i;
//		for (i = 0; i < 3; i++)
//		{
//			rel_pos[i] = neighbors->neighbors_list[actual_neighbor].position[i] - neighbors->position_estimator->local_position.pos[i];
//		}
//		print_util_dbg_print_vector(rel_pos,3);
//		print_util_dbg_print(", dist (x100):");
//		print_util_dbg_print_num(vectors_norm(rel_pos)*100,10);
//		print_util_dbg_print("\r\n");
//		delay_ms(100);
		
	}
}

void neighbors_selection_extrapolate_or_delete_position(neighbors_t *neighbors)
{
	int32_t i, ind, ind_sup;
	uint32_t delta_t;
	
	uint32_t actual_time = time_keeper_get_millis();
	
	for (ind = 0; ind < neighbors->number_of_neighbors; ind++)
	{
		delta_t = actual_time- neighbors->neighbors_list[ind].time_msg_received;

		if (delta_t >= NEIGHBOR_TIMEOUT_LIMIT_MS)
		{
			print_util_dbg_print("Suppressing neighbor number ");
			print_util_dbg_print_num(ind,10);
			print_util_dbg_print("\r\n");
			
			// suppressing element ind
			for (ind_sup = ind; ind_sup < (neighbors->number_of_neighbors - 1); ind_sup++)
			{
				neighbors->neighbors_list[ind_sup] = neighbors->neighbors_list[ind_sup + 1];
			}
			(neighbors->number_of_neighbors)--;
			
		}
		else if (delta_t > ORCA_TIME_STEP_MILLIS)
		{
			// extrapolating the last known position assuming a constant velocity
			
			for(i = 0; i < 3; i++)
			{
				neighbors->neighbors_list[ind].extrapolated_position[i] = neighbors->neighbors_list[ind].position[i] + neighbors->neighbors_list[ind].velocity[i] *((float)delta_t/1000);
			}
			//print_util_dbg_print("Extrapolated position (x100):");
			//print_util_dbg_print_num(neighbors->neighbors_list[ind].extrapolated_position[0],10);
			//print_util_dbg_print(", ");
			//print_util_dbg_print_num(neighbors->neighbors_list[ind].extrapolated_position[1],10);
			//print_util_dbg_print(", ");
			//print_util_dbg_print_num(neighbors->neighbors_list[ind].extrapolated_position[2],10);
			//print_util_dbg_print(")");
		}
		else
		{
			// taking the latest known position
			for (i = 0; i < 3; i++)
			{
				neighbors->neighbors_list[ind].extrapolated_position[i] = neighbors->neighbors_list[ind].position[i];
			}
			//print_util_dbg_print("Last known position (x100):");
			//print_util_dbg_print_num(neighbors->neighbors_list[ind].extrapolated_position[0],10);
			//print_util_dbg_print(", ");
			//print_util_dbg_print_num(neighbors->neighbors_list[ind].extrapolated_position[1],10);
			//print_util_dbg_print(", ");
			//print_util_dbg_print_num(neighbors->neighbors_list[ind].extrapolated_position[2],10);
			//print_util_dbg_print(")");
		}
	}
}

void neighbors_selection_update_consensus_altitude(neighbors_t *neighbors, bool const reset_position)
{
	if (reset_position)
		{
			float new_alt;
			print_util_dbg_print("Old altitude:");
			print_util_dbg_print_num(neighbors->position_estimator->local_position.origin.altitude,10);
			// Mean of own altitude with consensus of neighbors
			new_alt = (neighbors->position_estimator->local_position.origin.altitude + neighbors->number_of_neighbors * neighbors->alt_consensus)/(neighbors->number_of_neighbors+1);
			neighbors->position_estimator->local_position.pos[2] = 0.0;
			//neighbors->gps->alt_consensus_offset += new_alt - neighbors->position_estimator->local_position.origin.altitude;
			//neighbors->barometer->alt_consensus_offset += (new_alt - neighbors->position_estimator->local_position.origin.altitude);
			neighbors->position_estimator->local_position.origin.altitude = new_alt;
			print_util_dbg_print(", New altitude:");
			print_util_dbg_print_num(neighbors->position_estimator->local_position.origin.altitude,10);
			print_util_dbg_print(", consensus (x100):");
			print_util_dbg_print_num(neighbors->alt_consensus*100,10);
			print_util_dbg_print(", alt_cons_off (x100):");
			//print_util_dbg_print_num(neighbors->gps->alt_consensus_offset *100,10);
			print_util_dbg_print("\r\n");
		}
}

void neighbors_compute_communication_frequency(neighbors_t* neighbors)
{
	uint16_t i;

	
	if (neighbors->number_of_neighbors != 0)
	{
		uint32_t actual_time = time_keeper_get_millis();
		
		if ((actual_time - neighbors->previous_time) >= neighbors->update_time_interval)
		{
			//mavlink_message_t msg;
			
			neighbors->mean_comm_frequency = 0.0f;
			neighbors->variance_comm_frequency = 0.0f;
			
			for (i=0;i<neighbors->number_of_neighbors;++i)
			{
				//mavlink_msg_named_value_int_pack(	neighbors->mavlink_stream->sysid,
													//neighbors->mavlink_stream->compid,
													//&msg,
													//time_keeper_get_millis(),
													//"msg_count",
													//neighbors->neighbors_list[i].msg_count);
				//mavlink_stream_send(neighbors->mavlink_stream,&msg);
				
				neighbors->neighbors_list[i].comm_frequency = FREQ_LPF*neighbors->neighbors_list[i].comm_frequency + (1.0f-FREQ_LPF)*neighbors->neighbors_list[i].msg_count*1000.0f/(actual_time - neighbors->previous_time);
				neighbors->neighbors_list[i].msg_count = 0;
				neighbors->mean_comm_frequency += neighbors->neighbors_list[i].comm_frequency;
			}
			neighbors->previous_time = actual_time;
			neighbors->mean_comm_frequency /= neighbors->number_of_neighbors;

			for (i=0;i<neighbors->number_of_neighbors;++i)
			{
				neighbors->variance_comm_frequency += SQR(neighbors->mean_comm_frequency - neighbors->neighbors_list[i].comm_frequency);
			}
			neighbors->variance_comm_frequency /= neighbors->number_of_neighbors;
			
			
			
			//mavlink_msg_named_value_float_pack(	neighbors->mavlink_stream->sysid,
												//neighbors->mavlink_stream->compid,
												//&msg,
												//time_keeper_get_millis(),
												//"comm_freq",neighbors->mean_comm_frequency);
			//mavlink_stream_send(neighbors->mavlink_stream,&msg);
		}
	}
	else
	{
		neighbors->mean_comm_frequency = 0.0f;
		neighbors->variance_comm_frequency = 0.0f;
	}
}

void neighbors_collision_log(neighbors_t *neighbors)
{
	uint8_t ind, i;
	float relative_position[3];
	float dist;
	for (ind = 0; ind < neighbors->number_of_neighbors; ind++)
	{
		for (i = 0; i < 3; i++)
		{
			relative_position[i] = neighbors->position_estimator->local_position.pos[i] - neighbors->neighbors_list[ind].extrapolated_position[i];
		}
		dist = vectors_norm_sqr(relative_position);
		
		if (dist < neighbors->collision_dist_sqr && !neighbors->collision_log.collision_flag[ind])
		{
			neighbors->collision_log.count_collision++;
			if (neighbors->collision_log.count_near_miss != 0)
			{
				neighbors->collision_log.count_near_miss--;
			}
			neighbors->collision_log.collision_flag[ind] = true;
			print_util_dbg_print("Collision with neighbor:");
			print_util_dbg_print_num(ind,10);
			print_util_dbg_print(", nb of collisions:");
			print_util_dbg_print_num(neighbors->collision_log.count_collision,10);
			print_util_dbg_print("\r\n");
		}
		else if (dist < neighbors->near_miss_dist_sqr && !neighbors->collision_log.near_miss_flag[ind])
		{
			neighbors->collision_log.count_near_miss++;
			neighbors->collision_log.near_miss_flag[ind] = true;
			print_util_dbg_print("Near miss with neighbor:");
			print_util_dbg_print_num(ind,10);
			print_util_dbg_print(", nb of near miss:");
			print_util_dbg_print_num(neighbors->collision_log.count_near_miss,10);
			print_util_dbg_print("\r\n");
		}
		else if (dist > neighbors->near_miss_dist_sqr)
		{
			neighbors->collision_log.collision_flag[ind] = false;
			neighbors->collision_log.near_miss_flag[ind] = false;
		}
	}
}