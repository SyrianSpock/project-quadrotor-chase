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

void neighbors_selection_init(neighbors_t* neighbors, position_estimator_t *position_estimator, mavlink_message_handler_t *message_handler, const mavlink_stream_t* mavlink_stream)
{
	neighbors->number_of_neighbors = 0;
	neighbors->position_estimator = position_estimator;
	neighbors->mavlink_stream = mavlink_stream;

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

	if (msg->sysid == 1)
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

		neighbors->neighbors_list[actual_neighbor].neighbor_ID = msg->sysid;

		for (i = 0; i < 3; i++)
		{
			neighbors->neighbors_list[actual_neighbor].position[i] = local_pos_neighbor.pos[i];
		}
		neighbors->neighbors_list[actual_neighbor].velocity[X] = packet.vx / 100.0f;
		neighbors->neighbors_list[actual_neighbor].velocity[Y] = packet.vy / 100.0f;
		neighbors->neighbors_list[actual_neighbor].velocity[Z] = packet.vz / 100.0f;

		neighbors->neighbors_list[actual_neighbor].time_msg_received = time_keeper_get_millis();

	}
}
