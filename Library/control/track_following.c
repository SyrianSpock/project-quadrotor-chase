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
#include "small_matrix.h"
#include "kalman_predictor.h"


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
    // Kalman predictor variables
    static vector_4_t state_estimate;
    static matrix_4x4_t state_estimate_covariance;
    static matrix_4x4_t process_noise_covariance;
    static matrix_4x4_t design_matrix;
    static vector_4_t last_measurement;

    static float max_acc = 10.0f;
    static float delta_t = 0.0f;
    static uint32_t last_time_in_loop = 0;

    // Update time tracker & delta_t
    delta_t = (time_keeper_get_millis() - last_time_in_loop) / 1000.0f;
    last_time_in_loop = time_keeper_get_millis();

    // Flag to signal the disponibility of a new measurement
    static bool new_measurement_received = TRUE;
    static uint32_t last_measurement_time = 0;

    // Check if a new measurement has been received & set flag accordingly
    if(track_following->neighbors->neighbors_list[0].time_msg_received != last_measurement_time) {
        new_measurement_received = TRUE;
        last_measurement_time = track_following->neighbors->neighbors_list[0].time_msg_received;
    }
    else {
        new_measurement_received = FALSE;
    }

    // Initialise Kalman parameters once, and only once
    static bool kalman_init_done = FALSE;

    if(!kalman_init_done) {
        kalman_init(
            &state_estimate,
            &state_estimate_covariance,
            &process_noise_covariance,
            &design_matrix,
            max_acc,
            delta_t);
        kalman_init_done = TRUE;
    }

    // Kalman predictor loop
    kalman_predict(
        &state_estimate,
        &state_estimate_covariance,
        process_noise_covariance,
        delta_t);
    // Only correct the prediction if there is a new measurement
    if(new_measurement_received) {
        kalman_correct(
            &state_estimate,
            &state_estimate_covariance,
            &last_measurement,
            design_matrix,
			track_following);
    }

    // Use Kalman prediction output as waypoint
    track_following->waypoint_handler->waypoint_following.pos[0] = state_estimate.v[0];
    track_following->waypoint_handler->waypoint_following.pos[1] = state_estimate.v[1];
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
