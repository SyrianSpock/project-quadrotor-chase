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
 * \author Jonathan Arreguit
 * \author Dorian Konrad
 * \author Salah Missri
 * \author David Tauxe
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
    // Predict waypoint position with a Kalman algorithm
    track_following_kalman_predictor(track_following);

    // Apply PID control
    track_following_WP_control_PID(track_following);
}

// KALMAN PREDICTOR //

void track_following_kalman_predictor(track_following_t* track_following)
{
    // Kalman variables for x axis
    static vector_2_t state_estimate_x;
    static matrix_2x2_t state_estimate_covariance_x;
    static matrix_2x2_t process_noise_covariance_x;
    static matrix_2x2_t design_matrix_x;
    static vector_2_t last_measurement_x;
    // Kalman variables for y axis
    static vector_2_t state_estimate_y;
    static matrix_2x2_t state_estimate_covariance_y;
    static matrix_2x2_t process_noise_covariance_y;
    static matrix_2x2_t design_matrix_y;
    static vector_2_t last_measurement_y;
    // Kalman variables for z axis
    static vector_2_t state_estimate_z;
    static matrix_2x2_t state_estimate_covariance_z;
    static matrix_2x2_t process_noise_covariance_z;
    static matrix_2x2_t design_matrix_z;
    static vector_2_t last_measurement_z;
    // Kalman parameters
    static float max_acc = 10.0f;
    static float delta_t = 0.0f;
    static uint32_t last_time_in_loop = 0;

    // Update time tracker & delta_t
    delta_t = (time_keeper_get_millis() - last_time_in_loop) / 1000.0f;
    last_time_in_loop = time_keeper_get_millis();

    // Initialise Kalman parameters once, and only once
    static bool kalman_init_done = FALSE;

    if(!kalman_init_done) {
        kalman_init(
            &state_estimate_x,
            &state_estimate_covariance_x,
            &process_noise_covariance_x,
            &design_matrix_x,
            max_acc,
            delta_t);
        kalman_init(
            &state_estimate_y,
            &state_estimate_covariance_y,
            &process_noise_covariance_y,
            &design_matrix_y,
            max_acc,
            delta_t);
        kalman_init(
            &state_estimate_z,
            &state_estimate_covariance_z,
            &process_noise_covariance_z,
            &design_matrix_z,
            max_acc,
            delta_t);
        kalman_init_done = TRUE;
    }

    // Kalman predictor loop
    kalman_predict(
        &state_estimate_x,
        &state_estimate_covariance_x,
        process_noise_covariance_x,
        delta_t);
    kalman_predict(
        &state_estimate_y,
        &state_estimate_covariance_y,
        process_noise_covariance_y,
        delta_t);
    kalman_predict(
        &state_estimate_z,
        &state_estimate_covariance_z,
        process_noise_covariance_z,
        delta_t);
    // Only correct the prediction if there is a new measurement
    if(track_following_new_message_received(track_following)) {
        // Get last waypoint data for x, y and z
        last_measurement_x.v[0] =
            track_following->neighbors->neighbors_list[0].position[0];
        last_measurement_x.v[1] =
            track_following->neighbors->neighbors_list[0].velocity[0];
        last_measurement_y.v[0] =
            track_following->neighbors->neighbors_list[0].position[1];
        last_measurement_y.v[1] =
            track_following->neighbors->neighbors_list[0].velocity[1];
        last_measurement_z.v[0] =
            track_following->neighbors->neighbors_list[0].position[2];
        last_measurement_z.v[1] =
            track_following->neighbors->neighbors_list[0].velocity[2];
        // Correct Kalman predictor with this new data
        kalman_correct(
            &state_estimate_x,
            &state_estimate_covariance_x,
            &last_measurement_x,
            design_matrix_x,
            track_following);
        kalman_correct(
            &state_estimate_y,
            &state_estimate_covariance_y,
            &last_measurement_y,
            design_matrix_y,
            track_following);
        kalman_correct(
            &state_estimate_z,
            &state_estimate_covariance_z,
            &last_measurement_z,
            design_matrix_z,
            track_following);
    }

    // Use Kalman prediction output as waypoint
    track_following->waypoint_handler->waypoint_following.pos[0] =
        state_estimate_x.v[0];
    track_following->waypoint_handler->waypoint_following.pos[1] =
        state_estimate_y.v[0];
    track_following->waypoint_handler->waypoint_following.pos[2] =
        state_estimate_z.v[0];
}

// Function to check if there is a new measurement received
bool track_following_new_message_received(track_following_t* track_following)
{
    // Flag to signal the disponibility of a new measurement
    bool new_measurement_received = FALSE;

    // Store the time when last measurement was received
    static uint32_t last_measurement_time = 0;

    // Check if a new measurement has been received & set flag accordingly
    if(track_following->neighbors->neighbors_list[0].time_msg_received != last_measurement_time) {
        new_measurement_received = TRUE;
        last_measurement_time = track_following->neighbors->neighbors_list[0].time_msg_received;
    }

    return new_measurement_received;
}

// CONTROL //

// Implement PID for the waypoint following
void track_following_WP_control_PID(track_following_t* track_following)
{
    float error = 0;
    float offset = 0;

    static pid_controller_t track_following_pid_x =
    {
        .p_gain = 5.0f,
        .clip_min = -100.0f,
        .clip_max = 100.0f,
        .integrator={
            .pregain = 0.1f,
            .postgain = 0.1f,
            .accumulator = 0.0f,
            .maths_clip = 20.0f,
            .leakiness = 0.0f
        },
        .differentiator={
            .gain = 0.1f,
            .previous = 0.0f,
            .LPF = 0.5f,
            .maths_clip = 5.0f
        },
        .output = 0.0f,
        .error = 0.0f,
        .last_update = 0.0f,
        .dt = 1,
        .soft_zone_width = 0.0f
    };

    static pid_controller_t track_following_pid_y =
    {
        .p_gain = 5.0f,
        .clip_min = -100.0f,
        .clip_max = 100.0f,
        .integrator={
            .pregain = 0.1f,
            .postgain = 0.1f,
            .accumulator = 0.0f,
            .maths_clip = 20.0f,
            .leakiness = 0.0f
        },
        .differentiator={
            .gain = 0.1f,
            .previous = 0.0f,
            .LPF = 0.5f,
            .maths_clip = 5.0f
        },
        .output = 0.0f,
        .error = 0.0f,
        .last_update = 0.0f,
        .dt = 1,
        .soft_zone_width = 0.0f
    };

    static pid_controller_t track_following_pid_z =
    {
        .p_gain = 5.0f,
        .clip_min = -100.0f,
        .clip_max = 100.0f,
        .integrator={
            .pregain = 0.1f,
            .postgain = 0.1f,
            .accumulator = 0.0f,
            .maths_clip = 20.0f,
            .leakiness = 0.0f
        },
        .differentiator={
            .gain = 0.1f,
            .previous = 0.0f,
            .LPF = 0.5f,
            .maths_clip = 5.0f
        },
        .output = 0.0f,
        .error = 0.0f,
        .last_update = 0.0f,
        .dt = 1,
        .soft_zone_width = 0.0f
    };

    if (track_following_pid_x.integrator.accumulator > 15.0f) {
        track_following_pid_x.integrator.accumulator = 0.0f;
    }
    if (track_following_pid_y.integrator.accumulator > 15.0f) {
        track_following_pid_y.integrator.accumulator = 0.0f;
    }
    if (track_following_pid_z.integrator.accumulator > 15.0f) {
        track_following_pid_z.integrator.accumulator = 0.0f;
    }

    // Apply PID on x
    int i = 0;
    error = track_following_WP_distance_XYZ(track_following, i);
    offset = pid_control_update(&track_following_pid_x, error);
    track_following->waypoint_handler->waypoint_following.pos[i] += offset;
    // Apply PID on y
    i = 1;
    error = track_following_WP_distance_XYZ(track_following, i);
    offset = pid_control_update(&track_following_pid_y, error);
    track_following->waypoint_handler->waypoint_following.pos[i] += offset;
    // Apply PID on z
    i = 2;
    error = track_following_WP_distance_XYZ(track_following, i);
    offset = pid_control_update(&track_following_pid_y, error);
    track_following->waypoint_handler->waypoint_following.pos[i] += offset;
}

// FUNCTIONS //

// time_last_WP_ms: Time since last waypoint was received
uint32_t track_following_WP_time_last_ms(track_following_t* track_following)
{
    uint32_t timeWP = track_following->neighbors->neighbors_list[0].time_msg_received; // Last waypoint time in ms
    uint32_t time_actual = time_keeper_get_millis(); // actual time in ms
    uint32_t time_offset = time_actual - timeWP; // time since last waypoint in ms

    return time_offset;
}

// calculate distance to waypoint according to an axis
float track_following_WP_distance_XYZ(track_following_t* track_following, int i)
{
    float distance = track_following->waypoint_handler->waypoint_following.pos[i]
                    - track_following->position_estimator->local_position.pos[i];
    return distance;
}

// EVALUATE //

void track_following_send_dist(const track_following_t* track_following, const mavlink_stream_t* mavlink_stream, mavlink_message_t* msg)
{
    mavlink_msg_named_value_float_pack(    mavlink_stream->sysid,
                                        mavlink_stream->compid,
                                        msg,
                                        time_keeper_get_millis(),
                                        "dist2follow",
                                        track_following->dist2following);
}
