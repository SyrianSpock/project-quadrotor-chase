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

    // Apply PID control on x & y
    track_following_WP_control_PID(track_following);

    // Insure follower stays 5m below target
    track_following->waypoint_handler->waypoint_following.pos[2] -= 5.0f;
}


// Handle the Kalman predictor
void track_following_kalman_predictor(track_following_t* track_following)
{
    // Kalman variables for x, y and z axis
    static kalman_handler_t kalman_handler_x, kalman_handler_y, kalman_handler_z;
    static vector_3_t last_measurement_x, last_measurement_y, last_measurement_z;
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
        kalman_init(&kalman_handler_x, max_acc, delta_t);
        kalman_init(&kalman_handler_y, max_acc, delta_t);
        kalman_init(&kalman_handler_z, max_acc, delta_t);
        kalman_init_done = TRUE;
    }

    /*
        Call the Kalman prediction loop
        The prediction loop runs at higher rate than correction
     */
    kalman_predict(&kalman_handler_x, max_acc, delta_t);
    kalman_predict(&kalman_handler_y, max_acc, delta_t);
    kalman_predict(&kalman_handler_z, max_acc, delta_t);

    // Only correct the prediction if there is a new measurement
    if(track_following_new_message_received(track_following)) {
        /*
            Use Bezier curve interpolation to get information about a previous
            point velocity in order to compute a more accurate acceleration
            on the x and y axis
        */
        vector_2_t p1, p2, p3, p4; // Control points for Bezier interpolation
        vector_2_t bp; // Bezier estimated velocity
        float t = 0.9f;

        // Control point 1 : previous measured waypoint
        p1.v[0] = last_measurement_x.v[0];
        p1.v[1] = last_measurement_y.v[0];
        // Control point 2 : prev. measured waypoint + velocity at that waypoint
        p2.v[0] = p1.v[0] + last_measurement_x.v[1];
        p2.v[1] = p1.v[1] + last_measurement_y.v[1];
        // Control point 3 : current measured waypoint
        p4.v[0] = track_following->neighbors->neighbors_list[0].position[0];
        p4.v[1] = track_following->neighbors->neighbors_list[0].position[1];
        // Control point 4 : previous measured waypoint
        p3.v[0] = p4.v[0] - track_following->neighbors->neighbors_list[0].velocity[0];
        p3.v[1] = p4.v[1] - track_following->neighbors->neighbors_list[0].velocity[1];

        // Compute estimated velocity according to Bezier interpolation
        bp.v[0] = 3 * (1 - t) * (1 - t) * (p2.v[0] - p1.v[0]) \
                  + 6 * (1 - t) * t * (p3.v[0] - p2.v[0]) \
                  + 3 * t * t * (p4.v[0] - p3.v[0]);
        bp.v[1] = 3 * (1 - t) * (1 - t) * (p2.v[1] - p1.v[1]) \
                  + 6 * (1 - t) * t * (p3.v[1] - p2.v[1]) \
                  + 3 * t * t * (p4.v[1] - p3.v[1]);
        bp.v[0] = bp.v[0] / 3.0f;
        bp.v[1] = bp.v[1] / 3.0f;

        /*
            Use Bezier estimated velocity to compute more accurate acceleration along x and y
         */
        last_measurement_x.v[2] =
            (track_following->neighbors->neighbors_list[0].velocity[0]
            - bp.v[0]) / 0.4f;
        last_measurement_y.v[2] =
            (track_following->neighbors->neighbors_list[0].velocity[1]
            - bp.v[1]) / 0.4f;

        /*
            Use less accurate estimate on acceleration along z using previous waypoint data
         */
        last_measurement_z.v[2] = (track_following->neighbors->neighbors_list[0].velocity[2] - last_measurement_z.v[1]) / 4.0f;

        // Get last waypoint position data for x, y and z
        last_measurement_x.v[0] =
            track_following->neighbors->neighbors_list[0].position[0];
        last_measurement_y.v[0] =
            track_following->neighbors->neighbors_list[0].position[1];
        last_measurement_z.v[0] =
            track_following->neighbors->neighbors_list[0].position[2];

        // Get last waypoint position data for x, y and z
        last_measurement_x.v[1] =
            track_following->neighbors->neighbors_list[0].velocity[0];
        last_measurement_y.v[1] =
            track_following->neighbors->neighbors_list[0].velocity[1];
        last_measurement_z.v[1] =
            track_following->neighbors->neighbors_list[0].velocity[2];

        // Correct Kalman predictor with this new data
        kalman_correct(&kalman_handler_x, &last_measurement_x, track_following);
        kalman_correct(&kalman_handler_y, &last_measurement_y, track_following);
        kalman_correct(&kalman_handler_z, &last_measurement_z, track_following);
    }

    // Use Kalman position prediction output as waypoint
    track_following->waypoint_handler->waypoint_following.pos[0] =
        kalman_handler_x.state_estimate.v[0];
    track_following->waypoint_handler->waypoint_following.pos[1] =
        kalman_handler_y.state_estimate.v[0];
    track_following->waypoint_handler->waypoint_following.pos[2] =
        kalman_handler_z.state_estimate.v[0];
}


// Check if there is a new measurement received
bool track_following_new_message_received(track_following_t* track_following)
{
    // Flag to signal the disponibility of a new measurement
    bool new_measurement_received = FALSE;

    // Store the time when last measurement was received
    static uint32_t last_measurement_time = 0;

    // Check if a new measurement has been received & set flag accordingly
    if(track_following->neighbors->neighbors_list[0].time_msg_received != last_measurement_time) {
        new_measurement_received = TRUE;
        last_measurement_time =
            track_following->neighbors->neighbors_list[0].time_msg_received;
    }

    return new_measurement_received;
}


// Implements PID on position for the waypoint following
void track_following_WP_control_PID(track_following_t* track_following)
{
    // Initialise control variables
    float error = 0;
    float offset = 0;

    // Initialise PID controller on position along x axis
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

    // Initialise PID controller on position along y axis
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

    // Add Antirewind (ARW) to empty the integrator accumulator
    if (track_following_pid_x.integrator.accumulator > 15.0f) {
        track_following_pid_x.integrator.accumulator = 0.0f;
    }
    if (track_following_pid_y.integrator.accumulator > 15.0f) {
        track_following_pid_y.integrator.accumulator = 0.0f;
    }

    // Apply PID on position along x axis
    int i = 0;
    error = track_following_WP_distance_XYZ(track_following, i);
    offset = pid_control_update(&track_following_pid_x, error);
    track_following->waypoint_handler->waypoint_following.pos[i] += offset;
    // Apply PID on position along y axis
    i = 1;
    error = track_following_WP_distance_XYZ(track_following, i);
    offset = pid_control_update(&track_following_pid_y, error);
    track_following->waypoint_handler->waypoint_following.pos[i] += offset;
}


// Get time since last waypoint was received
uint32_t track_following_WP_time_last_ms(track_following_t* track_following)
{
    uint32_t timeWP = track_following->neighbors->neighbors_list[0].time_msg_received; // Last waypoint time in ms
    uint32_t time_actual = time_keeper_get_millis(); // actual time in ms
    uint32_t time_offset = time_actual - timeWP; // time since last waypoint in ms

    return time_offset;
}


// Compute distance to waypoint according to an axis
float track_following_WP_distance_XYZ(track_following_t* track_following, int i)
{
    float distance = track_following->waypoint_handler->waypoint_following.pos[i]
                    - track_following->position_estimator->local_position.pos[i];
    return distance;
}


void track_following_send_dist(const track_following_t* track_following, const mavlink_stream_t* mavlink_stream, mavlink_message_t* msg)
{
    mavlink_msg_named_value_float_pack(    mavlink_stream->sysid,
                                        mavlink_stream->compid,
                                        msg,
                                        time_keeper_get_millis(),
                                        "dist2follow",
                                        track_following->dist2following);
}
