/*******************************************************************************
 * \file kalman_predictor.c
 *
 * \author Jonathan Arreguit
 * \author Dorian Konrad
 * \author Salah Missri
 * \author David Tauxe
 *
 * \brief This file implements a Kalman predictor
 *
 ******************************************************************************/

#include "small_matrix.h"
#include "linear_algebra.h"
#include "kalman_predictor.h"

uint8_t kalman_init(
            vector_4_t * state_estimate,
            matrix_4x4_t * state_estimate_covariance,
            matrix_4x4_t * process_noise_covariance,
            matrix_4x4_t * design_matrix,
            const float max_acc,
            const float delta_t)
{
    // Make sure the input is set as expected
    if(state_estimate == NULL \
        || state_estimate_covariance == NULL \
        || process_noise_covariance == NULL \
        || design_matrix == NULL) {
        return 0;
    }

	for(int i = 0; i < 4; i++) {
		state_estimate->v[i]= 0.0f;
	}
	*state_estimate_covariance = zero_4x4;

    float sigma_x = 0.015625 * max_acc*max_acc * delta_t*delta_t*delta_t*delta_t;
    float sigma_v = 0.0625 * max_acc*max_acc * delta_t*delta_t;
	vector_4_t process_noise_variance = {.v={sigma_x, sigma_x, sigma_v, sigma_v}};
	*process_noise_covariance = diag_4x4(process_noise_variance);

    *design_matrix = ident_4x4;

    return 1;
}

uint8_t kalman_predict(
            vector_4_t * state_estimate,
            matrix_4x4_t * state_estimate_covariance,
            const matrix_4x4_t process_noise_covariance,
            const float delta_t)
{
    // Make sure the input is set as expected
    if(state_estimate == NULL || state_estimate_covariance == NULL) {
        return 0;
    }

    matrix_4x4_t state_propagation_matrix =
       {.v={{1.0f, 0.0f, delta_t, 0.0f},
            {0.0f, 1.0f, 0.0f,    delta_t},
            {0.0f, 0.0f, 1.0f,    0.0f},
            {0.0f, 0.0f, 0.0f,    1.0f}} };
    matrix_4x4_t state_propagation_matrix_trans =
       {.v={{1.0f,    0.0f,    0.0f, 0.0f},
            {0.0f,    1.0f,    0.0f, 0.0f},
            {delta_t, 0.0f,    1.0f, 0.0f},
            {0.0f,    delta_t, 0.0f, 1.0f}} };

    // Predict state estimate
    *state_estimate = mvmul4(state_propagation_matrix, *state_estimate);

    // Predict state estimate covariance
    *state_estimate_covariance = mmul4(*state_estimate_covariance,
                                       state_propagation_matrix_trans);
    *state_estimate_covariance = mmul4(state_propagation_matrix,
                                       *state_estimate_covariance);
    *state_estimate_covariance = madd4(*state_estimate_covariance,
                                       process_noise_covariance);

    return 1;
}

uint8_t kalman_correct(
            vector_4_t * state_estimate,
            matrix_4x4_t * state_estimate_covariance,
            vector_4_t * last_measurement,
            const matrix_4x4_t design_matrix,
            track_following_t* track_following)
{
    // Make sure the input is set as expected
    if(state_estimate == NULL \
       || state_estimate_covariance == NULL \
       || last_measurement == NULL) {
        return 0;
    }

    vector_4_t measurement_residual;
    matrix_4x4_t kalman_gain;

    // Take new measurement into account
    kalman_update_measurement(
        &measurement_residual,
        last_measurement,
        *state_estimate,
        design_matrix,
		track_following);

    // Compute the new optimal Kalman gain
    kalman_compute_gain(
        &kalman_gain,
        design_matrix);

    // Correct state estimate according to new measurement
    *state_estimate = vadd4(*state_estimate,
                            mvmul4(kalman_gain, measurement_residual));

    // Correct state estimate covariance according to new measurement
    matrix_4x4_t temp_m = mmul4(kalman_gain, design_matrix);
    temp_m = msub4(ident_4x4, temp_m);
    *state_estimate_covariance = mmul4(temp_m, *state_estimate_covariance);

    return 1;
}

uint8_t kalman_update_measurement(
            vector_4_t * measurement_residual,
            vector_4_t * last_measurement,
            const vector_4_t state_estimate,
            const matrix_4x4_t design_matrix,
			track_following_t * track_following)
{
    // Make sure the input is set as expected
    if(measurement_residual == NULL || last_measurement == NULL) {
        return 0;
    }

    // Get last waypoint data
    last_measurement->v[0] =
        track_following->neighbors->neighbors_list[0].position[0];
    last_measurement->v[1] =
        track_following->neighbors->neighbors_list[0].position[1];
    last_measurement->v[2] =
        track_following->neighbors->neighbors_list[0].velocity[0];
    last_measurement->v[3] =
        track_following->neighbors->neighbors_list[0].velocity[1];

    // Compute measurement residual according to new measurement
    *measurement_residual = vsub4(*last_measurement,
                                 mvmul4(design_matrix, state_estimate));

    return 1;
}

uint8_t kalman_compute_gain(
            matrix_4x4_t * kalman_gain,
            const matrix_4x4_t design_matrix)
{
    // Make sure the input is set as expected
    if(kalman_gain == NULL) {
        return 0;
    }

    // Simplified computation to maximise computational efficiency
    *kalman_gain = inv4(design_matrix);

    return 1;
}
