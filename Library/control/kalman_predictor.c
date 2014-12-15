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
            vector_2_t * state_estimate,
            matrix_2x2_t * state_estimate_covariance,
            matrix_2x2_t * process_noise_covariance,
            matrix_2x2_t * design_matrix,
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

    for(int i = 0; i < 2; i++) {
        state_estimate->v[i]= 0.0f;
    }
    *state_estimate_covariance = zero_2x2;

    float sigma_x = (1.0f / 8.0f) * max_acc * delta_t * delta_t;
    float sigma_v = (1.0f / 4.0f) * max_acc * delta_t;
    process_noise_covariance->v[0][0] = sigma_x * sigma_x;
    process_noise_covariance->v[0][1] = sigma_x * sigma_v;
    process_noise_covariance->v[1][0] = sigma_x * sigma_v;
    process_noise_covariance->v[1][1] = sigma_v * sigma_v;

    *design_matrix = ident_2x2;

    return 1;
}

uint8_t kalman_predict(
            vector_2_t * state_estimate,
            matrix_2x2_t * state_estimate_covariance,
            const matrix_2x2_t process_noise_covariance,
            const float delta_t)
{
    // Make sure the input is set as expected
    if(state_estimate == NULL || state_estimate_covariance == NULL) {
        return 0;
    }

    matrix_2x2_t state_propagation_matrix =
       {.v={{1.0f, delta_t},
            {0.0f, 1.0f}} };
    matrix_2x2_t state_propagation_matrix_trans =
       {.v={{1.0f,    0.0f},
            {delta_t, 1.0f}} };

    // Predict state estimate
    *state_estimate = mvmul2(state_propagation_matrix, *state_estimate);

    // Predict state estimate covariance
    *state_estimate_covariance = mmul2(*state_estimate_covariance,
                                       state_propagation_matrix_trans);
    *state_estimate_covariance = mmul2(state_propagation_matrix,
                                       *state_estimate_covariance);
    *state_estimate_covariance = madd2(*state_estimate_covariance,
                                       process_noise_covariance);

    return 1;
}

uint8_t kalman_correct(
            vector_2_t * state_estimate,
            matrix_2x2_t * state_estimate_covariance,
            vector_2_t * last_measurement,
            const matrix_2x2_t measurement_covariance,
            const matrix_2x2_t design_matrix,
            track_following_t* track_following)
{
    // Make sure the input is set as expected
    if(state_estimate == NULL \
       || state_estimate_covariance == NULL \
       || last_measurement == NULL) {
        return 0;
    }

    vector_2_t measurement_residual;
    matrix_2x2_t kalman_gain;

    // Take new measurement into account
    kalman_update_measurement_residual(
        &measurement_residual,
        last_measurement,
        *state_estimate,
        design_matrix,
        track_following);

    // Compute the new optimal Kalman gain
    kalman_compute_gain(
        &kalman_gain,
        state_estimate_covariance,
        measurement_covariance,
        design_matrix);

    // Correct state estimate according to new measurement
    *state_estimate = vadd2(*state_estimate,
                            mvmul2(kalman_gain, measurement_residual));

    // Correct state estimate covariance according to new measurement
    *state_estimate_covariance =
        mmul2(msub2(ident_2x2, mmul2(kalman_gain, design_matrix)),
              *state_estimate_covariance);

    return 1;
}

uint8_t kalman_update_measurement_residual(
            vector_2_t * measurement_residual,
            vector_2_t * last_measurement,
            const vector_2_t state_estimate,
            const matrix_2x2_t design_matrix,
            track_following_t * track_following)
{
    // Make sure the input is set as expected
    if(measurement_residual == NULL || last_measurement == NULL) {
        return 0;
    }

    // Compute measurement residual according to new measurement
    *measurement_residual = vsub2(*last_measurement,
                                 mvmul2(design_matrix, state_estimate));

    return 1;
}

uint8_t kalman_compute_gain(
            matrix_2x2_t * kalman_gain,
            matrix_2x2_t * state_estimate_covariance,
            const matrix_2x2_t measurement_covariance,
            const matrix_2x2_t design_matrix)
{
    // Make sure the input is set as expected
    if(kalman_gain == NULL || state_estimate_covariance == NULL) {
        return 0;
    }

    // As H = Identity, we can simplify the gain computation
    *kalman_gain = mmul2(*state_estimate_covariance,
                         inv2(madd2(*state_estimate_covariance,
                                    measurement_covariance)));

    return 1;
}
