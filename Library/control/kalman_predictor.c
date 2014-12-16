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


// Initialise the Kalman parameters
uint8_t kalman_init(
            kalman_handler_t * kalman_handler,
            float max_acc,
            float delta_t)
{
    // Make sure the input is set as expected
    if(kalman_handler == NULL) {
        return 0;
    }

    // Initialise state estimate and associated covariance matrix
    for(int i = 0; i < 3; i++) {
        kalman_handler->state_estimate.v[i]= 0.0f;
    }
    kalman_handler->state_estimate_covariance = zero_3x3;

    // Initialise Design matrix
    kalman_handler->design_matrix = ident_3x3;
    kalman_handler->design_matrix.v[2][1] = 1.0f;
    kalman_handler->design_matrix.v[2][2] = - 0.4f;

    // Initialise measurement covariance matrix
    kalman_handler->measurement_covariance = zero_3x3;
    kalman_handler->measurement_covariance.v[2][2] = 0.1f;

    return 1;
}


// Executes the prediction steps of the Kalman predictor
uint8_t kalman_predict(
            kalman_handler_t * kalman_handler,
            float max_acc,
            float delta_t)
{
    // Make sure the input is set as expected
    if(kalman_handler == NULL) {
        return 0;
    }

    // Compute new state propagation matrix
    matrix_3x3_t state_propagation_matrix =
       {.v={{1.0f, delta_t, delta_t * delta_t / 2.0f},
            {0.0f, 1.0f,    delta_t},
            {0.0f, 0.0f,    1.0f}} };
    matrix_3x3_t state_propagation_matrix_trans =
       {.v={{1.0f,                     0.0f,    0.0f},
            {delta_t,                  1.0f,    0.0f},
            {delta_t * delta_t / 2.0f, delta_t, 1.0f}} };

    // Compute new process noise covariance matrix
    matrix_3x3_t process_noise_covariance = zero_3x3;
    float sigma_x = (1.0f / 8.0f) * max_acc * delta_t * delta_t;
    float sigma_v = (1.0f / 4.0f) * max_acc * delta_t;
    process_noise_covariance.v[0][0] = sigma_x * sigma_x;
    process_noise_covariance.v[0][1] = sigma_x * sigma_v;
    process_noise_covariance.v[1][0] = sigma_x * sigma_v;
    process_noise_covariance.v[1][1] = sigma_v * sigma_v;

    // Predict state estimate, x_{k} = F * x_{k-1}
    kalman_handler->state_estimate =
        mvmul3(state_propagation_matrix, kalman_handler->state_estimate);

    // Predict state estimate covariance, P_{k} = F * P_{k-1} * F^T + Q
    kalman_handler->state_estimate_covariance =
        mmul3(kalman_handler->state_estimate_covariance,
              state_propagation_matrix_trans);
    kalman_handler->state_estimate_covariance =
        mmul3(state_propagation_matrix,
              kalman_handler->state_estimate_covariance);
    kalman_handler->state_estimate_covariance =
        madd3(kalman_handler->state_estimate_covariance,
              process_noise_covariance);

    return 1;
}


// Executes the correction (ie update) steps of the Kalman predictor
uint8_t kalman_correct(
            kalman_handler_t * kalman_handler,
            vector_3_t * last_measurement,
            track_following_t* track_following)
{
    // Make sure the input is set as expected
    if(kalman_handler == NULL || last_measurement == NULL) {
        return 0;
    }

    vector_3_t measurement_residual;
    matrix_3x3_t kalman_gain;

    // Take new measurement into account
    kalman_update_measurement_residual(
        kalman_handler,
        &measurement_residual,
        last_measurement,
        track_following);

    // Compute the new optimal Kalman gain
    kalman_compute_gain(
        kalman_handler,
        &kalman_gain);

    // Correct state estimate, x = x + K * y
    kalman_handler->state_estimate =
        vadd3(kalman_handler->state_estimate,
              mvmul3(kalman_gain, measurement_residual));

    // Correct state estimate covariance, P = (I − K * H) * P
    kalman_handler->state_estimate_covariance =
        mmul3(
            msub3(ident_3x3, mmul3(kalman_gain, kalman_handler->design_matrix)),
            kalman_handler->state_estimate_covariance);

    return 1;
}


// Computes the new measurement residual using the new measurement data
uint8_t kalman_update_measurement_residual(
            kalman_handler_t * kalman_handler,
            vector_3_t * measurement_residual,
            vector_3_t * last_measurement,
            track_following_t * track_following)
{
    // Make sure the input is set as expected
    if(kalman_handler == NULL || last_measurement == NULL) {
        return 0;
    }

    // Compute measurement residual, y = z - H * x
    *measurement_residual =
        vsub3(*last_measurement, mvmul3(kalman_handler->design_matrix,
                                        kalman_handler->state_estimate));

    return 1;
}


// Computes the new measurement residual using the new measurement data
uint8_t kalman_compute_gain(
            kalman_handler_t * kalman_handler,
            matrix_3x3_t * kalman_gain)
{
    // Make sure the input is set as expected
    if(kalman_handler == NULL || kalman_gain == NULL) {
        return 0;
    }

    matrix_3x3_t design_matrix_trans = trans3(kalman_handler->design_matrix);

    // Compute the residual covariance matrix, S = H * P * H^T + R
    matrix_3x3_t residual_covariance =
        madd3(
            mmul3(kalman_handler->design_matrix,
                  mmul3(kalman_handler->state_estimate_covariance,
                        design_matrix_trans)),
            kalman_handler->measurement_covariance);

    // Compute the Kalman gain matrix, K = P * H^T * S^-1
    *kalman_gain =
        mmul3(kalman_handler->state_estimate_covariance,
              mmul3(design_matrix_trans, inv3(residual_covariance)));

    return 1;
}
