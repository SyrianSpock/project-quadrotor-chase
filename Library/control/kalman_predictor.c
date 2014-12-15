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
            kalman_handler_t * kalman_handler,
            vector_2_t * measurement_variance,
            float max_acc,
            float delta_t)
{
    // Make sure the input is set as expected
    if(kalman_handler == NULL) {
        return 0;
    }

    for(int i = 0; i < 2; i++) {
        kalman_handler->state_estimate->v[i]= 0.0f;
    }
    *(kalman_handler->state_estimate_covariance) = zero_2x2;

    float sigma_x = (1.0f / 8.0f) * max_acc * delta_t * delta_t;
    float sigma_v = (1.0f / 4.0f) * max_acc * delta_t;
    kalman_handler->process_noise_covariance->v[0][0] = sigma_x * sigma_x;
    kalman_handler->process_noise_covariance->v[0][1] = sigma_x * sigma_v;
    kalman_handler->process_noise_covariance->v[1][0] = sigma_x * sigma_v;
    kalman_handler->process_noise_covariance->v[1][1] = sigma_v * sigma_v;

    *(kalman_handler->design_matrix) = ident_2x2;

    sigma_x = measurement_variance->v[0];
    sigma_v = measurement_variance->v[1];
    kalman_handler->measurement_covariance->v[0][0] = sigma_x * sigma_x;
    kalman_handler->measurement_covariance->v[0][1] = sigma_x * sigma_v;
    kalman_handler->measurement_covariance->v[1][0] = sigma_x * sigma_v;
    kalman_handler->measurement_covariance->v[1][1] = sigma_v * sigma_v;

    return 1;
}

uint8_t kalman_predict(
            kalman_handler_t * kalman_handler,
            float delta_t)
{
    // Make sure the input is set as expected
    if(kalman_handler == NULL) {
        return 0;
    }

    matrix_2x2_t state_propagation_matrix =
       {.v={{1.0f, delta_t},
            {0.0f, 1.0f}} };
    matrix_2x2_t state_propagation_matrix_trans =
       {.v={{1.0f,    0.0f},
            {delta_t, 1.0f}} };

    // Predict state estimate
    *(kalman_handler->state_estimate) =
        mvmul2(state_propagation_matrix, *(kalman_handler->state_estimate));

    // Predict state estimate covariance
    *(kalman_handler->state_estimate_covariance) =
        mmul2(*(kalman_handler->state_estimate_covariance),
              state_propagation_matrix_trans);
    *(kalman_handler->state_estimate_covariance) =
        mmul2(state_propagation_matrix,
              *(kalman_handler->state_estimate_covariance));
    *(kalman_handler->state_estimate_covariance) =
        madd2(*(kalman_handler->state_estimate_covariance),
              *(kalman_handler->process_noise_covariance));

    return 1;
}

uint8_t kalman_correct(
            kalman_handler_t * kalman_handler,
            vector_2_t * last_measurement,
            track_following_t* track_following)
{
    // Make sure the input is set as expected
    if(kalman_handler == NULL || last_measurement == NULL) {
        return 0;
    }

    vector_2_t measurement_residual;
    matrix_2x2_t kalman_gain;

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

    // Correct state estimate according to new measurement
    *(kalman_handler->state_estimate) =
        vadd2(*(kalman_handler->state_estimate),
              mvmul2(kalman_gain, measurement_residual));

    // Correct state estimate covariance according to new measurement
    *kalman_handler->state_estimate_covariance =
        mmul2(
            msub2(ident_2x2, mmul2(kalman_gain,
                                   *(kalman_handler->design_matrix))),
            *(kalman_handler->state_estimate_covariance));

    return 1;
}

uint8_t kalman_update_measurement_residual(
            kalman_handler_t * kalman_handler,
            vector_2_t * measurement_residual,
            vector_2_t * last_measurement,
            track_following_t * track_following)
{
    // Make sure the input is set as expected
    if(kalman_handler == NULL || last_measurement == NULL) {
        return 0;
    }

    // Compute measurement residual according to new measurement
    *measurement_residual =
        vsub2(*last_measurement, mvmul2(*(kalman_handler->design_matrix),
                                        *(kalman_handler->state_estimate)));

    return 1;
}

uint8_t kalman_compute_gain(
            kalman_handler_t * kalman_handler,
            matrix_2x2_t * kalman_gain)
{
    // Make sure the input is set as expected
    if(kalman_handler == NULL || kalman_gain == NULL) {
        return 0;
    }

    // As H = Identity, we can simplify the gain computation
    *kalman_gain = mmul2(*(kalman_handler->state_estimate_covariance),
                         inv2(madd2(*(kalman_handler->state_estimate_covariance),
                                    *(kalman_handler->measurement_covariance))));

    return 1;
}
