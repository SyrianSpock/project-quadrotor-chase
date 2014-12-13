#ifndef KALMAN_PREDICTOR_H
#define KALMAN_PREDICTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "small_matrix.h"
#include "track_following.h"

#define TRUE 1
#define FALSE 0
#define NULL 0

uint8_t kalman_init(
            vector_4_t * state_estimate,
            matrix_4x4_t * state_estimate_covariance,
            matrix_4x4_t * process_noise_covariance,
            matrix_4x4_t * design_matrix,
            const float max_acc,
            const float delta_t);

uint8_t kalman_predict(
            vector_4_t * state_estimate,
            matrix_4x4_t * state_estimate_covariance,
            const matrix_4x4_t process_noise_covariance,
            const float delta_t);

uint8_t kalman_correct(
            vector_4_t * state_estimate,
            matrix_4x4_t * state_estimate_covariance,
            vector_4_t * last_measurement,
            const matrix_4x4_t design_matrix,
            track_following_t* track_following);

uint8_t kalman_update_measurement(
            vector_4_t * measurement_residual,
            vector_4_t * last_measurement,
            const vector_4_t state_estimate,
            const matrix_4x4_t design_matrix,
			track_following_t * track_following);

uint8_t kalman_compute_gain(
            matrix_4x4_t * kalman_gain,
            const matrix_4x4_t design_matrix);


#ifdef __cplusplus
}
#endif

#endif
