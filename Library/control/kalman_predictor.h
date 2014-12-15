/*******************************************************************************
 * \file kalman_predictor.h
 *
 * \author Jonathan Arreguit
 * \author Dorian Konrad
 * \author Salah Missri
 * \author David Tauxe
 *
 * \brief This file implements a Kalman predictor
 *
 ******************************************************************************/

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
            vector_2_t * state_estimate,
            matrix_2x2_t * state_estimate_covariance,
            matrix_2x2_t * process_noise_covariance,
            matrix_2x2_t * design_matrix,
            const float max_acc,
            const float delta_t);

uint8_t kalman_predict(
            vector_2_t * state_estimate,
            matrix_2x2_t * state_estimate_covariance,
            const matrix_2x2_t process_noise_covariance,
            const float delta_t);

uint8_t kalman_correct(
            vector_2_t * state_estimate,
            matrix_2x2_t * state_estimate_covariance,
            vector_2_t * last_measurement,
            const matrix_2x2_t design_matrix,
            track_following_t* track_following);

uint8_t kalman_update_measurement_residual(
            vector_2_t * measurement_residual,
            vector_2_t * last_measurement,
            const vector_2_t state_estimate,
            const matrix_2x2_t design_matrix,
            track_following_t * track_following);

uint8_t kalman_compute_gain(
            matrix_2x2_t * kalman_gain,
            const matrix_2x2_t design_matrix);


#ifdef __cplusplus
}
#endif

#endif
