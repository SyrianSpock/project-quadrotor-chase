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

typedef struct kalman_handler_t {
    vector_2_t * state_estimate;
    matrix_2x2_t * state_estimate_covariance;
    matrix_2x2_t * design_matrix;
    matrix_2x2_t * measurement_covariance;
} kalman_handler_t;

uint8_t kalman_init(
            kalman_handler_t * kalman_handler,
            vector_2_t * measurement_variance,
            float max_acc,
            float delta_t);

uint8_t kalman_predict(
            kalman_handler_t * kalman_handler,
            float max_acc,
            float delta_t);

uint8_t kalman_correct(
            kalman_handler_t * kalman_handler,
            vector_2_t * last_measurement,
            track_following_t* track_following);

uint8_t kalman_update_measurement_residual(
            kalman_handler_t * kalman_handler,
            vector_2_t * measurement_residual,
            vector_2_t * last_measurement,
            track_following_t * track_following);

uint8_t kalman_compute_gain(
            kalman_handler_t * kalman_handler,
            matrix_2x2_t * kalman_gain);


#ifdef __cplusplus
}
#endif

#endif
