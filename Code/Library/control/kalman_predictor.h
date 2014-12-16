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

/**
 * \brief   Structure that contains all the main Kalman parameters
 *
 * \param   state_estimate                State estimate vector, contains
 *                                          (position, velocity, acceleration)
 * \param   state_estimate_covariance     State estimate covariance matrix
 * \param   design_matrix                 Design matrix, maps the state space
 *                                          to the measurement space
 * \param   measurement_covariance        Measurement covariance matrix that
 *                                          takes measurement errors into
 *                                          account
 * \param   track_following               Pointer to the structure of the
 *                                          track following
 */
typedef struct kalman_handler_t {
    vector_3_t state_estimate;
    matrix_3x3_t state_estimate_covariance;
    matrix_3x3_t design_matrix;
    matrix_3x3_t measurement_covariance;
} kalman_handler_t;


/**
 * \brief   Initialise the Kalman parameters
 *
 * \param   kalman_handler          Pointer to the structure that contains all
 *                                    the main Kalman parameters
 * \param   max_acc                 Value of the maximal acceleration that the
 *                                    system can exert
 * \param   delta_t                 Integration time
 */
uint8_t kalman_init(
            kalman_handler_t * kalman_handler,
            float max_acc,
            float delta_t);


/**
 * \brief   Executes the prediction steps of the Kalman predictor
 *            (predicts the state & the state covariance using a constant
 *             acceleration motion model)
 *
 * \param   kalman_handler          Pointer to the structure that contains all
 *                                    the main Kalman parameters
 * \param   max_acc                 Value of the maximal acceleration that the
 *                                    system can exert
 * \param   delta_t                 Integration time
 */
uint8_t kalman_predict(
            kalman_handler_t * kalman_handler,
            float max_acc,
            float delta_t);


/**
 * \brief   Executes the correction (ie update) steps of the Kalman predictor
 *            (corrects the state & the state covariance using the new
 *             measurement data received)
 *
 * \param   kalman_handler          Pointer  to the structure that contains all
 *                                    the main Kalman parameters
 * \param   last_measurement        Pointer to the last measurement data received
 * \param   track_following         Pointer to the structure of the
 *                                    track following
 */
uint8_t kalman_correct(
            kalman_handler_t * kalman_handler,
            vector_3_t * last_measurement,
            track_following_t* track_following);


/**
 * \brief   Computes the new measurement residual using the new measurement data
 *
 * \param   kalman_handler          Pointer to the structure that contains all
 *                                    the main Kalman parameters
 * \param   measurement_residual    Pointer to the measurement residual vector
 * \param   last_measurement        Pointer to the last measurement data received
 * \param   track_following         Pointer to the structure of the
 *                                    track following
 */
uint8_t kalman_update_measurement_residual(
            kalman_handler_t * kalman_handler,
            vector_3_t * measurement_residual,
            vector_3_t * last_measurement,
            track_following_t * track_following);


/**
 * \brief   Computes the new Kalman gain
 *
 * \param   kalman_handler          Pointer to the structure that contains all
 *                                    the main Kalman parameters
 * \param   kalman_gain             Pointer to the Kalman gain matrix
 */
uint8_t kalman_compute_gain(
            kalman_handler_t * kalman_handler,
            matrix_3x3_t * kalman_gain);


#ifdef __cplusplus
}
#endif

#endif
