/*
 * kalman_filter_velocity.h
 *
 *  Created on: 2024年6月7日
 *      Author: symc
 */

#ifndef CODE_CONTROL_VELOCITY_KALMAN_FILTER_VELOCITY_H_
#define CODE_CONTROL_VELOCITY_KALMAN_FILTER_VELOCITY_H_

#define STATE_SIZE 2
#define MEASUREMENT_SIZE 2

typedef struct {
    float x[STATE_SIZE]; // state vector
    float P[STATE_SIZE][STATE_SIZE]; // state covariance matrix
    float A[STATE_SIZE][STATE_SIZE]; // state transition matrix
    float B[STATE_SIZE]; // control matrix
    float H[MEASUREMENT_SIZE][STATE_SIZE]; // measurement matrix
    float Q[STATE_SIZE][STATE_SIZE]; // process noise covariance matrix
    float R[MEASUREMENT_SIZE][MEASUREMENT_SIZE]; // measurement noise covariance matrix
    float u[1]; // control vector
} kalman_filter_velocity_t;

void kalman_filter_velocity_init(kalman_filter_velocity_t *kf);
void kalman_filter_velocity_predict(kalman_filter_velocity_t *kf);
void kalman_filter_velocity_update(kalman_filter_velocity_t *kf, float z[MEASUREMENT_SIZE]);

#endif /* CODE_CONTROL_VELOCITY_KALMAN_FILTER_VELOCITY_H_ */
