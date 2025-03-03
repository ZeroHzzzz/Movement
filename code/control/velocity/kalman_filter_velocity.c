/*
 * kalman_filter_velocity.c
 *
 *  Created on: 2024年6月7日
 *      Author: symc
 *     Description: kalman filter for velocity
 *    Reference: https://en.wikipedia.org/wiki/Kalman_filter;
 *              https://blog.csdn.net/weixin_43942325/article/details/103416681;
 *    Note: large Q means large process noise which makes the system convergence
 * fast, trust the measurement more; large R means large measurement noise which
 * makes the system convergence slow but more stable.
 */

#include "kalman_filter_velocity.h"
#include "Ifx_LutAtan2F32.h"
#include "Ifx_LutLSincosF32.h"

static void mat_add(float a[STATE_SIZE][STATE_SIZE],
                    float b[STATE_SIZE][STATE_SIZE],
                    float c[STATE_SIZE][STATE_SIZE]);
static void mat_sub(float a[STATE_SIZE][STATE_SIZE],
                    float b[STATE_SIZE][STATE_SIZE],
                    float c[STATE_SIZE][STATE_SIZE]);
static void mat_mult(float a[STATE_SIZE][STATE_SIZE],
                     float b[STATE_SIZE][STATE_SIZE],
                     float c[STATE_SIZE][STATE_SIZE]);
static void mat_mult_vec(float a[STATE_SIZE][STATE_SIZE],
                         float b[STATE_SIZE],
                         float c[STATE_SIZE]);
static void mat_transpose(float a[STATE_SIZE][STATE_SIZE],
                          float b[STATE_SIZE][STATE_SIZE]);
static void vec_add(float a[STATE_SIZE],
                    float b[STATE_SIZE],
                    float c[STATE_SIZE]);
static void vec_sub(float a[STATE_SIZE],
                    float b[STATE_SIZE],
                    float c[STATE_SIZE]);
// static void vec_scalar_mult(float a[STATE_SIZE], float b, float
// c[STATE_SIZE]);
static void mat_inv(float a[STATE_SIZE][STATE_SIZE],
                    float b[STATE_SIZE][STATE_SIZE]);

void kalman_filter_velocity_init(kalman_filter_velocity_t* kf) {
    // Initialize the state vector
    /**
     * x = [0;
     *     0]
     */
    kf->x[0] = 0;
    kf->x[1] = 0;

    // Initialize the state covariance matrix
    /**
     * P = [1, 0;
     *     0, 1]
     */
    kf->P[0][0] = 1.0f;
    kf->P[0][1] = 0;
    kf->P[1][0] = 0;
    kf->P[1][1] = 1.0f;

    // Initialize the state transition matrix
    /**
     * A = [1, dt;
     *     0, 1]
     */
    float dt = 0.01f;
    kf->A[0][0] = 1.0f;
    kf->A[0][1] = dt;
    kf->A[1][0] = 0;
    kf->A[1][1] = 1.0f;

    // Initialize the control matrix (not used)
    kf->B[0] = 0;
    kf->B[1] = 0;

    // Initialize the measurement matrix
    /**
     * H = [1, 0;
     *     0, 1]
     */
    kf->H[0][0] = 1.0f;
    kf->H[0][1] = 0;
    kf->H[1][0] = 0;
    kf->H[1][1] = 1.0f;

    // Initialize the process noise covariance matrix
    /**
     * Q = [0.1, 0;
     *     0, 0.1]
     */
    kf->Q[0][0] = 0.1f;
    kf->Q[0][1] = 0;
    kf->Q[1][0] = 0;
    kf->Q[1][1] = 0.1f;

    // Initialize the measurement noise covariance matrix
    /**
     * R = [1.1, 0;
     *     0, 0.01]
     */
    kf->R[0][0] = 1.1f;
    kf->R[0][1] = 0;
    kf->R[1][0] = 0;
    kf->R[1][1] = 0.01f;

    // Initialize the control vector (not used)
    kf->u[0] = 0;
}
void kalman_filter_velocity_predict(kalman_filter_velocity_t* kf) {
    // Predict the state vector
    /**
     * x = A * x + B * u
     */
    float x_temp[STATE_SIZE];
    mat_mult_vec(kf->A, kf->x, x_temp);
    vec_add(x_temp, kf->B, kf->x);  // B * u = 0

    // Predict the state covariance matrix
    /**
     * P = A * P * A' + Q
     */
    float AP_temp[STATE_SIZE][STATE_SIZE];
    mat_mult(kf->A, kf->P, AP_temp);
    float At[STATE_SIZE][STATE_SIZE];
    mat_transpose(kf->A, At);
    float APAt[STATE_SIZE][STATE_SIZE];
    mat_mult(AP_temp, At, APAt);
    mat_add(APAt, kf->Q, kf->P);
}
void kalman_filter_velocity_update(kalman_filter_velocity_t* kf,
                                   float z[MEASUREMENT_SIZE]) {
    // Calculate the measurement residual
    /**
     * y = z - H * x
     */
    float Hx[MEASUREMENT_SIZE];
    mat_mult_vec(kf->H, kf->x, Hx);
    float y[MEASUREMENT_SIZE];
    vec_sub(z, Hx, y);

    // Calculate the residual covariance
    /**
     * S = H * P * H' + R
     */
    float HP_temp[MEASUREMENT_SIZE][STATE_SIZE];
    mat_mult(kf->H, kf->P, HP_temp);
    float Ht[STATE_SIZE][MEASUREMENT_SIZE];
    mat_transpose(kf->H, Ht);
    float HPHt[MEASUREMENT_SIZE][MEASUREMENT_SIZE];
    mat_mult(HP_temp, Ht, HPHt);
    float S[MEASUREMENT_SIZE][MEASUREMENT_SIZE];  // residual covariance
    mat_add(HPHt, kf->R, S);

    // Calculate the Kalman gain
    /**
     * K = P * H' * S^-1
     */
    float S_inv[MEASUREMENT_SIZE][MEASUREMENT_SIZE];
    mat_inv(S, S_inv);
    float PHt[STATE_SIZE][MEASUREMENT_SIZE];
    mat_mult(kf->P, Ht, PHt);
    float K[STATE_SIZE][MEASUREMENT_SIZE];  // Kalman gain
    mat_mult(PHt, S_inv, K);

    // Update the state vector
    /**
     * x = x + K * y
     */
    float Ky[STATE_SIZE];
    mat_mult_vec(K, y, Ky);
    vec_add(kf->x, Ky, kf->x);

    // Update the state covariance matrix
    /**
     * P = (I - K * H) * P
     */
    float KH[STATE_SIZE][STATE_SIZE];
    mat_mult(K, kf->H, KH);
    float I[STATE_SIZE][STATE_SIZE] = {{1, 0}, {0, 1}};
    float IKH[STATE_SIZE][STATE_SIZE];
    mat_sub(I, KH, IKH);
    float P_temp[STATE_SIZE][STATE_SIZE];
    mat_mult(IKH, kf->P, P_temp);

    // Copy P_temp to P
    for (uint8_t i = 0; i < STATE_SIZE; i++) {
        for (uint8_t j = 0; j < STATE_SIZE; j++) {
            kf->P[i][j] = P_temp[i][j];
        }
    }
}
static void mat_add(float a[STATE_SIZE][STATE_SIZE],
                    float b[STATE_SIZE][STATE_SIZE],
                    float c[STATE_SIZE][STATE_SIZE]) {
    for (uint8_t i = 0; i < STATE_SIZE; i++) {
        for (uint8_t j = 0; j < STATE_SIZE; j++) {
            c[i][j] = a[i][j] + b[i][j];
        }
    }
}
static void mat_sub(float a[STATE_SIZE][STATE_SIZE],
                    float b[STATE_SIZE][STATE_SIZE],
                    float c[STATE_SIZE][STATE_SIZE]) {
    for (uint8_t i = 0; i < STATE_SIZE; i++) {
        for (uint8_t j = 0; j < STATE_SIZE; j++) {
            c[i][j] = a[i][j] - b[i][j];
        }
    }
}
static void mat_mult(float a[STATE_SIZE][STATE_SIZE],
                     float b[STATE_SIZE][STATE_SIZE],
                     float c[STATE_SIZE][STATE_SIZE]) {
    for (uint8_t i = 0; i < STATE_SIZE; i++) {
        for (uint8_t j = 0; j < STATE_SIZE; j++) {
            c[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j];
        }
    }
}
static void mat_mult_vec(float a[STATE_SIZE][STATE_SIZE],
                         float b[STATE_SIZE],
                         float c[STATE_SIZE]) {
    for (uint8_t i = 0; i < STATE_SIZE; i++) {
        c[i] = a[i][0] * b[0] + a[i][1] * b[1];
    }
}
static void mat_transpose(float a[STATE_SIZE][STATE_SIZE],
                          float b[STATE_SIZE][STATE_SIZE]) {
    for (uint8_t i = 0; i < STATE_SIZE; i++) {
        for (uint8_t j = 0; j < STATE_SIZE; j++) {
            b[j][i] = a[i][j];
        }
    }
}
static void vec_add(float a[STATE_SIZE],
                    float b[STATE_SIZE],
                    float c[STATE_SIZE]) {
    for (uint8_t i = 0; i < STATE_SIZE; i++) {
        c[i] = a[i] + b[i];
    }
}
static void vec_sub(float a[STATE_SIZE],
                    float b[STATE_SIZE],
                    float c[STATE_SIZE]) {
    for (uint8_t i = 0; i < STATE_SIZE; i++) {
        c[i] = a[i] - b[i];
    }
}
// static void vec_scalar_mult(float a[STATE_SIZE], float b, float
// c[STATE_SIZE]){
//     for(uint8_t i = 0; i < STATE_SIZE; i++){
//         c[i] = a[i] * b;
//     }
// }
static void mat_inv(float a[STATE_SIZE][STATE_SIZE],
                    float b[STATE_SIZE][STATE_SIZE]) {
    // only for 2x2 matrix
    float det = a[0][0] * a[1][1] - a[0][1] * a[1][0];
    if (det == 0) {
        return;
    }
    float inv_det = 1.0f / det;
    b[0][0] = a[1][1] * inv_det;
    b[0][1] = -a[0][1] * inv_det;
    b[1][0] = -a[1][0] * inv_det;
    b[1][1] = a[0][0] * inv_det;
}
