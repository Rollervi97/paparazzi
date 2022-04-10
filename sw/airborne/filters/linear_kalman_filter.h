/*
 * Copyright (C) 2021 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "filters/linear_kalman_filter.c"
 *
 * Generic discrete Linear Kalman Filter
 */

#ifndef LINEAR_KALMAN_FILTER_H
#define LINEAR_KALMAN_FILTER_H

#include "std.h"
#include "math/pprz_algebra_float.h"
#include "stdio.h"

// maximum size for the state vector
#ifndef KF_MAX_STATE_SIZE
#define KF_MAX_STATE_SIZE 6
#endif

// maximum size for the command vector
#ifndef KF_MAX_CMD_SIZE
#define KF_MAX_CMD_SIZE 2
#endif

// maximum size for the measurement vector
#ifndef KF_MAX_MEAS_SIZE
#define KF_MAX_MEAS_SIZE 6
#endif

struct linear_kalman_filter {
  // filled by user after calling init function
  float A[KF_MAX_STATE_SIZE][KF_MAX_STATE_SIZE];  ///< dynamic matrix
  float B[KF_MAX_STATE_SIZE][KF_MAX_CMD_SIZE];    ///< command matrix
  float C[KF_MAX_MEAS_SIZE][KF_MAX_STATE_SIZE];   ///< observation matrix
  float D[KF_MAX_MEAS_SIZE][KF_MAX_CMD_SIZE];     ///< feedthrough matrix
  float P[KF_MAX_STATE_SIZE][KF_MAX_STATE_SIZE];  ///< state covariance matrix
  float Q[KF_MAX_STATE_SIZE][KF_MAX_STATE_SIZE];  ///< proces covariance noise
  float R[KF_MAX_MEAS_SIZE][KF_MAX_MEAS_SIZE];    ///< measurement covariance noise
  // float H[KF_MAX_MEAS_SIZE][KF_MAX_STATE_SIZE];   ///< feedthrough process noise matrix


  float X[KF_MAX_STATE_SIZE];   ///< estimated state X
  float Y[KF_MAX_MEAS_SIZE];    ///< estimated measurement

  uint8_t n;  ///< state vector size (<= KF_MAX_STATE_SIZE)
  uint8_t c;  ///< command vector size (<= KF_MAX_CMD_SIZE)
  uint8_t m;  ///< measurement vector size (<= KF_MAX_MEAS_SIZE)
};

/** Init all matrix and vectors to zero
 *
 * @param filter pointer to a filter structure
 * @param n size of the state vector
 * @param c size of the command vector
 * @param m size of the measurement vector
 * @return false if n, c or m are larger than the maximum value
 */
static inline bool linear_kalman_filter_init(struct linear_kalman_filter *filter, uint8_t n, uint8_t c, uint8_t m)
{
  if (n > KF_MAX_STATE_SIZE || c > KF_MAX_CMD_SIZE || m > KF_MAX_MEAS_SIZE) {
    filter->n = 0;
    filter->c = 0;
    filter->m = 0;
    return false; // invalide sizes;
  }
  filter->n = n;
  filter->c = c;
  filter->m = m;

  // Matrix
  MAKE_MATRIX_PTR(_A, filter->A, n);
  float_mat_zero(_A, n, n);
  MAKE_MATRIX_PTR(_B, filter->B, n);
  float_mat_zero(_B, n, c);
  MAKE_MATRIX_PTR(_C, filter->C, m);
  float_mat_zero(_C, m, n);
  MAKE_MATRIX_PTR(_D, filter->D, m);
  float_mat_zero(_D, m, c);
  MAKE_MATRIX_PTR(_P, filter->P, n);
  float_mat_zero(_P, n, n);
  MAKE_MATRIX_PTR(_Q, filter->Q, n);
  float_mat_zero(_Q, n, n);
  MAKE_MATRIX_PTR(_R, filter->R, m);
  float_mat_zero(_R, m, m);

  // Vector
  float_vect_zero(filter->X, n);
  float_vect_zero(filter->Y, m);

  return true;
}


/** Prediction step
 *
 * X = Ad * X + Bd * U
 * P = Ad * P * Ad' + Q
 *
 * @param filter pointer to the filter structure
 * @param U command vector
 */
static inline void linear_kalman_filter_predict(struct linear_kalman_filter *filter, float *U)
{
  float AX[filter->n];
  float BU[filter->n];
  float tmp[filter->n][filter->n];

  MAKE_MATRIX_PTR(_A, filter->A, filter->n);
  MAKE_MATRIX_PTR(_B, filter->B, filter->n);
  MAKE_MATRIX_PTR(_P, filter->P, filter->n);
  MAKE_MATRIX_PTR(_Q, filter->Q, filter->n);
  MAKE_MATRIX_PTR(_tmp, tmp, filter->n);

  // X = A * X + B * U
  float_mat_vect_mul(AX, _A, filter->X, filter->n, filter->n);
  float_mat_vect_mul(BU, _B, U, filter->n, filter->c);
  float_vect_sum(filter->X, AX, BU, filter->n);

  // P = A * P * A' + Q
  float_mat_mul(_tmp, _A, _P, filter->n, filter->n, filter->n); // A * P
  float_mat_mul_transpose(_P, _tmp, _A, filter->n, filter->n, filter->n); // * A'
  float_mat_sum(_P, _P, _Q, filter->n, filter->n); // + Q
}


/** Update step
 *
 * S = Cd * P * Cd' + R
 * K = P * Cd' / S
 * X = X + K * (Y - Cd * X)
 * P = P - K * Cd * P
 *
 * @param filter pointer to the filter structure
 * @param Y measurement vector
 */
static inline void linear_kalman_filter_update(struct linear_kalman_filter *filter, float *Y)
{
  float S[filter->m][filter->m];
  float K[filter->n][filter->m];
  float tmp1[filter->n][filter->m];
  float tmp2[filter->n][filter->n];

  MAKE_MATRIX_PTR(_P, filter->P, filter->n);
  MAKE_MATRIX_PTR(_C, filter->C, filter->m);
  MAKE_MATRIX_PTR(_R, filter->R, filter->m);
  MAKE_MATRIX_PTR(_S, S, filter->m);
  MAKE_MATRIX_PTR(_K, K, filter->n);
  MAKE_MATRIX_PTR(_tmp1, tmp1, filter->n);
  MAKE_MATRIX_PTR(_tmp2, tmp2, filter->n);

  // S = Cd * P * Cd' + R
  float_mat_mul_transpose(_tmp1, _P, _C, filter->n, filter->n, filter->m); // P * C'
  float_mat_mul(_S, _C, _tmp1, filter->m, filter->n, filter->m); // C *
  float_mat_sum(_S, _S, _R, filter->m, filter->m); // + R

  // K = P * Cd' * inv(S)
  float_mat_invert(_S, _S, filter->m); // inv(S) in place
  float_mat_mul(_K, _tmp1, _S, filter->n, filter->m, filter->m); // tmp1 {P*C'} * inv(S)

  // P = P - K * C * P
  float_mat_mul(_tmp2, _K, _C, filter->n, filter->m, filter->n); // K * C
  float_mat_mul_copy(_tmp2, _tmp2, _P, filter->n, filter->n, filter->n); // * P
  float_mat_diff(_P, _P, _tmp2, filter->n, filter->n); // P - K*H*P

  //  X = X + K * err
  float err[filter->n];
  float dx_err[filter->n];

  float_mat_vect_mul(err, _C, filter->X, filter->m, filter->n); // C * X
  float_vect_diff(err, Y, err, filter->m); // err = Y - C * X
  float_mat_vect_mul(dx_err, _K, err, filter->n, filter->m); // K * err
  float_vect_sum(filter->X, filter->X, dx_err, filter->n); // X + dx_err
  }

/** Prediction step and update step for system with feedthrough matrix D
 *
 * X = Ad * X + Bd * U
 * P = Ad * P * Ad' + Q
 * S = Cd * P * Cd' + R
 * K = P * Cd' / S
 * X = X + K * (Y - Cd * X - Dd * U)
 * P = P - K * Cd * P
 * @param filter pointer to the filter structure
 * @param U command vector
 * @param Y measurement vector
 */
static inline void linear_kalman_filter_predict_and_update(struct linear_kalman_filter *filter, float *U, float *Y)
{
  float AX[filter->n];
  float BU[filter->n];
  float tmp[filter->n][filter->n];

  MAKE_MATRIX_PTR(_A, filter->A, filter->n);
  MAKE_MATRIX_PTR(_B, filter->B, filter->n);
  MAKE_MATRIX_PTR(_P, filter->P, filter->n);
  MAKE_MATRIX_PTR(_Q, filter->Q, filter->n);
  MAKE_MATRIX_PTR(_tmp, tmp, filter->n);

  // X = A * X + B * U
  float_mat_vect_mul(AX, _A, filter->X, filter->n, filter->n);
  float_mat_vect_mul(BU, _B, U, filter->n, filter->c);
  float_vect_sum(filter->X, AX, BU, filter->n);

  // P = A * P * A' + Q
  float_mat_mul(_tmp, _A, _P, filter->n, filter->n, filter->n); // A * P
  float_mat_mul_transpose(_P, _tmp, _A, filter->n, filter->n, filter->n); // * A'
  float_mat_sum(_P, _P, _Q, filter->n, filter->n); // + Q



  float S[filter->m][filter->m];
  float K[filter->n][filter->m];
  float tmp1[filter->n][filter->m];
  float tmp2[filter->n][filter->n];

  MAKE_MATRIX_PTR(_C, filter->C, filter->m);
  MAKE_MATRIX_PTR(_D, filter->D, filter->m);
  MAKE_MATRIX_PTR(_R, filter->R, filter->m);
  MAKE_MATRIX_PTR(_S, S, filter->m);
  MAKE_MATRIX_PTR(_K, K, filter->n);
  MAKE_MATRIX_PTR(_tmp1, tmp1, filter->n);
  MAKE_MATRIX_PTR(_tmp2, tmp2, filter->n);

  // S = Cd * P * Cd' + R
  float_mat_mul_transpose(_tmp1, _P, _C, filter->n, filter->n, filter->m); // P * C'
  float_mat_mul(_S, _C, _tmp1, filter->m, filter->n, filter->m); // C *
  float_mat_sum(_S, _S, _R, filter->m, filter->m); // + R

  // K = P * Cd' * inv(S)
  float_mat_invert(_S, _S, filter->m); // inv(S) in place
  float_mat_mul(_K, _tmp1, _S, filter->n, filter->m, filter->m); // tmp1 {P*C'} * inv(S)

  // P = P - K * C * P
  float_mat_mul(_tmp2, _K, _C, filter->n, filter->m, filter->n); // K * C
  float_mat_mul_copy(_tmp2, _tmp2, _P, filter->n, filter->n, filter->n); // * P
  float_mat_diff(_P, _P, _tmp2, filter->n, filter->n); // P - K*H*P

  //  X = X + K * err
  float err[filter->m];
  float err2[filter->m];
  float dx_err[filter->n];
  float DU[filter->m];

  float_mat_vect_mul(err, _C, filter->X, filter->m, filter->n); // C * X
  float_mat_vect_mul(err2, _D, U, filter->m, filter->c); // D * U
  float_vect_diff(err, Y, err, filter->m); // err = Y - C * X
  float_vect_diff(err, err, err2, filter->m); // err = Y - C * X - D * U
  float_mat_vect_mul(dx_err, _K, err, filter->n, filter->m); // K * err
  float_vect_sum(filter->X, filter->X, dx_err, filter->n); // X + dx_err

  float_mat_vect_mul(err, _C, filter->X, filter->m, filter->n);
  float_mat_vect_mul(DU, _D, U, filter->m, filter->c);
  float_vect_sum(filter->Y, err, DU, filter->m);
}
/** Prediction step and update step for system with feedthrough matrix D taken from discrete Kalman
 * filter block in Simulink ('current estimate')
 *
 * X = Ad * X + Bd * U
 * P = Ad * P * Ad' + Q
 * S = Cd * P * Cd' + R
 * M = P * Cd' / S
 * L = Ad * P * Cd' / S
 * Z = (I - M*Cd)P(I - M*Cd)' + M*R*M'
 * X = X + L * (Y - Cd * X - Dd * U)
 * P = Ad * Z * Ad' + Q
 * @param filter pointer to the filter structure
 * @param U command vector
 * @param Y measurement vector
 */
// static inline void linear_kalman_filter_predict_and_update_MATLAB(struct linear_kalman_filter *filter, float *U, float *Y)
// {
//   printf("-------------------- ENTERING MATLAB PRED AND UPD FNC --------------------\n");
//   // prediction step
//   float AX[filter->n];
//   float BU[filter->n];
//   float L[filter->n][filter->m];
//   float S[filter->m][filter->m];
//   float K[filter->n][filter->m];
//   float M[filter->n][filter->m];
//   float Z[filter->n][filter->n];
//   float tmp1[filter->n][filter->m];
//   float tmp2[filter->n][filter->n];
//   float I[filter->n][filter->n];
//   MAKE_MATRIX_PTR(_A, filter->A, filter->n);
//   MAKE_MATRIX_PTR(_B, filter->B, filter->n);
//   MAKE_MATRIX_PTR(_P, filter->P, filter->n);
//   MAKE_MATRIX_PTR(_Q, filter->Q, filter->n);
//   // X = A * X + B * U
//   printf("X contrib %f \n",filter->X[0]);
//   printf("A[0][0] contrib %f \n",filter->A[0][0]);
//   float_mat_vect_mul(AX, _A, filter->X, filter->n, filter->n);
//   printf("AX contrib %f \n",AX[0]);
//   printf("B[0][0] = %f, B[0][1] = %f \n",filter->B[0][0], filter->B[0][1]);
//   printf("U[0] = %f, U[1] = %f \n",U[0], U[1]); 
//   float_mat_vect_mul(BU, _B, U, filter->n, filter->c);
//   printf("BU contrib %f \n",BU[0]);
//   float_vect_sum(filter->X, AX, BU, filter->n);
//   printf("new X = %f \n",filter->X[0]);
//   // update step
//   // MAKE_MATRIX_PTR(_C, filter->C, filter->m);
//   // MAKE_MATRIX_PTR(_D, filter->D, filter->m);
//   // MAKE_MATRIX_PTR(_R, filter->R, filter->m);
//   // MAKE_MATRIX_PTR(_S, S, filter->m);
//   // MAKE_MATRIX_PTR(_L, L, filter->n);
//   // MAKE_MATRIX_PTR(_M, M, filter->n);
//   // MAKE_MATRIX_PTR(_Z, Z, filter->n);
//   // MAKE_MATRIX_PTR(_tmp1, tmp1, filter->n);
//   // MAKE_MATRIX_PTR(_I, I, filter->n);
//   // MAKE_MATRIX_PTR(_tmp2, tmp2, filter->n);
//   // float_mat_diagonal_scal(_I, 1.0, filter->n);
//   // // S = Cd * P * Cd' + R
//   // float_mat_mul_transpose(_tmp1, _P, _C, filter->n, filter->n, filter->m); // P * C'
//   // float_mat_mul(_S, _C, _tmp1, filter->m, filter->n, filter->m); // C *
//   // float_mat_sum(_S, _S, _R, filter->m, filter->m); // + R
//   // // L = A * P * Cd' * inv(S)
//   // // M = P * Cd' * inv(S)
//   // float_mat_invert(_S, _S, filter->m); // inv(S) in place
//   // float_mat_mul(_M, _tmp1, _S, filter->n, filter->m, filter->m); // tmp1{P*C'}*inv(S)
//   // float_mat_mul_copy(_tmp1, _A, _tmp1, filter->n, filter->m, filter->m); // tmp1 = A * P * C
//   // float_mat_mul(_L, _tmp1, _S, filter->n, filter->m, filter->m); // tmp1 {A*P*C'} * inv(S)
//   // // Z = (I - M*Cd)P(I - M*Cd)' + M*R*M'
//   // float_mat_mul(_tmp2, _M, _C, filter->n, filter->m, filter->n); // M*Cd
//   // float_mat_diff(_tmp2, _I, _tmp2, filter->n, filter->n); // I - M*Cd
//   // float_mat_mul_transpose(_Z, _P, _tmp2, filter->n, filter->n, filter->n); // P*(I-M*Cd)'
//   // float_mat_mul_copy(_Z, _tmp2, _Z, filter->n, filter->n, filter->n); // (I-M*Cd)*P*(I-M*Cd)'
//   // float_mat_mul(_tmp1, _M, _R, filter->n, filter->m, filter->m); // M*R
//   // float_mat_mul_transpose(_I, _tmp1, _M, filter->n, filter->m, filter->n); // M*R*M'
//   // float_mat_sum(_Z, _Z, _I, filter->n, filter->n); // (I-M*C)*P*(I-M*C)' + M*R*M' 
//   // // P = Ad * Z * Ad' + Q
//   // float_mat_mul_transpose(_P, _Z, _A, filter->n, filter->n, filter->n); // Z*A'
//   // float_mat_mul(_P, _A, _P, filter->n, filter->n, filter->n); // A*Z*A'
//   // float_mat_sum(_P, _P, _Q, filter->n, filter->n); // A*Z*A'+Q
//   // //  X = X + K * err
//   // float err[filter->m];
//   // float err2[filter->m];
//   // float dx_err[filter->n];
//   // float DU[filter->m];
//   // float_mat_vect_mul(err, _C, filter->X, filter->m, filter->n); // C * X
//   // float_mat_vect_mul(err2, _D, U, filter->m, filter->c); // D * U
//   // float_vect_diff(err, Y, err, filter->m); // err = Y - C * X
//   // float_vect_diff(err, err, err2, filter->m); // err = Y - C * X - D * U
//   // float_mat_vect_mul(dx_err, _L, err, filter->n, filter->m); // L * err
//   // float_vect_sum(filter->X, filter->X, dx_err, filter->n); // X + dx_err
//   // float_mat_vect_mul(err, _C, filter->X, filter->m, filter->n);
//   // float_mat_vect_mul(DU, _D, U, filter->m, filter->c);
//   // float_vect_sum(filter->Y, err, DU, filter->m);
// }

#endif /* DISCRETE_EKF_H */
