/*
 * Copyright (C) Ewoud Smeur <ewoud_smeur@msn.com>
 * MAVLab Delft University of Technology
 *
 * This control algorithm is Incremental Nonlinear Dynamic Inversion (INDI)
 *
 * This is a simplified implementation of the (soon to be) publication in the
 * journal of Control Guidance and Dynamics: Adaptive Incremental Nonlinear
 * Dynamic Inversion for Attitude Control of Micro Aerial Vehicles
 *
 * This file is part of paparazzi.
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/** @file stabilization_attitude_quat_indi_simple.h
 * Stabilization based on INDI for multicopters.
 * It supports both rate and attidue control.
 */

#ifndef STABILIZATION_INDI_SIMPLE_H
#define STABILIZATION_INDI_SIMPLE_H

#include "firmwares/rotorcraft/stabilization/stabilization_attitude_ref_quat_int.h"
#include "filters/low_pass_filter.h"
#include "filters/complementary_filter.h"
#include "modules/rigid_body_model/nederdrone_yaw_dynamic.h"

extern uint8_t use_complementary_feedback;
extern uint8_t high_freq_component_active;
extern uint8_t use_LTI_acc;
extern uint8_t NF_on;
extern float NF_freq;
extern uint8_t KF_on;
extern float NF_bandwidth;

extern uint8_t dist_on;
extern float dist_start_time;
extern float dist_value;
extern float dist_length;

// extern float new_r_dot_cutoff;
extern float complementary_cross_freq;
extern float rigid_body_acc;
extern struct Int32Quat   stab_att_sp_quat;  ///< with #INT32_QUAT_FRAC
extern struct Int32Eulers stab_att_sp_euler; ///< with #INT32_ANGLE_FRAC

struct Indi_gains {
  struct FloatRates att;
  struct FloatRates rate;
};

struct IndiEstimation {
  Butterworth2LowPass u[3];
  Butterworth2LowPass rate[3];
  float rate_d[3];
  float rate_dd[3];
  float u_d[3];
  float u_dd[3];
  struct FloatRates g1;
  float g2;
  float mu;
};

struct IndiVariables {
  float cutoff_r;
  struct FloatRates angular_accel_ref;
  struct FloatRates du;
  struct FloatRates u_in;
  struct FloatRates u_act_dyn;
  float rate_d[3];

  Butterworth2LowPass u[3];
  Butterworth2LowPass rate[3];
  struct FloatRates g1;
  float g2;

  struct Indi_gains gains;

  bool adaptive;             ///< Enable adataptive estimation
  float max_rate;            ///< Maximum rate in rate control in rad/s
  float attitude_max_yaw_rate; ///< Maximum yaw rate in atttiude control in rad/s
  struct IndiEstimation est; ///< Estimation parameters for adaptive INDI
};


extern struct IndiVariables indi;
extern void stabilization_indi_init(void);
extern void stabilization_indi_enter(void);
extern void stabilization_indi_set_failsafe_setpoint(void);
extern void stabilization_indi_set_rpy_setpoint_i(struct Int32Eulers *rpy);
extern void stabilization_indi_set_earth_cmd_i(struct Int32Vect2 *cmd, int32_t heading);
extern void stabilization_indi_rate_run(struct FloatRates rates_sp, bool in_flight);
extern void stabilization_indi_attitude_run(struct Int32Quat quat_sp, bool in_flight);
extern void stabilization_indi_read_rc(bool in_flight, bool in_carefree, bool coordinated_turn);
extern void stabilization_indi_simple_reset_r_filter_cutoff(float new_cutoff);

extern void stabilization_indi_simple_complementary_filter_flag_handler(bool dummy_1);
extern void stabilization_indi_simple_high_freq_component_complementary_filter(bool dummy_2);
extern void stabilization_indi_simple_NF_handler(bool dummy_3);
extern void stabilization_indi_simple_KF_feedback_handler(bool dummy_4);
extern void stabilization_indi_simple_reset_NF_bandwidth(float newNFb);

extern void stabilization_indi_simple_dist_on(bool dum);
extern void stabilization_indi_simple_set_dist_val(float dum);
extern void stabilization_indi_simple_set_dist_length(float dum);

// extern void stabilization_indi_simple_reset_r_dot_filter_cutoff(float new_r_dot_cutoff);
extern void stabilization_indi_simple_reset_complementary_cross_frequency(float new_ccf);
extern void stabilization_indi_simple_reset_NF_freq(float newNFfreq);

// extern bool stabilization_indi_simple_get_dist_on();
// extern int32_t stabilization_indi_simple_get_dist_val();
#endif /* STABILIZATION_INDI_SIMPLE_H */

