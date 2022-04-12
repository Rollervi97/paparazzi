/*
 * Copyright (C) Alessandro Collicelli <alessandrocollicelli97@gmail.com>
 * MAVLab Delft University of Technology
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

/** @file nederdrone_yaw_dynamic.c
 * @brief MAVLab Delft University of Technology
 * Rigid body model for yaw dynamic of Nederdrone vehicle
 */
#ifndef NEDERDRONE_YAW_DYNAMIC_H
#define NEDERDRONE_YAW_DYNAMIC_H

#include "std.h"
#include "filters/low_pass_filter.h"
#include "filters/high_pass_filter.h"
#include "modules/actuators/motor_mixing.h"
#include "filters/linear_kalman_filter.h"
// #include "firmwares/rotorcraft/stabilization/stabilization_indi_simple.h"

// struct FirstOrderLowPass propeller_dyn;
// struct FirstOrderHighPass propeller_dyn_dot;
extern float ND_pole;
extern float g_servo;
extern float g_prop;
extern float Q_val;
extern float R_11_val;
extern float R_22_val;
float alpha[4]; // = {ALPHA_1, ALPHA_2, ALPHA_3, ALPHA_4};
// float input_quantities[4];
extern float rigid_body_yaw_acceleration;
// float servo_rate;

void init_Least_Square_rigid_body(void);
void init_KF_pprz(void);
void init_ND_LTI_model(bool flag);
void run_Least_Square_rigid_body(void);
void run_ND_LRI_model(void);


extern void yaw_dynamic_init(void);
extern void yaw_dynamic_run(void);

extern void nederdrone_yaw_dynamic_rset_pole(float dummy_11);
extern void nederdrone_yaw_dynamic_rset_g_prop(float dummy_22);
extern void nederdrone_yaw_dynamic_rset_g_servo(float dummy_33);
extern void nederdrone_yaw_dynamic_rset_Q(float dummy_44);
extern void nederdrone_yaw_dynamic_rset_R11(float dummy_55);
extern void nederdrone_yaw_dynamic_rset_R22(float dummy_66);

extern void read_rigid_body_yaw_acceleration(float *RB_angular_acceleration);
extern void read_KF_pprz_est(float *KF_pprz_acc);
extern void read_ND_LTI_model(float *ND_LTI_acc);

#endif