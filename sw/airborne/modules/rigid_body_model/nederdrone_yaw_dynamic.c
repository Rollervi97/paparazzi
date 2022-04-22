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

#ifndef ALPHA_1
#define ALPHA_1 -0.032
#endif

#ifndef ALPHA_2
#define ALPHA_2 0.0
#endif

#ifndef ALPHA_3
#define ALPHA_3 0.0368
#endif

#ifndef ALPHA_4
#define ALPHA_4 4.652
#endif

#ifndef PROPELLER_POLE
#define PROPELLER_POLE 18.0
#endif

#ifndef SERVO_POLE
#define SERVO_POLE 50.0
#endif

#ifndef YAW_RATE_COMP_CUTOFF_F
#define YAW_RATE_COMP_CUTOFF_F 5.0
#endif

#ifndef KF_STATE_D
#define KF_STATE_D 1
#endif

#ifndef KF_CTRL_D
#define KF_CTRL_D 2
#endif

#ifndef KF_MEAS_D
#define KF_MEAS_D 2
#endif

#include "stdio.h"
#include "math.h"
#include "stdlib.h"
#include "std.h"
#include "state.h"
#include "generated/airframe.h"
#include "paparazzi.h"
#include "filters/low_pass_filter.h"
#include "filters/high_pass_filter.h"
#include "modules/actuators/motor_mixing.h"
#include "modules/rigid_body_model/nederdrone_yaw_dynamic.h"
#include "filters/linear_kalman_filter.h"

// #include "firmwares/rotorcraft/stabilization/stabilization_indi_simple.h"

static struct FirstOrderLowPass propeller_dyn;
static struct FirstOrderHighPass propeller_dyn_dot;
static struct FirstOrderHighPass ND_LTI_model;
struct linear_kalman_filter KF_pprz;

float ND_pole = -2.737;
float g_prop = 0.004129; 
float g_servo = 5.535;
float Q_val = 5.0;
float R_11_val = 5.0;
float R_22_val = 20.0;

Butterworth2LowPass prop_signal;
Butterworth2LowPass servo_signal;
Butterworth2LowPass yaw_rate_filt;
Butterworth2LowPass KF_meas;

float alpha[4]; // = {ALPHA_1, ALPHA_2, ALPHA_3, ALPHA_4};
float input_quantities[4];
float last_servo_deflection;
float rigid_body_yaw_acceleration;
float sample_time;//= 1 / PERIODIC_FREQUENCY;
float servo_rate;
float rate[2]; // {current rate, previous rate}



#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

static void send_nederdrone_yaw_dynamic(struct transport_tx *trans, struct link_device *dev)
{
  float t1 = rigid_body_yaw_acceleration * 180.0f / M_PI; 
  pprz_msg_send_NEDERDRONE_YAW_DYNAMIC(trans, dev, AC_ID,
                                  &t1, // deg/s^2
                                  &KF_pprz.Y[1], // deg/s^2
                                  &ND_LTI_model.last_out, // deg/s^2
                                  &ND_pole, &g_prop, &g_servo,
                                  &input_quantities[1], &input_quantities[3]);
}

#endif

void init_Least_Square_rigid_body(void)
{
  
  init_first_order_low_pass(&propeller_dyn, 1 / PROPELLER_POLE, sample_time, 0.0);
  init_first_order_high_pass(&propeller_dyn_dot, 1 / PROPELLER_POLE, sample_time, 0.0);
  init_butterworth_2_low_pass(&yaw_rate_filt, 1/(2 * M_PI * YAW_RATE_COMP_CUTOFF_F) ,sample_time, 0.0);
  init_butterworth_2_low_pass(&prop_signal, 1/(2 * M_PI * YAW_RATE_COMP_CUTOFF_F) ,sample_time, 0.0);
  init_butterworth_2_low_pass(&servo_signal, 1/(2 * M_PI * YAW_RATE_COMP_CUTOFF_F) ,sample_time, 0.0);

  // init_first_order_low_pass(&servo_dyn, SERVO_POLE, sample_time, 0.0);
  alpha[0] =  ALPHA_1;
  alpha[1] =  ALPHA_2;
  alpha[2] =  ALPHA_3;
  alpha[3] =  ALPHA_4;

  for (int8_t i=0; i<4; i++){
    input_quantities[i] = 0.0;
  }

  rigid_body_yaw_acceleration = 0.0;
  last_servo_deflection = 0.0;
  servo_rate = 0.0;
  // printf("last servo deflection = %f \n", last_servo_deflection);
}

void init_KF_pprz(void)
{
  linear_kalman_filter_init(&KF_pprz, KF_STATE_D, KF_CTRL_D, KF_MEAS_D);
  KF_pprz.A[0][0] = ND_pole;

  KF_pprz.B[0][0] = g_prop;
  KF_pprz.B[0][1] = g_servo;
  
  KF_pprz.C[0][0] = 1.0;
  KF_pprz.C[1][0] = ND_pole;
  
  KF_pprz.D[1][0] = g_prop;
  KF_pprz.D[1][1] = g_servo;
  KF_pprz.Q[0][0] = Q_val;
  KF_pprz.R[0][0] = R_11_val;
  KF_pprz.R[1][1] = R_22_val;

  rate[0] = 0.0;
  rate[1] = 0.0;
}

void init_ND_LTI_model(bool flag)
{ 
  if(flag){
   init_first_order_high_pass(&ND_LTI_model, 1 / -ND_pole, sample_time, 0.0);
  } else {
    init_first_order_high_pass(&ND_LTI_model, 1 / -ND_pole, sample_time, ND_LTI_model.last_out);
  }
}

void run_Least_Square_rigid_body(void)
{
  // yaw rate is given in rad/s, we need deg/s so we calculate the quantity and then we m
  
  // input_quantities[0] = stateGetBodyRates_f()->r * abs(stateGetBodyRates_f()->r) * 180.0 * 180.0 / M_PI / M_PI;
  update_butterworth_2_low_pass(&yaw_rate_filt, stateGetBodyRates_f()->r * 180 / M_PI);
  input_quantities[0] = yaw_rate_filt.o[0] * abs(yaw_rate_filt.o[0]);
  // printf("last controller cmd %f \n", indi.u_in.r);
  // finish to code servo dynamic calculation
  // lala2 = BoundAbs(indi.u_in.r*2, 6000) * 37.82 / 6000.0; // servo required deflection in deg
  last_servo_deflection = input_quantities[3];
  // printf("check 1 %f \n", indi.u_in.r);
  update_butterworth_2_low_pass(&servo_signal, indi.u_in.r); // applying filtering for controller signal input
  servo_rate = servo_signal.o[0] * 2; // calculate command sent to Actuator
  BoundAbs(servo_rate, 6000); // bound the command according to the airframe file
  
  servo_rate = SERVO_POLE * (servo_rate * 37.82 / 6000.0 - last_servo_deflection); // calculating the angular rate of the servo [deg/s]
  BoundAbs(servo_rate, 60/0.15); // limiting servo rate according to specifications
  // printf("check super %f ---- %f ----- %f\n", last_servo_deflection, sample_time, servo_rate);
  input_quantities[3] = last_servo_deflection + sample_time * servo_rate;
  BoundAbs(input_quantities[3], 37.82); // using limited servo rate to update servo position [deg]
  // printf("Last servo def %f, sample_Time = %f, servo_rate = %f \n servo contrib %f \n", last_servo_deflection, sample_time, servo_rate, input_quantities[3]);
  // filering actuator command as in the INDI actuator synchronization loop
  update_butterworth_2_low_pass(&prop_signal, last_bounded_yaw_cmd);
  update_first_order_low_pass(&propeller_dyn, prop_signal.o[0]);
  update_first_order_high_pass(&propeller_dyn_dot,  prop_signal.o[0]);
  
  input_quantities[1] = propeller_dyn.last_out; // motor yaw command after motor mixing routine [PPRZ_CMD]
  input_quantities[2] = propeller_dyn_dot.last_out; // motor yaw command after motor mixing routine [PPRZ_CMD/s]
  // input_quantities[3] = servo_dyn.o[0];

  //calculation of rigid body model acceleration
  rigid_body_yaw_acceleration = 0.0;
  // printf("Acceleration initiailidez to zero: %f \n", rigid_body_yaw_acceleration);
  for (int8_t i=0; i<4; i++){
    rigid_body_yaw_acceleration = rigid_body_yaw_acceleration + input_quantities[i] * alpha[i];
  }
  rigid_body_yaw_acceleration = rigid_body_yaw_acceleration / 180 * M_PI; // getting the acceleration in rad/s^2 
  // printf("check final %f ---- %f \n \n\n\n", input_quantities[1], input_quantities[3]);
  // printf("Acceleration estimated: %f \n", rigid_body_yaw_acceleration);
}

void run_ND_LRI_model(void)
{
  // calculate input
  float input = input_quantities[1] * g_prop + input_quantities[3] * g_servo;
  // input /= abs(ND_pole);
  update_first_order_high_pass(&ND_LTI_model, input);
}

void yaw_dynamic_init(void)
{
  sample_time = 1.0 / PERIODIC_FREQUENCY;
  init_Least_Square_rigid_body();
  init_KF_pprz();

  init_butterworth_2_low_pass(&KF_meas,  1/(2 * M_PI * 5.0) ,sample_time, 0.0);
  init_ND_LTI_model(true);
  rate[0] = 0;
  rate[1] = 0;
  // printf("LS_RB acc = %f, Kalman filter acc = %f, LTI model acc = %f \n", rigid_body_yaw_acceleration, KF_pprz.Y[1], ND_LTI_model.last_out);

  // printf("Rigid body mod initilized");
  #if PERIODIC_TELEMETRY
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_NEDERDRONE_YAW_DYNAMIC, send_nederdrone_yaw_dynamic);
  #endif
}


void nederdrone_yaw_dynamic_rset_pole(float dummy_11){
  ND_pole = dummy_11;
  init_ND_LTI_model(false);
  init_KF_pprz();
}

void nederdrone_yaw_dynamic_rset_g_prop(float dummy_22){
  g_prop = dummy_22;
  init_KF_pprz();
}

void nederdrone_yaw_dynamic_rset_g_servo(float dummy_33){
  g_servo = dummy_33;
  init_KF_pprz();
}

void nederdrone_yaw_dynamic_rset_Q(float dummy_44){ 
  Q_val = dummy_44;
  init_KF_pprz();
}

void nederdrone_yaw_dynamic_rset_R11(float dummy_55){ 
  R_11_val = dummy_55;
  init_KF_pprz();
}

void nederdrone_yaw_dynamic_rset_R22(float dummy_66){
  R_22_val = dummy_66;
  init_KF_pprz();
}

void yaw_dynamic_run(void){
  // updating least squared rigid body model estimation
  float U[KF_CTRL_D];
  float Y[KF_MEAS_D];
  run_Least_Square_rigid_body();

  // updating Kalman filter estimation 
  U[0] = input_quantities[1];
  U[1] = input_quantities[3];
  rate[1] = rate[0];
  rate[0] = stateGetBodyRates_f()->r * 180 / M_PI;
  update_butterworth_2_low_pass(&KF_meas, (rate[0]-rate[1]) * PERIODIC_FREQUENCY);
  Y[0] = yaw_rate_filt.o[0];
  Y[1] = KF_meas.o[0];
  // linear_kalman_filter_predict(&KF_pprz, U);
  // linear_kalman_filter_update(&KF_pprz, Y);

  // printf("ND_YAW_DYN, U[0] = %f, U[1] = %f, Y[0] = %f \n", U[0], input_quantities[3], Y[0]);
  linear_kalman_filter_predict_and_update(&KF_pprz, U, Y);
  
  // update LTI model
  run_ND_LRI_model();

}

void read_rigid_body_yaw_acceleration(float *RB_angular_acceleration) {
  *RB_angular_acceleration = rigid_body_yaw_acceleration;
} // read rigid body yaw acceleration

void read_KF_pprz_est(float *KF_pprz_acc)
{ 
  *KF_pprz_acc = KF_pprz.Y[0] * M_PI / 180.0;
}

void read_ND_LTI_model(float *ND_LTI_acc){
  // outputin rad/s^2
  *ND_LTI_acc = ND_LTI_model.last_out * M_PI / 180.0;
}