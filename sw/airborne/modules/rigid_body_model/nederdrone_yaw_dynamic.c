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

// #include "firmwares/rotorcraft/stabilization/stabilization_indi_simple.h"

static struct FirstOrderLowPass propeller_dyn;
static struct FirstOrderHighPass propeller_dyn_dot;
Butterworth2LowPass prop_signal;
Butterworth2LowPass servo_signal;
Butterworth2LowPass yaw_rate_filt;

float alpha[4]; // = {ALPHA_1, ALPHA_2, ALPHA_3, ALPHA_4};
float input_quantities[4];
float last_servo_deflection;
float rigid_body_yaw_acceleration;
float sample_time;
float servo_rate;



#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

static void send_nederdrone_yaw_dynamic(struct transport_tx *trans, struct link_device *dev)
{
  //The estimated G values are scaled, so scale them back before sending
  pprz_msg_send_NEDERDRONE_YAW_DYNAMIC(trans, dev, AC_ID,
                                   &input_quantities[0],
                                   &input_quantities[1],
                                   &input_quantities[2],
                                   &input_quantities[3],
                                   &rigid_body_yaw_acceleration);
}

#endif

void yaw_dynamic_init(void)
{
  sample_time = 1.0 / PERIODIC_FREQUENCY;
  init_first_order_low_pass(&propeller_dyn, 1 / PROPELLER_POLE, sample_time, 0.0);
  init_first_order_high_pass(&propeller_dyn_dot, 1 / PROPELLER_POLE, sample_time, 0.0);
  init_butterworth_2_low_pass(&yaw_rate_filt, 1/(2 * M_PI * YAW_RATE_COMP_CUTOFF_F) ,sample_time, 0.0);
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
  // printf("Rigid body mod initilized");
  #if PERIODIC_TELEMETRY
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_NEDERDRONE_YAW_DYNAMIC, send_nederdrone_yaw_dynamic);
  #endif
}

void yaw_dynamic_run(void){
  // yaw rate is given in rad/s, we need deg/s so we calculate the quantity and then we m
  
  // input_quantities[0] = stateGetBodyRates_f()->r * abs(stateGetBodyRates_f()->r) * 180.0 * 180.0 / M_PI / M_PI;
  update_butterworth_2_low_pass(&yaw_rate_filt, stateGetBodyRates_f()->r * 180 / M_PI);
  input_quantities[0] = yaw_rate_filt.o[0] * abs(yaw_rate_filt.o[0]);

  // finish to code servo dynamic calculation
  // lala2 = BoundAbs(indi.u_in.r*2, 6000) * 37.82 / 6000.0; // servo required deflection in deg
  last_servo_deflection = input_quantities[3];
  update_butterworth_2_low_pass(&servo_signal, indi.u_in.r); // applying filtering for controller signal input
  servo_rate = servo_signal.o[0] * 2; // calculate command sent to Actuator
  BoundAbs(servo_rate, 6000); // bound the command according to the airframe file
  servo_rate = SERVO_POLE * (servo_rate * 37.82 / 6000.0 - last_servo_deflection); // calculating the angular rate of the servo [deg/s]
  BoundAbs(servo_rate, 60/0.15); // limiting servo rate according to specifications
  input_quantities[3] = last_servo_deflection + sample_time * servo_rate; // using limited servo rate to update servo position [deg]

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
  // printf("Acceleration estimated: %f \n", rigid_body_yaw_acceleration);
// static inline void read_rigid_body_yaw_acceleration(float *RB_angular_acceleration){
//   // read rigid body yaw acceleration
//   *RB_angular_acceleration =  rigid_body_yaw_acceleration;
}
void read_rigid_body_yaw_acceleration(float *RB_angular_acceleration) {
  *RB_angular_acceleration = rigid_body_yaw_acceleration;
} // read rigid body yaw acceleration
