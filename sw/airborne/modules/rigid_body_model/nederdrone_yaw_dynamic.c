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
  float temp = rigid_body_yaw_acceleration;
  pprz_msg_send_NEDERDRONE_YAW_DYNAMIC(trans, dev, AC_ID,
                                   &input_quantities[0],
                                   &input_quantities[1],
                                   &input_quantities[2],
                                   &input_quantities[3],
                                   &temp);
}

#endif

void yaw_dynamic_init(void)
{
  sample_time = 1.0 / PERIODIC_FREQUENCY;
  init_first_order_low_pass(&propeller_dyn, PROPELLER_POLE, sample_time, 0.0);
  init_first_order_high_pass(&propeller_dyn_dot, PROPELLER_POLE, sample_time, 0.0);
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
  printf("Rigid body mod initilized");
  #if PERIODIC_TELEMETRY
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_NEDERDRONE_YAW_DYNAMIC, send_nederdrone_yaw_dynamic);
  #endif
}

void yaw_dynamic_run(void){
  // get rates in deg/sec, keep only the yaw one
  // printf("Rigid body mod running\n");
  // float current_ang_rate = ;
  input_quantities[0] = (pow(stateGetBodyRates_f()->r, 3)) / abs(stateGetBodyRates_f()->r);
  // finish to code servo dynamic calculation
  // lala2 = BoundAbs(indi.u_in.r*2, 6000) * 37.82 / 6000.0; // servo required deflection in deg
  last_servo_deflection = input_quantities[3];
  servo_rate = indi.u_in.r * 2;
  BoundAbs(servo_rate, 6000);
  servo_rate = SERVO_POLE * (servo_rate * 37.82 / 6000.0 - last_servo_deflection);
  BoundAbs(servo_rate, 60/0.15);
  input_quantities[3] = last_servo_deflection + sample_time * servo_rate;

  
  update_first_order_low_pass(&propeller_dyn, last_bounded_yaw_cmd);
  // update_first_order_high_pass(&propeller_dyn_dot, last_bounded_yaw_cmd);
  // update_first_order_low_pass(&servo_dyn, lala2);

  input_quantities[1] = propeller_dyn.last_out;
  input_quantities[2] = 0.0; //propeller_dyn_dot.o[0];
  // input_quantities[3] = servo_dyn.o[0];

  rigid_body_yaw_acceleration = 0.0;

  for (int8_t i=0; i<4; i++){
    rigid_body_yaw_acceleration = rigid_body_yaw_acceleration + input_quantities[i] * alpha[i];
  }

// static inline void read_rigid_body_yaw_acceleration(float *RB_angular_acceleration){
//   // read rigid body yaw acceleration
//   *RB_angular_acceleration =  rigid_body_yaw_acceleration;
}
