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

#ifndef PROPELLER_POLE
#define PROPELLER_POLE 18.0
#endif

#ifndef SERVO_POLE
#define SERVO_POLE 50.0
#endif

#include "std.h"
#include "state.h"
#include "generated/airframe.h"
#include "paparazzi.h"
#include "modules/radio_control/radio_control.h"
#include "filters/low_pass_filter.h"
#include "filters/high_pass_filter.h"
#include "modules/actuators/motor_mixing.h"

#include "firmwares/rotorcraft/stabilization/stabilization_indi_simple.h"

static struct FirstOrderLowPass propeller_dyn;
static struct FirstOrderHighPass propeller_dyn_dot;

float alpha[4]; // = {ALPHA_1, ALPHA_2, ALPHA_3, ALPHA_4};
float input_quantities[4];
float last_servo_deflection;
float rigid_body_yaw_acceleration;
float sample_time;



void yaw_dynamic_init(void)
{
  sample_time = 1.0 / PERIODIC_FREQUENCY;
  init_first_order_low_pass(&propeller_dyn, PROPELLER_POLE, sample_time, 0.0);
  init_first_order_high_pass(&propeller_dyn_dot, PROPELLER_POLE, sample_time, 0.0);
  init_first_order_low_pass(&servo_dyn, SERVO_POLE, sample_time, 0.0);
  alpha[0] =  ALPHA_1;
  alpha[1] =  ALPHA_2;
  alpha[2] =  ALPHA_3;
  alpha[3] =  ALPHA_4;

  for (int8_t i=0; i<4; i++){
    input_quantities[i] = 0.0;
  }

  rigid_body_yaw_acceleration = 0.0;
  last_servo_deflection = 0.0;
  // servo_rate = 0.0;
}


void yaw_dynamic_run(void){
  // get rates in deg/sec, keep only the yaw one
  input_quantities[0] = ((stateGetBodyRates_f()->r) ^ 3) / abs(stateGetBodyRates_f()->r;
  // finish to code servo dynamic calculation
  // lala2 = BoundAbs(indi.u_in.r*2, 6000) * 37.82 / 6000.0; // servo required deflection in deg
  last_servo_deflection = input_quantities[3];
  float servo_rate = BoundAbs(SERVO_POLE * ((BoundAbs(indi.u_in.r*2, 6000) * 37.82 / 6000.0) - last_servo_deflection), 60.0 / 0.15);
  input_quantities[3] = last_servo_deflection + sample_time * servo_rate;

  
  update_first_order_low_pass(&propeller_dyn, last_bounded_yaw_cmd);
  update_first_order_high_pass(&propeller_dyn_dot, last_bounded_yaw_cmd);
  // update_first_order_low_pass(&servo_dyn, lala2);

  input_quantities[1] = propeller_dyn.o[0];
  input_quantities[2] = propeller_dyn_dot.o[0];
  // input_quantities[3] = servo_dyn.o[0];

  rigid_body_yaw_acceleration = 0.0;
  for (int8_t i=0; i<4; i++){
    rigid_body_yaw_acceleration = rigid_body_yaw_acceleration + input_quantities[i] * alpha[i];
  }

float read_rigid_body_yaw_acceleration(void) {
  return rigid_body_yaw_acceleration;
}
}