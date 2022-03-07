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
#include "modules/actuators/motor_mixing.h"
// #include "firmwares/rotorcraft/stabilization/stabilization_indi_simple.h"

// struct FirstOrderLowPass propeller_dyn;
// struct FirstOrderHighPass propeller_dyn_dot;

// float alpha[4]; // = {ALPHA_1, ALPHA_2, ALPHA_3, ALPHA_4};
// float input_quantities[4];
extern float rigid_body_yaw_acceleration;
// float servo_rate;

extern void yaw_dynamic_init(void);
extern void yaw_dynamic_run(void);
static inline void read_rigid_body_yaw_acceleration(float *RB_angular_acceleration){
  *RB_angular_acceleration = rigid_body_yaw_acceleration;
} // read rigid body yaw acceleration

#endif