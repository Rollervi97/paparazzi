/*
 * Copyright (C) 2022 Alessandro Collicelli
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

/** @file filters/complementary_filter.h
 *  @brief Complementary filter based on low pass filter with bilinear transform
 *
 */

#ifndef COMPLEMENTARY_FILTER_H
#define COMPLEMENTARY_FILTER_H

#include "std.h"
#include "math/pprz_algebra_int.h"
#include "low_pass_filter.h"


struct SecondOrderComplementaryButterworth{
    Butterworth2LowPass AuxLowFrequencyComponent;
    float LowFrequencyComponent;
    float HighFrequencyComponent;
    Butterworth2LowPass AuxHighFrequencyComponent;
    float filter_output;

};

static inline void init_SecondOrderComplementaryButterworth(struct SecondOrderComplementaryButterworth *filter, float tau, float sample_time, float valueLF, float valueHF){
    init_butterworth_2_low_pass(&filter->AuxLowFrequencyComponent, tau, sample_time, valueLF);
    filter->HighFrequencyComponent = valueHF;
    filter->LowFrequencyComponent = valueLF;
    init_butterworth_2_low_pass(&filter->AuxHighFrequencyComponent, tau, sample_time, valueHF);
    filter->filter_output = valueLF + valueHF;
}

static inline void update_SecondOrderComplementaryButterworth(struct SecondOrderComplementaryButterworth *filter, float valueLF, float valueHF){
    update_butterworth_2_low_pass(&filter->AuxLowFrequencyComponent, valueLF);
    update_butterworth_2_low_pass(&filter->AuxHighFrequencyComponent, valueHF);
    filter->HighFrequencyComponent = valueHF - filter->AuxHighFrequencyComponent.o[0];
    filter->LowFrequencyComponent = filter->AuxLowFrequencyComponent.o[0];
    filter->filter_output = filter->HighFrequencyComponent + filter->LowFrequencyComponent;
}

#endif

