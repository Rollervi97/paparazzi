//
// Created by ASUS on 06/12/2021.
//

#ifndef PPRZ_DOUBLET_H
#define PPRZ_DOUBLET_H

#endif //THESIS_PPRZ_DOUBLET_H

#include "std.h"

/**
 * Initialize with doublet_init
 * */

struct doublet_t{
    float t0, t1, t2, t3, t4, t5, tf, total_length_s;
    float current_value;
    float current_time_s;

    bool mod3211;
};

void doublet_init(struct doublet_t *doublet, float length_s, float extra_waiting_time_s float current_time_s, bool mod3211);

void doublet_reset(struct doublet_t *doublet, float current_time_s);

bool doublet_is_running(struct doublet_t *doublet, float current_time_s);

void doublet_update(struct doublet_t *doublet, float current_time_s);