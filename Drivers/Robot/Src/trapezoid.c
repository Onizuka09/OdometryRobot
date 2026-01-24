#include "trapezoid.h"
#include <math.h>

void Trapezoid_Init(TrapezoidProfile *p, float v_max, float a_max, float d_max, float target_dist, float dt) {
    p->v_max = v_max;
    p->a_max = a_max;
    p->d_max = d_max;
    p->target_dist = target_dist;
    p->dt = dt;
    p->t = 0.0f;
    p->finished = false;

    // Calculate time needed to reach v_max
    p->t_acc = v_max / a_max;
    p->t_dec = v_max / d_max;

    // Distance covered during acc and dec
    float d_acc = 0.5f * a_max * (p->t_acc * p->t_acc);
    float d_dec = 0.5f * d_max * (p->t_dec * p->t_dec);

    if (d_acc + d_dec > target_dist) {
        // Triangle profile: distance is too short to reach v_max
        // Re-calculate based on peak velocity reached
        float peak_v = sqrtf((2.0f * a_max * d_max * target_dist) / (a_max + d_max));
        p->t_acc = peak_v / a_max;
        p->t_dec = peak_v / d_max;
        p->t_cruise = 0;
        p->v_max = peak_v;
    } else {
        // Full trapezoid
        float d_cruise = target_dist - (d_acc + d_dec);
        p->t_cruise = d_cruise / v_max;
    }
    
    p->t_total = p->t_acc + p->t_cruise + p->t_dec;
}

float Trapezoid_Update(TrapezoidProfile *p) {
    if (p->t >= p->t_total) {
        p->finished = true;
        return 0.0f;
    }

    float v_target = 0.0f;

    if (p->t < p->t_acc) {
        // Phase 1: Acceleration
        v_target = p->a_max * p->t;
    } else if (p->t < (p->t_acc + p->t_cruise)) {
        // Phase 2: Cruise
        v_target = p->v_max;
    } else if (p->t < p->t_total) {
        // Phase 3: Deceleration
        float t_decel = p->t - (p->t_acc + p->t_cruise);
        v_target = p->v_max - (p->d_max * t_decel);
    }

    p->t += p->dt;
    if (v_target < 0) v_target = 0;
    return v_target;
}

float Trapezoid_UpdateLimiter(TrapezoidProfile *p) {
    float v_target = 0.0f;

    // Phase 1: Acceleration phase
    if (p->t < p->t_acc) {
        v_target = p->a_max * p->t;
    } 
    // Phase 2: Stay at max velocity (The "Limit")
    else {
        v_target = p->v_max;
    }

    // Increment time for the next call
    p->t += p->dt;

    // Final safety: ensure we never return a negative speed 
    // and don't accidentally exceed v_max due to float precision
    if (v_target > p->v_max) v_target = p->v_max;
    if (v_target < 0.0f) v_target = 0.0f;

    return v_target;
}
