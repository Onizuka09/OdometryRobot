#ifndef TRAPEZOID_H
#define TRAPEZOID_H

#include <stdbool.h>

typedef struct {
    float v_max;
    float a_max;
    float d_max;
    float target_dist;
    float dt;
    
    float t_acc;
    float t_dec;
    float t_cruise;
    float t_total;
    
    float t;
    bool finished;
} TrapezoidProfile;

void Trapezoid_Init(TrapezoidProfile *p, float v_max, float a_max, float d_max, float target_dist, float dt);
float Trapezoid_Update(TrapezoidProfile *p);
bool Trapezoid_IsFinished(TrapezoidProfile *p);

#endif