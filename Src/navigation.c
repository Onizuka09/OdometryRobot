#include <math.h>
#include "navigation.h"
#include "motor.h"
#include "delay.h"
#include "bsp.h"
#include "trapezoid.h"
#define VEL_ACCEL_RATE 10.0f

typedef struct
{
    float kp;
    float ki;
    float kd;
    float error;
    float correction;
    float ramped_setpoint;
    float target;

} PID_Profile;
typedef struct
{
    uint32_t now;
    uint32_t last; // last time
} Timestamps;

Timestamps vel_time = {.now = 0, .last = 0};

PID_Profile pid_vel = {
    .kp = 2.5f,
    .ki = 2.0f,
    .kd = 0.05f,
    .error = 0.0f,
    .correction = 0.0f,
    .ramped_setpoint = 0.0f,
    .target = 0.0f

};


float velocity_ramp_limit = 0.0f;
float velocity_target = 0.0f;

PID_Profile pid_pos = {
    .kp = 4.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .error = 0.0f,
    .correction = 0.0f,
    .ramped_setpoint = 0.0f,
    .target = 0.0f

};
uint32_t now = 0.0;  // current time
uint32_t last = 0.0; // last time
PID_Profile pid_w = {
    .kp = 1.8f,
    .ki = 2.5f,
    .kd = 0.05f,
    .error = 0.0f,
    .correction = 0.0f,
    .ramped_setpoint = 0.0f,
    .target = 0.0f

};
PID_Profile pid_theta = {
    .kp = 2.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .error = 0.0f,
    .correction = 0.0f,
    .ramped_setpoint = 0.0f,
    .target = 0.0f};
void angle_control(float desired_angle, OdometryTypedef *odo)
{
    float w_P = 0, w_I = 0, w_D = 0;
    float theta_P = 0, theta_I = 0, theta_D = 0;
    
    // --- FIX 1: Reset the ramp state at the start ---
    pid_w.ramped_setpoint = 0.0f;

    // absolute target calculation
    pid_theta.target = desired_angle ;
    while (pid_theta.target >  PI) pid_theta.target -= 2.0f * PI;
    while (pid_theta.target < -PI) pid_theta.target += 2.0f * PI;

    // // --- FIX 2: Initialize last_errors to CURRENT error to prevent the first-loop spike ---
    // OdoUpdate(odo, 0.01f); // Get fresh data
    // float initial_theta_error = pid_theta.target - odo->angle_rad;
    // while (initial_theta_error >  PI) initial_theta_error -= 2.0f * PI;
    // while (initial_theta_error < -PI) initial_theta_error += 2.0f * PI;
    
    float theta_last_error = 0.0f ; // initial_theta_error;
    float w_last_error = 0.0f; // Velocity starts at 0, so 0 is fine here

    last = get_current_time_ms();
    
    while (1)
    {
        now = get_current_time_ms();
        float dt = (float)(now - last) / 1000.0f;
        if (dt <= 0) dt = 0.001f;
        last = now;
        
        OdoUpdate(odo, dt);

        // --- POSITION PID (Master) ---
        pid_theta.error = pid_theta.target - odo->angle_rad;
        // Normalize angle error to [-pi, pi]
        while (pid_theta.error >  PI) pid_theta.error -= 2.0f * PI;
        while (pid_theta.error < -PI) pid_theta.error += 2.0f * PI;
        
        // Exit Condition: Accuracy and Stability
        if (fabsf(pid_theta.error) <= 0.00175f) 
        {
            break;
        }

        theta_P = pid_theta.error;
        theta_I += pid_theta.error * dt;
        
       

        theta_D = (pid_theta.error - theta_last_error) / dt;
        theta_last_error = pid_theta.error;

        pid_theta.correction = pid_theta.kp * theta_P + pid_theta.ki * theta_I + pid_theta.kd * theta_D;

        // --- VELOCITY GOVERNOR (Symmetrical Ramp) ---
        pid_w.target = pid_theta.correction;

        if (pid_w.ramped_setpoint < MAX_ANGULAR_VEL)
        {
            pid_w.ramped_setpoint += MAX_ANGULAR_ACCEL * dt;
        }

        // FIX 3: Symmetrical Clamping (Works for both Left and Right turns)
        if (pid_w.target >  pid_w.ramped_setpoint) pid_w.target =  pid_w.ramped_setpoint;
        if (pid_w.target < -pid_w.ramped_setpoint) pid_w.target = -pid_w.ramped_setpoint;

        // Final Safety limit
        if (pid_w.target >  MAX_ANGULAR_VEL) pid_w.target =  MAX_ANGULAR_VEL;
        if (pid_w.target < -MAX_ANGULAR_VEL) pid_w.target = -MAX_ANGULAR_VEL;

        // --- VELOCITY PID (Slave) ---
        pid_w.error = pid_w.target - odo->w;
        w_P = pid_w.error;

        if (fabsf(pid_w.error) > 0.05f)
        {
            w_I += pid_w.error * dt;
        }

        if (w_I > 25.0f) w_I = 25.0f;
        if (w_I < -25.0f) w_I = -25.0f;

        w_D = (pid_w.error - w_last_error) / dt;
        w_last_error = pid_w.error;

        pid_w.correction = pid_w.kp * w_P + w_I * pid_w.ki + pid_w.kd * w_D;

        // Motor Command (Rotation)
        run_motors(ANGULAR_CMD, pid_w.correction,-pid_w.correction);

        delay_ms(10);
    }
    
    run_motors(ANGULAR_CMD, 0, 0); 
    TurOff_led();
    last = 0 ; now = 0 ; 
}
void position_control(int16_t desired_distance, OdometryTypedef *odo)
{
    // desired_distance = 10; // 10 cm
    float pos_P = 0, pos_I = 0, pos_D = 0;
    float v_P = 0, v_I = 0, v_D = 0;
    float pos_last_error = 0.0f;
    float vel_last_error = 0.0f;
    velocity_ramp_limit = 0.0f;

    // OdoUpdate()
    pid_pos.target = desired_distance + odo->total_distance;
    last = get_current_time_ms();

    while (1)
    {
        now = get_current_time_ms();
        float dt = (float)(now - last) / 1000.0f;
        if (dt <= 0)
            dt = 0.001f;
        last = now;

        OdoUpdate(odo, dt);
        pid_pos.error = pid_pos.target - odo->total_distance;
        if (fabsf(pid_pos.error) <= 0.2f) // exit loop
        {
            break;
        }

        pos_P = pid_pos.error;
        pos_I += pid_pos.error * dt;
        pos_D = (pid_pos.error - pos_last_error) / dt;
        pos_last_error = pid_pos.error;

        // corrected speed
        pid_pos.correction = pid_pos.kp * pos_P + pid_pos.ki * pos_I + pid_pos.kd * pos_D;

        // velocity
        // We ramp up a "Speed Limit" so the robot doesn't jerk at the start

        if (velocity_ramp_limit < MAX_VELOCITY)
        {
            velocity_ramp_limit += VEL_ACCEL_RATE * dt;
        }
        velocity_target = pid_pos.correction;

        if (velocity_target > velocity_ramp_limit)
            velocity_target = velocity_ramp_limit;
        if (velocity_target > MAX_VELOCITY)
            velocity_target = MAX_VELOCITY;
        if (velocity_target < -MAX_VELOCITY)
            velocity_target = -MAX_VELOCITY;

        // --- PID CALCULATION ---
        pid_vel.error = velocity_target - odo->v;
        // error = desired_motor_speed.value.linear_speed - odo->v; //
        v_P = pid_vel.error;
        // Deadzone for I to prevent jitter at high speed
        if (fabs(pid_vel.error) > 0.5f)
        {
            v_I += pid_vel.error * dt;
        }
        // anti windup for i
        if (v_I > 20.0f)
            v_I = 20.0f;
        if (v_I < -20.0f)
            v_I = -20.0f;
        v_D = (pid_vel.error - vel_last_error) / dt;
        vel_last_error = pid_vel.error;
        pid_vel.correction = pid_vel.kp * v_P + v_I * pid_vel.ki + pid_vel.kd * v_D;

        run_motors(POSITION_CMD, pid_vel.correction, pid_vel.correction);

        delay_ms(10);
    }
    run_motors(POSITION_CMD, 0, 0); // Active brake
    TurOff_led();
    last = 0;
    now = 0;
}
void move_linear_speed(float desired_motor_speed, OdometryTypedef *odo)
{
    float v_P = 0, v_I = 0, v_D = 0;

    // This starts at 0 and climbs to your desired speed
    float final_target = 40;
    float last_error = 0;
    desired_motor_speed = final_target;

    // This starts at 0 and climbs to your desired speed

    vel_time.last = get_current_time_ms();
    while (1)
    {
        /* code */

        vel_time.now = get_current_time_ms();
        float dt = (float)(vel_time.now - vel_time.last) / 1000;
        if (dt <= 0)
            dt = 0.001f;
        vel_time.last = vel_time.now;
        OdoUpdate(odo, dt);
        // ramp
        if (pid_vel.ramped_setpoint < final_target)
        {
            pid_vel.ramped_setpoint += VEL_ACCEL_RATE * dt;
            if (pid_vel.ramped_setpoint > final_target)
                pid_vel.ramped_setpoint = final_target;
        }
        // If we want to slow down, decrease it
        else if (pid_vel.ramped_setpoint > final_target)
        {
            pid_vel.ramped_setpoint -= VEL_ACCEL_RATE * dt;
            if (pid_vel.ramped_setpoint < final_target)
                pid_vel.ramped_setpoint = final_target;
        }

        // --- PID CALCULATION ---
        pid_vel.error = pid_vel.ramped_setpoint - odo->v;
        // error = desired_motor_speed.value.linear_speed - odo->v; //
        v_P = pid_vel.error;
        // Deadzone for I to prevent jitter at high speed
        if (fabs(pid_vel.error) > 0.5f)
        {
            v_I += pid_vel.error * dt;
        }
        // anti windup for i
        if (v_I > 20.0f)
            v_I = 20.0f;
        if (v_I < -20.0f)
            v_I = -20.0f;
        v_D = (pid_vel.error - last_error) / dt;
        last_error = pid_vel.error;
        pid_vel.correction = pid_vel.kp * v_P + v_I * pid_vel.ki + pid_vel.kd * v_D;

        // //
        // if (correction > MAX_VELOCITY)
        //     correction = MAX_VELOCITY;
        // if (correction < -MAX_VELOCITY)
        //     correction = -MAX_VELOCITY;

        // if (fabsf(pid_vel.error) <= 0.1f)
        // {
        //     run_motors(0, 0); // stop
        //     TurOff_led();
        //     break;
        // }
        run_motors(POSITION_CMD, pid_vel.correction, pid_vel.correction);

        delay_ms(10);
    }
    return;
}

void move_angular_speed(float desired_motor_ang_speed, OdometryTypedef *odo)
{
    float w_P = 0, w_I = 0, w_D = 0;

    // This starts at 0 and climbs to your desired speed
    float final_target = desired_motor_ang_speed;
    float last_error = 0;
    pid_w.target = final_target;

    // This starts at 0 and climbs to your desired speed

    last = get_current_time_ms();
    while (1)
    {
        /* code */

        now = get_current_time_ms();
        float dt = (float)(now - last) / 1000;
        if (dt <= 0)
            dt = 0.001f;
        last = now;
        OdoUpdate(odo, dt);
        // ramp
        if (pid_w.ramped_setpoint < final_target)
        {
            pid_w.ramped_setpoint += MAX_ANGULAR_ACCEL * dt;
            if (pid_w.ramped_setpoint > final_target)
                pid_w.ramped_setpoint = final_target;
        }
        // If we want to slow down, decrease it
        else if (pid_w.ramped_setpoint > final_target)
        {
            pid_w.ramped_setpoint -= MAX_ANGULAR_ACCEL * dt;
            if (pid_w.ramped_setpoint < final_target)
                pid_w.ramped_setpoint = final_target;
        }

        // --- PID CALCULATION ---
        pid_w.error = pid_w.ramped_setpoint - odo->w;
        // error = desired_motor_speed.value.linear_speed - odo->v; //
        w_P = pid_w.error;
        // Deadzone for I to prevent jitter at high speed
        if (fabs(pid_w.error) > 0.05f)
        {
            w_I += pid_w.error * dt;
        }
        // anti windup for i
        if (w_I > 20.0f)
            w_I = 20.0f;
        if (w_I < -20.0f)
            w_I = -20.0f;

        w_D = (pid_w.error - last_error) / dt;

        last_error = pid_w.error;

        pid_w.correction = pid_w.kp * w_P + w_I * pid_w.ki + pid_w.kd * w_D;

        // if (correction > MAX_VELOCITY)
        //     correction = MAX_VELOCITY;
        // if (correction < -MAX_VELOCITY)
        //     correction = -MAX_VELOCITY;

        // consider we reached desired output this is only for testing the pid ang velocity

        run_motors(ANGULAR_CMD, pid_w.correction, -pid_w.correction);

        delay_ms(10);
    }
    run_motors(ANGULAR_CMD, 0, 0);
    TurOff_led();

    return;
}