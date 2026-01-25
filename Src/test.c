#include "test.h"
#include "delay.h"
volatile uint16_t encL = 0 ; 
volatile uint16_t encR = 0 ; 
void pwm_test()
{
    for (int d = 0; d <= 99; d++)
    {
        TIM15->CCR1 = d;
        TIM15->CCR2 = d;

        delay_ms(10); // Adjust speed of fade
    }

    delay_ms(500);

    // Fade out (100% to 0%)
    for (int d = 99; d > 0; d--)
    {
        TIM15->CCR1 = d;
        TIM15->CCR2 = d;

        delay_ms(10); // Adjust speed of fade
    }
}
int test_encoder()
{
     encL = TIM1->CNT;
     encR = TIM3->CNT;
     return 0 ;
}

/*

void move_distance(float distance_cm, OdometryTypedef *odo)
{
    float P = 0, I = 0, D = 0;
    float correction_vspeed = 0.0;
    float ramped_setpoint = 0;
    // This starts at 0 and climbs to your desired speed
    float final_target = 40;
    float last_error = 0;

    last = get_current_time_ms();
    while (1)
    {
        now = get_current_time_ms();
        float dt = (float)(now - last) / 1000;
        if (dt <= 0)
            dt = 0.001f;
        last = now;
        OdoUpdate(odo, dt);
        // ramp
        if (ramped_setpoint < final_target)
        {
            ramped_setpoint += ACCEL_RATE * dt;
            if (ramped_setpoint > final_target)
                ramped_setpoint = final_target;
        }
        // If we want to slow down, decrease it
        else if (ramped_setpoint > final_target)
        {
            ramped_setpoint -= ACCEL_RATE * dt;
            if (ramped_setpoint < final_target)
                ramped_setpoint = final_target;
        }

        // --- PID CALCULATION ---
        // IMPORTANT: We calculate error against the RAMP, not the final 50.
        error = ramped_setpoint - odo->v;
        // error = desired_motor_speed.value.linear_speed - odo->v; //
        P = error;
        // Deadzone for I to prevent jitter at high speed
        if (fabs(error) > 0.5f)
        {
            I += error * dt;
        }
        // I += error * dt;
        // anti windup for i
        if (I > 20.0f)
            I = 20.0f;
        if (I < -20.0f)
            I = -20.0f;
        D = (error - last_error) / dt;
        last_error = error;
        correction = kp * P + I * ki + kd * D;

        // //
        // if (correction > MAX_VELOCITY)
        //     correction = MAX_VELOCITY;
        // if (correction < -MAX_VELOCITY)
        //     correction = -MAX_VELOCITY;

        if (fabs(error) <= 0.1f)
        {
            run_motors(0, 0); // stop
            TurOff_led();
            break;
        }
        run_motors(correction, correction);

        delay_ms(10);
    }
}
*/