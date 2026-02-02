#include "odometry.h"
#include "encoder.h"
#include <math.h>
#include <string.h> 
#define MAX_AVG_SPEED_COUNT 20
volatile uint16_t prev_left_encoder = 0;
volatile uint16_t prev_right_encoder = 0;
volatile uint16_t current_left = 0;
volatile uint16_t current_right = 0;
volatile int16_t left_delta = 0;
volatile int16_t right_delta = 0;
float total_angle_rad = 0;
float total_left_distance = 0;
float total_right_distance = 0;
float total_distance = 0, current_distance = 0;
float theta_deg = 0, theta_rad = 0;

float avg_linear[MAX_AVG_SPEED_COUNT];
float avg_angular[MAX_AVG_SPEED_COUNT];
int arr_index_avg = 0;
int max_mumber_reads = 0;
static float rad_to_deg(double x);
void OdoInit(OdometryTypedef *odo)
{
    *odo = (OdometryTypedef){0};
    memset(odo,0, sizeof(OdometryTypedef)); 
    // odo->x = 0;
    // odo->y = 0;
    odo->angle_rad = 0;
    odo->angle_deg = 0;
    // odo->v = 0;
    // odo->w = 0;
    // odo->left_speed = 0;
    // odo->right_speed = 0;
    odo->total_distance = 0;
    // odo->current_distance = 0;
    prev_left_encoder = 0;
    prev_right_encoder = 0;
    total_left_distance = 0;
    total_right_distance = 0;
    for (int i = 0; i < MAX_AVG_SPEED_COUNT; i++)
    {
        avg_angular[i] = 0;
        avg_angular[i] = 0;
    }
    arr_index_avg = 0;
    max_mumber_reads = 0;
    current_left = 0;
    current_right = 0;
    left_delta = 0;
    right_delta = 0;
    total_angle_rad = 0;
    total_left_distance = 0;
    total_right_distance = 0;
    total_distance = 0;
    theta_deg = 0;
    theta_rad = 0;
    TIM1->CNT = 0 ; 
    TIM3->CNT = 0 ; 
     
    OdoUpdate(odo, 0);
}

void OdoUpdate(OdometryTypedef *odo, float dt)
{

    // Read current encoder values with rollover handling
    current_left = timer1_LeftEncoder_Read();
    current_right = timer3_RightEncoder_Read();

    // Calculate delta with proper rollover handling
    left_delta = (current_left - prev_left_encoder);
    right_delta = (current_right - prev_right_encoder);
    // Enhanced rollover detection
    if (left_delta > 30000)
    { // backward movement
        left_delta -= 65536;
    }
    else if (left_delta < -30000)
    { // forward movement
        left_delta += 65536;
    }

    if (right_delta > 30000)
    {
        right_delta -= 65536;
    }
    else if (right_delta < -30000)
    {
        right_delta += 65536;
    }

    // Store previous values
    prev_left_encoder = current_left;
    prev_right_encoder = current_right;

    odo->left_encoder_delta = left_delta;
    odo->right_encoder_delta = right_delta;
    // ==== distace calculation  ==================
    // Calculate distances
    float left_distance = left_delta * DISTANCE_PER_PULSE_CM;
    float right_distance = right_delta * DISTANCE_PER_PULSE_CM;
    // odo->left_distance = left_distance;
    // odo->right_distance = right_distance;
    // Update cumulative distances
    total_left_distance += left_distance;
    total_right_distance += right_distance;

    // Calculate displacements
    current_distance = (left_distance + right_distance) / 2.0f;
    total_distance += current_distance;
    // =========================== end distance ===============================

    // ====== calculation of cooridnates (x,y) ==========================
    odo->x += current_distance * cos(odo->angle_rad);
    odo->y += current_distance * sin(odo->angle_rad);
    // ====== end calculation of cooridnates (x,y) ==========================

    // ====== calculation of angle ==========================
    float angle_theta = (right_distance - left_distance) / (WHEELBASE_CM); //  to make evrything in CM
    odo->angle_rad += angle_theta;
    // normalize angle
    while (odo->angle_rad > PI)
    {
        odo->angle_rad -= 2 * PI;
    }
    while (odo->angle_rad < -PI)
    {
        odo->angle_rad += 2 * PI;
    }
    odo->angle_deg = rad_to_deg(odo->angle_rad);
    // ====== END calculation of angle ==========================
    // ====== calculation of sped  ==========================
    float vL = 0.0f;
    float vR = 0.0f;
    float w = 0.0f;
    if (dt > 0.0f)
    {
        vL = (left_distance / dt);  // cm/s
        vR = (right_distance / dt); // cm/s
        w = (angle_theta / dt);     // rad/s
        // float right_speed = (right_encoder_speed + left_encoder_speed)/2 + odo->w * WHEELBASE_CM/2;
        // float left_speed = (right_encoder_speed + left_encoder_speed)/2 - odo->w * WHEELBASE_CM/2;
    }

    float v = (vL + vR) / 2;
    avg_linear[arr_index_avg] = v;
    avg_angular[arr_index_avg] = w;

    arr_index_avg = (arr_index_avg + 1) % MAX_AVG_SPEED_COUNT;
    float tmpv = 0;
    float tmpw = 0;
    if (max_mumber_reads < MAX_AVG_SPEED_COUNT)
        max_mumber_reads++;
    for (int i = 0; i < max_mumber_reads; i++)
    {
        tmpv += avg_linear[i];
        tmpw += avg_angular[i];
    }
    odo->v = (float)(tmpv / max_mumber_reads);
    odo->w = (float)(tmpw / max_mumber_reads);
    // ====== END calculation of sped  ==========================

    odo->total_distance = total_distance;
    odo->current_distance = current_distance;
}

void OdoResetPosition(OdometryTypedef *odo)
{
    odo->x = 0;
    odo->y = 0;
    odo->angle_deg = 0;
    odo->angle_rad = 0;
    total_left_distance = 0;
    total_right_distance = 0;
}
static float rad_to_deg(double x)
{
    return (x * 360 / (2 * PI));
}