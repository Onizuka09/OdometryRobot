#ifndef _ODOMETRY_H__
#define _ODOMETRY_H__
#include <stdint.h> 
#define WHEELBASE_CM 21 
#define PI                    3.14159265358979323846f  
#define WHEEL_DIAMETER_CM 6.5f      
#define WHEEL_RADIUS_CM (WHEEL_DIAMETER_CM / 2.0f)      
// number of pulses each full  wheel rotation 
#define PULSE_PER_REVOLUTION 224 
#define PRECISION 4 
#define DISTANCE_PER_PULSE_CM   ((PI * WHEEL_DIAMETER_CM )/(PULSE_PER_REVOLUTION* PRECISION))
#define PERIOD 10 // period to call the odometry update function
// #define MM_PER_COUNT (2 * 31415 * WHEEL_RADIUS_MM / PULSE_PER_REVOLUTION) // 2πr/cpr


typedef struct {
    // Position and Orientation
    float x;                    // X position in cm (relative to start)
    float y;                    // Y position in cm (relative to start)
    float angle_rad;               // Orientation in radians (-PI to PI)
    float angle_deg;               // Orientation in radians (-PI to PI)
    
    // Velocities
    float v;      // Linear velocity in cm/s
    float w;     // Angular velocity in rad/s
    float left_speed;          // Left wheel speed in cm/s
    float right_speed;         // Right wheel speed in cm/s
    
    // Encoder Data
    int16_t left_encoder_delta;  // Left encoder ticks since last update
    int16_t right_encoder_delta; // Right encoder ticks since last update
    
    // Cumulative Measurements
    float total_distance;      // Total distance traveled in cm
    float current_distance ;   // the current traveled distance (each 10 ms)
    // Additional useful metrics (like first implementation)
    float left_distance;       // Left wheel distance this update
    float right_distance;      // Right wheel distance this update
    
} OdometryTypedef;



void OdoInit(OdometryTypedef* odo);
void OdoReset(OdometryTypedef* odo);
void OdoUpdate(OdometryTypedef* odo, float dt);

#endif