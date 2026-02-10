#ifndef NAVIGATION_H__
#define NAVIGATION_H__
/*
 * @brief: robot navigation
 * move_distance --- move_angle
 *                |
 *                |
 *           move_distance
 *                |
 *                |
 *       motor control (pwm)
 *
 */
#include <stdint.h>
#include "odometry.h"
typedef enum
{
    ANGULAR_CMD=0,
    POSITION_CMD,
} Command_type;
/*
   - absolute angle: control therobot needs face the desired angle (world frame orientation)
   - relative angle: incremental rotation 
*/
typedef enum { 
    ABSOLUTE_ANGLE , 
    RELATIVE_ANGLE, 
}ANGLE_CONTORL_t; 
typedef struct
{

    union
    {
        float angular_speed; // rad/s
        float linear_speed;  // cm/s
    } value;
} speed_t;

void move_angular_speed(float desired_motor_ang_speed, OdometryTypedef *odo);

void move_linear_speed(float desired_motor_speed, OdometryTypedef *odo);
void position_control(int16_t desired_distance, OdometryTypedef *odo);

/** 
* @brief robot angle control
* @param desired_angle target angle in rad
* @param odo            odometry struct 
*/
void angle_control(ANGLE_CONTORL_t angle_type , float desired_angle, OdometryTypedef *odo);

#endif