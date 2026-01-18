typedef struct {
    float max_vel;      // Maximum velocity
    float max_accel;    // Maximum acceleration
    float max_decel;    // Maximum deceleration (can = max_accel)
    
    float current_pos;  // Current position/angle
    float target_pos;   // Target position/angle
    float current_vel;  // Current velocity
    
    float accel_dist;   // Distance needed to accelerate
    float decel_dist;   // Distance needed to deceler
    float cruise_dist;  // Distance at constant velocity
    
    enum {ACCEL, CRUISE, DECEL, DONE} phase;
} TrapezoidalProfile;

void Profile_Init(TrapezoidalProfile* prof, float max_v, float max_a) {
    prof->max_vel = max_v;
    prof->max_accel = max_a;
    prof->max_decel = max_a;
    prof->phase = DONE;
}

void Profile_SetTarget(TrapezoidalProfile* prof, float current, float target) {
    prof->current_pos = current;
    prof->target_pos = target;
    prof->current_vel = 0;
    
    float distance = fabs(target - current);
    float direction = (target > current) ? 1.0f : -1.0f;
    
    // Calculate if we can reach max velocity
    float vmax_reachable = sqrtf(prof->max_accel * distance);
    float use_vel = fminf(prof->max_vel, vmax_reachable);
    
    // Phase distances
    prof->accel_dist = (use_vel * use_vel) / (2.0f * prof->max_accel);
    prof->decel_dist = (use_vel * use_vel) / (2.0f * prof->max_decel);
    prof->cruise_dist = distance - prof->accel_dist - prof->decel_dist;
    
    if(prof->cruise_dist < 0) {
        // Triangular profile (no cruise phase)
        prof->accel_dist = distance * prof->max_decel / (prof->max_accel + prof->max_decel);
        prof->decel_dist = distance - prof->accel_dist;
        prof->cruise_dist = 0;
        use_vel = sqrtf(2.0f * prof->max_accel * prof->accel_dist);
    }
    
    prof->phase = ACCEL;
}

float Profile_Update(TrapezoidalProfile* prof, float dt) {
    float distance_left = fabs(prof->target_pos - prof->current_pos);
    
    switch(prof->phase) {
        case ACCEL:
            prof->current_vel += prof->max_accel * dt;
            if(prof->current_vel >= prof->max_vel || 
               fabs(prof->current_pos - prof->target_pos) <= prof->accel_dist) {
                prof->phase = (prof->cruise_dist > 0) ? CRUISE : DECEL;
                prof->current_vel = fminf(prof->current_vel, prof->max_vel);
            }
            break;
            
        case CRUISE:
            if(distance_left <= prof->decel_dist) {
                prof->phase = DECEL;
            }
            break;
            
        case DECEL:
            prof->current_vel -= prof->max_decel * dt;
            if(prof->current_vel < 0) prof->current_vel = 0;
            break;
            
        case DONE:
            return 0;
    }
    
    // Apply velocity with correct sign
    float direction = (prof->target_pos > prof->current_pos) ? 1.0f : -1.0f;
    prof->current_pos += prof->current_vel * dt * direction;
    
    // Check if reached target
    if(distance_left < 0.001f) {
        prof->phase = DONE;
        return 0;
    }
    
    return prof->current_vel * direction;
}