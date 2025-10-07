#pragma once

#include <AP_Math/AP_Math.h>

struct OABendyType {
    Vector2f avoidance_vec;
    Vector2f origin;
    float nearest_distance;
    uint8_t obstacle_type;
    
    void reset() {
        avoidance_vec.zero();
        origin.zero();
        nearest_distance = 0;
        obstacle_type = 0;
    }
};