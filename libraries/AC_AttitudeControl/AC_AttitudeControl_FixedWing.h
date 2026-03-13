#pragma once

/// @file    AC_AttitudeControl_TVBS.h
/// @brief   ArduCopter attitude control library for Fixed Wing

// This is a wrapper around the non-OO attitude controller in ArduPlane to allow FixedWing to use WP_Nav

#include "AC_AttitudeControl.h"
#include "AC_AttitudeControl_Multi.h"

class AC_AttitudeControl_FixedWing : public AC_AttitudeControl_Multi
{
public:
    using AC_AttitudeControl_Multi::AC_AttitudeControl_Multi;

    // Constructor for Fixed Wing needs to take a plane and doesn't need motors
	AC_AttitudeControl_FixedWing(AP_AHRS_View &ahrs, const AP_Vehicle::FixedWing &aparm, float dt);

    // empty destructor to suppress compiler warning
    virtual ~AC_AttitudeControl_FixedWing() {}

    // Ensure attitude controllers have zero errors to relax rate controller output
    // Relax only the roll and yaw rate controllers if exclude_pitch is true
    // These methods were copied from AC_AttitudeControl_TS and probably are not required here
    virtual void relax_attitude_controllers(bool exclude_pitch) override;
    virtual void input_euler_rate_yaw_euler_angle_pitch_bf_roll(bool plane_controls, float body_roll_cd, float euler_pitch_cd, float euler_yaw_rate_cds) override;

    // Methods required to support AR_WPNav and AR_WPNavOA
    float AC_AttitudeControl_FixedWing::get_accel_max();
    float AC_AttitudeControl_FixedWing::get_decel_max();
    float AC_AttitudeControl_FixedWing::get_turn_lat_accel_max();
    void AC_AttitudeControl_FixedWing::get_forward_speed(float &speed);
    float AC_AttitudeControl_FixedWing::get_desired_speed_accel_limited(float desired_speed, float dt);
    float AC_AttitudeControl_FixedWing::get_stopping_distance(float speed);
};
