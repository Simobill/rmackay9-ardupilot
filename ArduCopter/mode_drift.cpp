#include "Copter.h"

#if MODE_DRIFT_ENABLED == ENABLED

/*
 * Init and run calls for drift flight mode
 */

#ifndef DRIFT_SPEEDGAIN
 # define DRIFT_SPEEDGAIN 8.0f
#endif
#ifndef DRIFT_SPEEDLIMIT
 # define DRIFT_SPEEDLIMIT 560.0f
#endif

#ifndef DRIFT_THR_ASSIST_GAIN
 # define DRIFT_THR_ASSIST_GAIN 0.0018f    // gain controlling amount of throttle assistance
#endif

#ifndef DRIFT_THR_ASSIST_MAX
 # define DRIFT_THR_ASSIST_MAX  0.3f    // maximum assistance throttle assist will provide
#endif

#ifndef DRIFT_THR_MIN
 # define DRIFT_THR_MIN         0.213f  // throttle assist will be active when pilot's throttle is above this value
#endif
#ifndef DRIFT_THR_MAX
 # define DRIFT_THR_MAX         0.787f  // throttle assist will be active when pilot's throttle is below this value
#endif

// drift_init - initialise drift controller
bool Copter::ModeDrift::init(bool ignore_checks)
{
    if (copter.position_ok() || ignore_checks) {
        return true;
    } else {
        return false;
    }
}

// drift_run - runs the drift controller
// should be called at 100hz or more
void Copter::ModeDrift::run()
{
    static float braker = 0.0f;
    static float roll_input = 0.0f;

    // convert pilot input to lean angles
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, copter.aparm.angle_max);

    // grab velocity
    const Vector3f& vel = inertial_nav.get_velocity();

    // rotate roll, pitch input from north facing to vehicle's perspective
    float roll_vel =  vel.y * ahrs.cos_yaw() - vel.x * ahrs.sin_yaw(); // body roll vel
    float pitch_vel = vel.y * ahrs.sin_yaw() + vel.x * ahrs.cos_yaw(); // body pitch vel

    // gain scheduling for yaw
    float pitch_vel2 = MIN(fabsf(pitch_vel), 2000);
    float target_yaw_rate = ((float)target_roll/1.0f) * (1.0f - (pitch_vel2 / 5000.0f)) * g.acro_yaw_p;

    roll_vel = constrain_float(roll_vel, -DRIFT_SPEEDLIMIT, DRIFT_SPEEDLIMIT);
    pitch_vel = constrain_float(pitch_vel, -DRIFT_SPEEDLIMIT, DRIFT_SPEEDLIMIT);

    roll_input = roll_input * .96f + (float)channel_yaw->get_control_in() * .04f;

    // convert user input into desired roll velocity
    float roll_vel_error = roll_vel - (roll_input / DRIFT_SPEEDGAIN);

    // roll velocity is feed into roll acceleration to minimize slip
    target_roll = roll_vel_error * -DRIFT_SPEEDGAIN;
    target_roll = constrain_float(target_roll, -4500.0f, 4500.0f);

    // If we let go of sticks, bring us to a stop
    if (is_zero(target_pitch)){
        // .14/ (.03 * 100) = 4.6 seconds till full braking
        braker += .03f;
        braker = MIN(braker, DRIFT_SPEEDGAIN);
        target_pitch = pitch_vel * braker;
    } else {
        braker = 0.0f;
    }

    // get pilot's desired throttle
    float pilot_throttle_scaled = get_pilot_desired_throttle(channel_throttle->get_control_in());

    if (!motors->armed()) {
        motors->set_desired_spool_state(AP_Motors::DESIRED_SHUT_DOWN);
        attitude_control->set_yaw_target_to_current_heading();
        attitude_control->reset_rate_controller_I_terms();
    } else if (ap.throttle_zero || (!motors->get_interlock() && ap.using_interlock)) {
        // This is designed to preserve flying functions in the case of inflight motor interlock disabled.
        if (ap.land_complete) {
            attitude_control->set_yaw_target_to_current_heading();
            attitude_control->reset_rate_controller_I_terms();
        }
        motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
    } else {
        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
        if (motors->get_spool_mode() == AP_Motors::THROTTLE_UNLIMITED) {
            set_land_complete(false);
        }
    }

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

    // output pilot's throttle with angle boost
    attitude_control->set_throttle_out(get_throttle_assist(vel.z, pilot_throttle_scaled), true, g.throttle_filt);
}

// get_throttle_assist - return throttle output (range 0 ~ 1) based on pilot input and z-axis velocity
float Copter::ModeDrift::get_throttle_assist(float velz, float pilot_throttle_scaled)
{
    // throttle assist - adjusts throttle to slow the vehicle's vertical velocity
    //      Only active when pilot's throttle is between 213 ~ 787
    //      Assistance is strongest when throttle is at mid, drops linearly to no assistance at 213 and 787
    float thr_assist = 0.0f;
    if (pilot_throttle_scaled > DRIFT_THR_MIN && pilot_throttle_scaled < DRIFT_THR_MAX) {
        // calculate throttle assist gain
        thr_assist = 1.2f - ((float)fabsf(pilot_throttle_scaled - 0.5f) / 0.24f);
        thr_assist = constrain_float(thr_assist, 0.0f, 1.0f) * -DRIFT_THR_ASSIST_GAIN * velz;

        // ensure throttle assist never adjusts the throttle by more than 300 pwm
        thr_assist = constrain_float(thr_assist, -DRIFT_THR_ASSIST_MAX, DRIFT_THR_ASSIST_MAX);
    }
    
    return constrain_float(pilot_throttle_scaled + thr_assist, 0.0f, 1.0f);
}
#endif
