/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include "Copter.h"
#include <AC_PID/AC_PI_2D.h>


/*
 * control_guidednogps.pde - init and run calls for guidednogps, flight mode
 */

#define GUIDEDNOGPS_ATTITUDE_TIMEOUT_MS  1000    // guidednogps mode's attitude controller times out after 1 second with no new updates
#define GUIDEDNOGPS_PID_P                   3.00
#define GUIDEDNOGPS_PID_I                   0.10
#define GUIDEDNOGPS_PID_D                   20.0
#define GUIDEDNOGPS_PID_IMAX                 500
#define GUIDEDNOGPS_PID_FILT_HZ             20.0
struct {
    uint32_t update_time_ms;
    float roll_cd;
    float pitch_cd;
    float yaw_cd;
    float climb_rate_cms;
} static guidednogps_angle_state = {0,0.0f, 0.0f, 0.0f, 0.0f};  // Stores attitude input controls

static AC_PID guidednogps_pid_x(GUIDEDNOGPS_PID_P,
                                GUIDEDNOGPS_PID_I,
                                GUIDEDNOGPS_PID_D,
                                GUIDEDNOGPS_PID_IMAX,
                                GUIDEDNOGPS_PID_FILT_HZ,
                                MAIN_LOOP_SECONDS);

static AC_PID guidednogps_pid_y(GUIDEDNOGPS_PID_P,
                                GUIDEDNOGPS_PID_I,
                                GUIDEDNOGPS_PID_D,
                                GUIDEDNOGPS_PID_IMAX,
                                GUIDEDNOGPS_PID_FILT_HZ,
                                MAIN_LOOP_SECONDS);

// guidednogps_init - initialise guidednogps controller
bool Copter::guidednogps_init(bool ignore_checks)
{
#if FRAME_CONFIG == HELI_FRAME
    // do not allow helis to enter Guided NOGPS if the Rotor Runup is not complete
    if (!ignore_checks && !motors.rotor_runup_complete()){
        return false;
    }
#endif

    // initialize vertical speeds and leash lengths
    pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control.set_accel_z(g.pilot_accel_z);

    // initialise position and desired velocity
    pos_control.set_alt_target(inertial_nav.get_altitude());
    pos_control.set_desired_velocity_z(inertial_nav.get_velocity_z());

    // stop takeoff if running
    takeoff_stop();

    // Initialize the angle control
    guidednogps_angle_state.update_time_ms = millis();

    // Initialize the pid controller (for relative position input)
    guidednogps_pid_x.reset_I();
    guidednogps_pid_y.reset_I();

    return true;
}

// alt-hold angle target
void Copter::guidednogps_set_angle(const Quaternion &q, float climb_rate_cms)
{
    // convert quaternion to euler angles
    q.to_euler(guidednogps_angle_state.roll_cd, guidednogps_angle_state.pitch_cd, guidednogps_angle_state.yaw_cd);
    guidednogps_angle_state.roll_cd = ToDeg(guidednogps_angle_state.roll_cd) * 100.0f;
    guidednogps_angle_state.pitch_cd = ToDeg(guidednogps_angle_state.pitch_cd) * 100.0f;
    guidednogps_angle_state.yaw_cd = wrap_180_cd_float(ToDeg(guidednogps_angle_state.yaw_cd) * 100.0f);

    guidednogps_angle_state.climb_rate_cms = climb_rate_cms;
    guidednogps_angle_state.update_time_ms = millis();

    // TODO: this should have its own logging function, but we use the guided attitude control for now
    Log_Write_GuidedTarget(Guided_Angle,
        Vector3f(guidednogps_angle_state.roll_cd, guidednogps_angle_state.pitch_cd, guidednogps_angle_state.yaw_cd),
        Vector3f(0.0f, 0.0f, guidednogps_angle_state.climb_rate_cms));
}

// guidednogps_run - runs the guidednogps controller
// should be called at 100hz or more
void Copter::guidednogps_run()
{
    GuidedNoGPSModeState guidednogps_state;
    float takeoff_climb_rate = 0.0f;

    // initialize vertical speeds and acceleration
    pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control.set_accel_z(g.pilot_accel_z);

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // Target roll, pitch, yaw
    float target_roll, target_pitch;

    // Desired climb rate
    float target_climb_rate;

#if FRAME_CONFIG == HELI_FRAME
    // helicopters are held on the ground until rotor speed runup has finished
    bool takeoff_triggered = (ap.land_complete && motors.rotor_runup_complete());
#else
    bool takeoff_triggered = (ap.land_complete && motors.spool_up_complete());
#endif

    //For external input
    float target_yaw = ahrs.yaw_sensor;

    // Guided NOGPS State Machine Determination
    if (!motors.armed() || !motors.get_interlock()) {
        guidednogps_state = GuidedNoGPS_MotorStop;
    } else if (!ap.auto_armed){
        guidednogps_state = GuidedNoGPS_Disarmed;
    } else if (takeoff_state.running || takeoff_triggered){
        guidednogps_state = GuidedNoGPS_Takeoff;
    } else if (ap.land_complete){
        guidednogps_state = GuidedNoGPS_Landed;
    } else {
        guidednogps_state = GuidedNoGPS_Flying;
    }

    Vector2f target_xy;
    Vector2f cmd;

    // Determine if we have new attitude input commands
    uint32_t tnow = millis();
    if (tnow - guidednogps_angle_state.update_time_ms < GUIDEDNOGPS_ATTITUDE_TIMEOUT_MS) {
        // constrain desired lean angles
        target_roll = guidednogps_angle_state.roll_cd;
        target_pitch = guidednogps_angle_state.pitch_cd;

        // wrap yaw request
        target_yaw = wrap_180_cd_float(guidednogps_angle_state.yaw_cd);
    } else {
        target_roll = target_pitch = 0;
        target_climb_rate = 0;
#if PRECISION_LANDING == ENABLED
        //Check for input target data
        if(precland.get_target_rel_pos_xy(target_xy)) {
            //Run the controller
            guidednogps_pid_x.set_input_filter_d(target_xy.x);
            guidednogps_pid_y.set_input_filter_d(target_xy.y);
            cmd.x = guidednogps_pid_x.get_pid();
            cmd.y = guidednogps_pid_y.get_pid();
            target_roll = cmd.x;
            target_pitch = -cmd.y;
        } else {
            guidednogps_pid_x.reset_I();
            guidednogps_pid_y.reset_I();
        }
#endif
    }

    //Constrain lean angles
    float total_in = pythagorous2(target_roll, target_pitch);
    float angle_max = attitude_control.get_althold_lean_angle_max();
    if (total_in > angle_max) {
      float ratio = angle_max / total_in;
      target_roll *= ratio;
      target_pitch *= ratio;
    }

    // constrain climb rate to nav rate
    target_climb_rate = constrain_float(guidednogps_angle_state.climb_rate_cms, -fabs(wp_nav.get_speed_down()), wp_nav.get_speed_up());

    Vector3f att_target(target_pitch, target_roll, target_yaw);
    Log_Write_GuidedNoGPS(guidednogps_state, target_xy, guidednogps_pid_x.get_p(), guidednogps_pid_x.get_d(), guidednogps_pid_x.get_i(), guidednogps_pid_y.get_p(), guidednogps_pid_y.get_d(), guidednogps_pid_y.get_i(), att_target, target_climb_rate);

    // Guided NOGPS State Machine
    switch (guidednogps_state) {

    case GuidedNoGPS_Disarmed:

#if FRAME_CONFIG == HELI_FRAME  // Helicopters always stabilize roll/pitch/yaw
        attitude_control.set_yaw_target_to_current_heading();
        // call attitude controller
        attitude_control.input_euler_angle_roll_pitch_yaw(target_roll, target_pitch, target_yaw, true);
        attitude_control.set_throttle_out(0,false,g.throttle_filt);
#else
        motors.set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        // Multicopter do not stabilize roll/pitch/yaw when disarmed
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
#endif  // HELI_FRAME
        pos_control.relax_alt_hold_controllers(get_throttle_pre_takeoff(channel_throttle->control_in)-throttle_average);
        break;

    case GuidedNoGPS_MotorStop:

#if FRAME_CONFIG == HELI_FRAME    
        // helicopters are capable of flying even with the motor stopped, therefore we will attempt to keep flying
        // call attitude controller
        attitude_control.input_euler_angle_roll_pitch_yaw(target_roll, target_pitch, target_yaw, true);

        // force descent rate and call position controller
        pos_control.set_alt_target_from_climb_rate(-abs(g.land_speed), G_Dt, false);
        pos_control.update_z_controller();
#else   // Multicopter do not stabilize roll/pitch/yaw when motor are stopped
        motors.set_desired_spool_state(AP_Motors::DESIRED_SHUT_DOWN);
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
        pos_control.relax_alt_hold_controllers(get_throttle_pre_takeoff(channel_throttle->control_in)-throttle_average);
#endif  // HELI_FRAME
        break;

    case GuidedNoGPS_Takeoff:

        // initiate take-off
        if (!takeoff_state.running) {
            takeoff_timer_start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
            // indicate we are taking off
            set_land_complete(false);
            // clear i terms
            set_throttle_takeoff();
        }

        // get take-off adjusted pilot and takeoff climb rates
        takeoff_get_climb_rates(target_climb_rate, takeoff_climb_rate);

        // set motors to full range
        motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

        // call attitude controller
        attitude_control.input_euler_angle_roll_pitch_yaw(target_roll, target_pitch, target_yaw, true);

        // call position controller
        pos_control.set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        pos_control.add_takeoff_climb_rate(takeoff_climb_rate, G_Dt);
        pos_control.update_z_controller();
        break;

    case GuidedNoGPS_Landed:

#if FRAME_CONFIG == HELI_FRAME  // Helicopters always stabilize roll/pitch/yaw
        attitude_control.set_yaw_target_to_current_heading();
        // call attitude controller
        attitude_control.input_euler_angle_roll_pitch_yaw(target_roll, target_pitch, target_yaw, true);
        attitude_control.set_throttle_out(get_throttle_pre_takeoff(channel_throttle->control_in),false,g.throttle_filt);
#else   // Multicopter stabilize roll/pitch/yaw when landed
        attitude_control.set_throttle_out(get_throttle_pre_takeoff(channel_throttle->control_in),false,g.throttle_filt);
        // if throttle zero reset attitude and exit immediately
        if (ap.throttle_zero) {
            motors.set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        }else{
            motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
        }
#endif
        pos_control.relax_alt_hold_controllers(get_throttle_pre_takeoff(channel_throttle->control_in)-throttle_average);
        break;

    case GuidedNoGPS_Flying:
        motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
        // call attitude controller
        //attitude_control.input_euler_angle_roll_pitch_yaw(target_roll, target_pitch, target_yaw, true);
        attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, 0);

        // call throttle controller
        if (sonar_enabled && (sonar_alt_health >= SONAR_ALT_HEALTH_MAX)) {
            // if sonar is ok, use surface tracking
            target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control.get_alt_target(), G_Dt);
        }

        // call position controller
        pos_control.set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        pos_control.update_z_controller();
        break;
    }
}
