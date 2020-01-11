/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * control_procyon.cpp - ODTU otopilot yarismasi kodu
 */

#define circle_radius 300.0 // cm
#define circle_center_x 500.0 // distance cm from arm point
#define circle_center_z 500.0 // distance cm from arm point
#define circle_steps 60 // circle update rate

#define initial_altitude 200.0 // altitude to start autonomous mode in cm

const unsigned int step_angle = (360 / circle_steps); // angle between steps

float circle_next_step_x = 0.0f;
float circle_next_step_z = 0.0f;
int step_n = 1;

 // procyon_init - initialise flight mode
static bool procyon_init(bool ignore_checks)
{
    // if not armed or throtle at zero, deny mode
    if (!motors.armed() || g.rc_3.control_in < 50) {
        return false;
    }
    stopAboveArm_init();
    return true;
}

// procyon_run - runs the main controller
// will be called at 100hz or more
static void procyon_run()
{
    // if not armed or throttle at zero, set throttle to zero and exit immediately
    if (!motors.armed()) {
        attitude_control.relax_bf_rate_controller();
        attitude_control.set_yaw_target_to_current_heading();
        attitude_control.set_throttle_out(0, false);
        return;
    }

    // if state completed, move to next
    if (procyon_state_complete) {
        switch (procyon_state) {

        case Procyon_StopAboveArm:
            balletMove_init();
            break;
        /*case Procyon_BalletMove:
            hypotenuseMove_init();
            break;
        case Procyon_HypotenuseMove:
            dejaVuDescent_init();
            break;
        case Procyon_DejaVuDescent:
            fireHole_init();
            break;
        case Procyon_FireHole:
            // closing
            break;*/
        }
    }

    switch (procyon_state) {
    case Procyon_StopAboveArm:
        stopAboveArm_run();
        break;
    case Procyon_BalletMove:
        balletMove_run();
        break;
    /*case Procyon_HypotenuseMove:
        hypotenuseMove_run();
        break;
    case Procyon_DejaVuDescent:
        dejaVuDescent_run();
        break;
    case Procyon_FireHole:
        fireHole_run();
        break;*/
    }
}

// position vector correction
static Vector3f position_relative_to_initial_bearing(int initial_bearing_angle, Vector3f desired_position) {

    Vector3f relative_position = Vector3f(0.0f, 0.0f, desired_position.z);
    int init_rad = initial_bearing_angle * PI / 180;

    relative_position.x = desired_position.x * cos(init_rad) - desired_position.y * sin(init_rad);
    relative_position.y = desired_position.y * cos(init_rad) + desired_position.x * sin(init_rad);

    return relative_position;
}

static void stopAboveArm_init() {

    procyon_state = Procyon_StopAboveArm;
    procyon_state_complete = false;

    wp_nav.wp_and_spline_init();
    Vector3f destination = Vector3f(0.0f, 0.0f, 200.0f); // 200 cm above arm point
    destination = position_relative_to_initial_bearing(initial_armed_bearing, destination);

    wp_nav.set_wp_destination(destination);
    set_auto_yaw_mode(AUTO_YAW_HOLD);
}

static void stopAboveArm_run() {

    /*
    TBD:
    arming check
    landing check

    yaw rate is not needed here.
    */

    wp_nav.update_wpnav();
    pos_control.update_z_controller();

    attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), 0.0f);

    procyon_state_complete = wp_nav.reached_wp_destination();
}


static void balletMove_init() {
    procyon_state_complete = false;
    procyon_state = Procyon_BalletMove;

    Vector3f destination = Vector3f(500.0f, 0.0f, 200.0f); // stop by 500 cm ahead (old dest: 0-0-200)
    destination = position_relative_to_initial_bearing(initial_armed_bearing, destination);
    //wp_nav.set_wp_destination(destination);
}

static void balletMove_run() {
    wp_nav.update_wpnav();
    pos_control.update_z_controller();

    attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), 2000.0f);

    procyon_state_complete = wp_nav.reached_wp_destination();
}


static void hypotenuseMove_init(){
    procyon_state_complete = false;
    procyon_state = Procyon_HypotenuseMove;

    Vector3f destination = Vector3f(0.0f, 0.0f, 700.0f); // stop by 700 cm above (old dest: 500-0-200)
    wp_nav.set_wp_destination(destination);
}

static void hypotenuseMove_run(){
    wp_nav.update_wpnav();
    pos_control.update_z_controller();

    attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), 2000.0f);

    procyon_state_complete = wp_nav.reached_wp_destination();
}


static void dejaVuDescent_init() {
    procyon_state_complete = false;
    procyon_state = Procyon_DejaVuDescent;

    Vector3f destination = Vector3f(0.0f, 0.0f, 200.0f); // get back to st-1 (old dest: 0-0-700)
    wp_nav.set_wp_destination(destination);
}

static void dejaVuDescent_run() {
    wp_nav.update_wpnav();
    pos_control.update_z_controller();

    attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), 2000.0f);

    procyon_state_complete = wp_nav.reached_wp_destination();

}


static void fireHole_init() {
    procyon_state_complete = false;
    procyon_state = Procyon_FireHole;

    update_circle_step(step_n);
    Vector3f destination = Vector3f(circle_next_step_x, 0.0f, circle_next_step_z); // go to first step of circle (approx. 531-0-201)
    wp_nav.set_wp_destination(destination);
}

static void fireHole_run() {

    // update horizontal and vertical controllers
    wp_nav.update_wpnav();
    pos_control.update_z_controller();

    // check if step complete
    if (wp_nav.reached_wp_destination()) {
        step_n++;  // increase step number
        if (step_n >= circle_steps) {  // if steps are complete, return state complete
            procyon_state_complete = true;
            return;
        }
        update_circle_step(step_n);
        Vector3f destination = Vector3f(circle_next_step_x, 0.0f, circle_next_step_z);
        wp_nav.set_wp_destination(destination);
    }

}


static void update_circle_step(unsigned int step) {
    unsigned int desired_circle_angle = step * step_angle;
    circle_next_step_z = sin((desired_circle_angle - 90)*PI / 180 /* rad-to-degrees */ ) * circle_radius + circle_center_z;
    circle_next_step_x = cos((desired_circle_angle - 90)*PI / 180 /* rad-to-degrees */ ) * circle_radius + circle_center_x;
}

