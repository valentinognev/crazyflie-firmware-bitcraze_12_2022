/**
ZVUV
 */
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"
#include "static_mem.h"
#include "system.h"
#include <string.h> //to enable memset command
#include "app.h"
#include "log.h"
#include "debug.h"
#include "mr18_driver.h"
#include "commander.h"
#include "param.h"
#include <math.h>
#include "state_machine.h"
#include "motors.h"
#include "telemetry.h"

#define DEBUG_MODULE "STATE_MACHINE"

#ifndef MR18_APP_STACKSIZE
#define MR18_APP_STACKSIZE 500
#endif

#ifndef MR18_APP_PRIORITY
#define MR18_APP_PRIORITY 3
#endif

#ifndef TELEMETRY_APP_STACKSIZE
#define TELEMETRY_APP_STACKSIZE 100
#endif

#ifndef TELEMETRY_APP_PRIORITY
#define TELEMETRY_APP_PRIORITY 0
#endif

#define CLAMP(x, TH)  (((x) > (TH)) ? (TH) : (((x) < (-TH)) ? (-TH) : (x)))
#define PI 3.141592653F

//external setpoint parameters
#ifdef INVERTED_FLIGHT_MODE
    static const float landing_height = -1.7f; //Aron: should be a param dependent on flight scenario
    static float height_sp = -0.35f; // can be updated via param
    static uint8_t LD_Mode = 0; // LU
#else
    static const float landing_height = 0.1f; //Aron: should be a param dependent on flight scenario
    static float height_sp = 0.35f; // can be updated via param
    static uint8_t LD_Mode = 1; // LD
#endif
#define MAX(a, b) ((a>b)?a:b)
#define MIN(a, b) ((a<b)?a:b)
#define SIN_AMP 15.0F
#define SIN_FREQ 0.5F
#define VNOM_FLT_ALPHA 0.1F

static void BIT1();
static void BIT2();
static void autonomous_phase_1();
static void test_kalman1_converged();
static void test_kalman2_converged();
static void strobe(const int16_t strobe_power, const uint16_t rise_time, const uint16_t wait_time);
static void loco_rtn_home();
static void loco_goto_wp();

static State state = idle;
static Phase1_SubState Phase1_SubState_var = starting;
static RandomExp_SubState RandomExp_SubState_var = EXP_go;
static SM_State SM_State_var = SM_loco_rtn_home;// SM_takeoff_N_stab; //ARON: REMOVE BEFORE MERGE
static SM_State SM_State_var_prev = SM_loco_rtn_home;// SM_takeoff_N_stab; //ARON: REMOVE BEFORE MERGE

static int8_t timeout = 0; 
static int8_t Bolt_Prio = 4; // Active_Autonomous=4 / CRTP=3 (stronger) / Bolt_SM=2 / Nano_SM = 1
static int8_t OPER_flag = 0;
static int8_t safety_fa = 0; 
// -1 = Unable to Arm, Stabilizer Stopped, Release and Try Again, Otherwise Reboot Bolt
// -2 = Unable to Arm, Cap Missing, Return Cap and Try Again
// -3 = Unable to Arm, Stabilizer Not Ready, Complete PBIT and Try Again
// -4 = Take-off Aborted due to Cap Abort, Return Cap and Try Again 
// -5 = Take-off Aborted due to Stabilizer Stop, Release and Try Again, Otherwise Reboot Bolt
// -6 = Arm aborted due to timeout
// -7 = Unable to Arm, Cap On
// -8 = Running BIT1

static uint8_t emergencyStop = 0;
static uint8_t launch_flag = 0;
static uint8_t mr_init_val = 3; // 0=default / 1=mr5 connected / 2=mr18 connected / 3=no ranger
static uint8_t state_entry = 0;
static uint8_t Stopped_Rise_Flag = 0;
static uint8_t Stopped_Prev = 0;
static uint8_t stab_stop=0;
static uint8_t stab_ready=0;
static uint8_t allw_OF_ret = 0;
static uint8_t allw_Z_ret = 0;
static uint8_t allw_W_ret = 0;
static uint8_t leg_completed = 0;
static uint8_t trn_arnd_fl = 0;
static uint8_t go_bkwrd_fl = 0;
static uint8_t sudden_w_change = 0;
static uint8_t sudden_z_change = 0;
static uint8_t OF_issue = 0;
static uint8_t sudden_w_change_recovery = 1;
static uint8_t sudden_z_change_recovery = 1;
static uint8_t OF_issue_recovery = 1;
static uint8_t allw_OA = 0;
static uint8_t allw_hover = 0;
static uint8_t TstSlwDwn = 0;
static uint8_t throttleCoef = 0;  //ii
static uint8_t cap_counter = 0;
static uint8_t cap_counter_th = 9;
static uint8_t MAN_AUTO_PROPTEST = 1; // 0 = Manual / 1 = Auto / 2 = Prop. Test
static uint8_t led_power = 0; // default
static uint8_t BIT1_PASS = 0; // This will include the following tests: shutter, mr18
static uint8_t BIT2_PASS = 0; // This will include the following tests: prop, h_est, v_est, mr18
static uint8_t motorPass = 0;
static uint8_t ExtKalman = 1; // 1 = kalman1 , 2 = kalman2
static uint8_t PositionLoop = 0; // 0 = Velocity Loop , 1 = Position Loop
static uint8_t kalman1_converged = 0;
static uint8_t kalman2_converged = 0;
static uint8_t reached_final_wp = 0;
static uint8_t loco_ii = 0;
static uint8_t waypoint_timer_on = 0;

static uint16_t id_psi_cmd = 0; //ToDo: change to uint16_t and test
static uint16_t id_vx_cmd = 0; //ToDo: change to uint16_t and test
static uint16_t id_vy_cmd = 0; //ToDo: change to uint16_t and test
static uint16_t id_vx_nom = 0; //ToDo: change to uint16_t and test
static uint16_t id_vy_nom = 0; //ToDo: change to uint16_t and test
static uint16_t id_OF_OL = 0; //ToDo: change to uint16_t and test
static uint16_t id_OF_IL = 0; //ToDo: change to uint16_t and test

static int16_t strobe_power_high = 0;
static int16_t strobe_power_med = 200;
static uint16_t rise_time_slow = 2000; // msec
static uint16_t rise_time_fast = 450; // msec
static uint16_t wait_time_slow = 500; // msec
static uint16_t wait_time_fast = 50; // msec
static uint32_t hover_time = 3000; // msec

static uint16_t id_Stopped=0;
static uint16_t id_z_est = 0;
static uint16_t id_stab_stop=0;
static uint16_t id_stab_ready=0;
static uint16_t id_bottom_tof = 0;
static uint16_t id_safety_tof = 0;
static uint16_t bottom_tof = 0;
static uint16_t safety_tof = 0;
static uint16_t safety_tof_th = 45; // mm
static uint16_t flr_clrnce = 80; //mm
static uint16_t catch_th = 150; //mm
static uint16_t hd_idle_throttle = 15000;
static uint16_t idle_throttle = 6500;
static uint16_t id_SlwDwnFct = 0;
static uint16_t id_motorTestCount = 0;
static uint16_t id_motorPass = 0;
static uint16_t motorTestCount = 0;

static uint32_t estop_time = 0;
static uint32_t launch_time = 0;
static uint32_t trigger_time = 0;
static uint32_t scan_time = 0;
static uint32_t d_time = 0;
static uint32_t stab_time = 700; //msec = 400 (throttle up) + 300 (angular rise time OR the time it takes a good est to diverge)
static uint32_t trigger_dt = 6000;
static uint32_t prep_turn_dt = 400;
            
static float psi_cmd = 0;
static float vx_cmd = 0;
static float vy_cmd = 0;
static float vx_nom = 0;
static float vy_nom = 0;
static float vx_sp = 0.0f; // can be updated via param
static float vy_sp = 0.0f; // can be updated via param
static float psi_sp = 0.0f; // can be updated via param
static float PNG_k_psi = 0.01f; // can be updated via param
static float max_psi_cmd = 70.0f; // can be updated via param
static float lag_timer = 10000.0f; // can be updated via param
static float yawSpeed = 0;
static float velSide = 0;
static float velFront = 0;
static float height = 0;
static float vnom_sign = 0;
static float vnom_flt = 0;
static float z_est = 0;
static float HH_h_sp_bias = 0.2; //m
static float SlwDwnFct = 1.0;

// LOCO Params
static uint8_t geofence_breached = 0;
static uint8_t geofence_counter = 0;
static uint8_t geofence_counter_th = 15;

static logVarId_t id_ExtState_x = 0;
static logVarId_t id_ExtState_y = 0;
static logVarId_t id_ExtState_z = 0;
static logVarId_t id_ExtState_vz = 0;
static logVarId_t id_ExtState_roll = 0;
static logVarId_t id_ExtState_pitch = 0;
static logVarId_t id_ExtState_yaw = 0;

static logVarId_t idFF = 0;

static uint32_t waypoint_delay = 5000;
static uint32_t waypoint_entry_time = 0;

static float loco_x_sp = 0.0;
static float loco_y_sp = 0.0;
static float loco_z_sp = 0.0;

// Column Inside NGB
// static uint8_t loco_rtn_home_vec_len = 6;
// static uint32_t safety_loco_timeout = 180000;
// static float loco_rtn_home_x[] = {18.5, 18.5,   18.5,   18.5,   18.5,   18.5};
// static float loco_rtn_home_y[] = {7.0,  7.0,    7.0,    7.0,    7.0,    7.0};
// static float loco_rtn_home_z[] = {5.0,  10.0,   15.0,   10.0,   5.0,    2.0};

/* Ground Level Test inside NGB
static float loco_rtn_home_x[] = {15.7,21.3,15.7,21.1,15.7};
static float loco_rtn_home_y[] = {8.8,8.9,5.1,5.4,8.8};
static float loco_rtn_home_z[] = {2.0,2.0,2.0,2.0,2.0};
*/
// "Chet" Outside to Inside NGB 
/*
static float loco_rtn_home_x[] = {3.5,18.5,18.5,18.5,18.5,18.5,18.5};
static float loco_rtn_home_y[] = {7.0,7.0,7.0,7.0,7.0,7.0,7.0};
static float loco_rtn_home_z[] = {30.0,30.0,20.0,15.0,10.0,5.0,2.0};
*/
// Column Inside NGB (for Z integ test)
// static float loco_rtn_home_x[] = {18.5,18.5,18.5,18.5,18.5,18.5};
// static float loco_rtn_home_y[] = {7.0,7.0,7.0,7.0,7.0,7.0};
// static float loco_rtn_home_z[] = {5.0,10.0,5.0,15.0,5.0,2.0};

// "Chet" Outside to Inside NGB (After fix)
// static uint8_t loco_rtn_home_vec_len = 6;
// static uint32_t safety_loco_timeout = 180000;
// static float loco_rtn_home_x[] = {3.5,3.5,18.5,18.5,18.5,18.5};
// static float loco_rtn_home_y[] = {7.0,7.0,7.0,7.0,7.0,7.0};
// static float loco_rtn_home_z[] = {5.0,30.0,30.0,7.0,7.0,2.0};

// "Chet" Outside to Inside NGB and back
// static uint8_t loco_rtn_home_vec_len = 14;
// static uint32_t safety_loco_timeout = 300000;
// static float loco_rtn_home_x[] = {3.5,  3.5,    18.5,   18.5,   18.5,   18.5,   25.0,   18.5,   18.5,   18.5,   3.5,    3.5,    3.5,    3.5};
// static float loco_rtn_home_y[] = {7.0,  7.0,    7.0,    7.0,    7.0,    7.0,    7.0,    7.0,    7.0,    7.0,    7.0,    7.0,    7.0,    7.0};
// static float loco_rtn_home_z[] = {5.0,  30.0,   30.0,   7.0,    7.0,    2.0,    2.0,    2.0,    5.0,    30.0,   30.0,   7.0,   7.0,   2.0};
// "Chet" Outside to Inside NGB
// static float loco_rtn_home_x[] = {3.5,18.5,18.5,18.5,18.5,18.5,18.5};
// static float loco_rtn_home_y[] = {7.0,7.0,7.0,7.0,7.0,7.0,7.0};
// static float loco_rtn_home_z[] = {30.0,30.0,20.0,15.0,10.0,5.0,2.0};
static uint8_t loco_rtn_home_vec_len = 5;
static uint32_t safety_loco_timeout = 300000;
static float loco_rtn_home_x[] = {0.0, 0.0, 3.0, 3.0, 3.0};
static float loco_rtn_home_y[] = {0.0, 0.0, 0.0, 0.0, 0.0};
static float loco_rtn_home_z[] = {1.5, 3.0, 3.0, 1.5, 0.0};


static float ExtState_x = 0.0;
static float ExtState_y = 0.0;
static float ExtState_z = 0.0;
static float loco_ctl_err = 10.0;
static float loco_ctl_radius = 0.3;
static float geofence_x_max = 40; //7 meters beyond station #7
static float geofence_y_max = 20; //7 meters beyond station #1
static float geofence_z_max = 50; //20 meters above highest waypoint
static float geofence_x_min = -10; //10 meters beyond origin
static float geofence_y_min = -10; //10 meters beyond origin
static float geofence_z_min = -5; //10 meters beyond origin

static setpoint_t setpoint;
static paramVarId_t id_motor_power_enb;
static paramVarId_t id_m1_power;
static paramVarId_t id_m2_power;
static paramVarId_t id_m3_power;
static paramVarId_t id_m4_power;
static paramVarId_t id_led_power;
static paramVarId_t id_Auto_Flight;
static paramVarId_t id_Reset_Est;
static paramVarId_t id_startPropTest;

static bool landHard = false;

static void setHoverSetpoint(setpoint_t *setpoint, float vx, float vy, float z, float yawrate) {
    setpoint->mode.x = modeVelocity;
    setpoint->mode.y = modeVelocity;
    setpoint->mode.z = modeAbs;
    setpoint->mode.yaw = modeVelocity;
    setpoint->velocity_body = true;

    setpoint->velocity.x = vx;
    setpoint->velocity.y = vy;
    setpoint->position.z = z;
    setpoint->attitudeRate.yaw = yawrate;
}

static void setPositionSetpoint(setpoint_t *setpoint, float x, float y, float z, float yawrate) {
    setpoint->mode.x = modeAbs;
    setpoint->mode.y = modeAbs;
    setpoint->mode.z = modeAbs;
    setpoint->mode.yaw = modeVelocity;
    setpoint->velocity_body = true; // Aron: not sure if this should be false

    setpoint->position.x = x;
    setpoint->position.y = y;
    setpoint->position.z = z;
    setpoint->attitudeRate.yaw = yawrate;
}

// Two Possible Nominal TOs:
// 1. Stab (8) --> Stab + Avoid (9) --> Hover + Avoid (1)
// 2. Stab (8) --> Hover (0) --> Hover + Avoid (1)
// Stab Only = No Avoidance + Open Vel. Loop
// Stab + Avoid = With Avoidance + Open Vel. Loop
// Hover = No Avoidance + Closed Vel. Loop
// Hover + Avoid = With Avoidance + Closed Vel. Loop

// SM States which FA Cannot Change (Internal Transitioning)
static void takeoff_N_stab(){
    height = height_sp;
    velFront = 0;
    velSide = 0;
    yawSpeed = 0;

    d_time = xTaskGetTickCount() - launch_time;
    bottom_tof = logGetUint(id_bottom_tof);

    if (bottom_tof > flr_clrnce) 
    {
        DEBUG_PRINT("SM: Floor Cleared\n");
        SM_State_var = SM_stab_N_avoid;
        DEBUG_PRINT("SM: Internal Transition to Stab&Avoid\n");
        state_entry=1;
        allw_OA = 1;
    }
    else if (d_time > stab_time) 
    {
        DEBUG_PRINT("SM: Takeoff Transient Passed\n"); //irrelevant when in Hand-Held takeoff (hard to time, dependent on user)
        SM_State_var = SM_takeoff_N_hover;
        DEBUG_PRINT("SM: Internal Transition to Takeoff&Hover\n");
        state_entry=1;
        allw_hover=1;
    }
}

static void stab_N_avoid()
{
    height = height_sp;
    velFront = 0;
    velSide = 0;
    yawSpeed = 0;

    d_time = xTaskGetTickCount() - launch_time;

    if (d_time > stab_time) 
    {
        DEBUG_PRINT("SM: Takeoff Transient Passed\n"); //irrelevant when in Hand-Held takeoff (hard to time, dependent on user)
        SM_State_var = SM_hover_N_avoid;
        DEBUG_PRINT("SM: Internal Transition to Hover&Avoid\n");
        state_entry=1;
        allw_hover=1;
    }
}

static void takeoff_N_hover()
{
    height = height_sp;
    velFront = 0;
    velSide = 0;
    yawSpeed = 0;

    bottom_tof = logGetUint(id_bottom_tof);

    if (bottom_tof > flr_clrnce) 
    {
        DEBUG_PRINT("SM: Floor Cleared\n");
        SM_State_var = SM_hover_N_avoid;
        DEBUG_PRINT("SM: Internal Transition to Hover&Avoid\n");
        state_entry=1;
        allw_OA = 1;
    }
}

static void catching(){
    height = height_sp;
    velFront = 0;
    velSide = 0;
    yawSpeed = 0;

    // if (d_time > hover_before_cutoff) // future imp. once terrian issue is solved for LD allowing gradual land-into hand
    launch_flag = 0;
    emergencyStop = 0;
    state = idle;                   
    SM_State_var = SM_takeoff_N_stab;
    SM_State_var_prev = SM_takeoff_N_stab;
    Phase1_SubState_var = starting;
    RandomExp_SubState_var = EXP_go;
    allw_hover = 0;
    allw_OA = 0;
    safety_fa = 0;
    state_entry=1;
    DEBUG_PRINT("Hand-Held Catch, Internal Transition to Idle!\n");  
}

// SM States which FA Can Change (External Transitioning)
static void prep_4_catch(){
    PositionLoop = 1; //Aron: remove upon merge
    // height = height_sp;
    // velFront = 0;
    // velSide = 0;
    // yawSpeed = 0;
    // allw_OA = 1; //Aron: consider stopping avoidance, keep for now when testing in confined space

    bottom_tof = logGetUint(id_bottom_tof);

    if (bottom_tof < catch_th)
    {
        DEBUG_PRINT("SM: Hand Detected by State Machine\n");
        // height_sp = ((float) bottom_tof)*0.001f; // future imp. once terrian issue is solved for LD

        paramSetInt(id_m1_power, (uint16_t) 0);
        paramSetInt(id_m2_power, (uint16_t) 0);
        paramSetInt(id_m3_power, (uint16_t) 0);
        paramSetInt(id_m4_power, (uint16_t) 0);
        paramSetInt(id_motor_power_enb, (uint8_t) 1); // switch to const. power
        DEBUG_PRINT("SM: Motor Cutoff by State Machine, Works For Looking Up\n");

        SM_State_var = SM_catching;
        state_entry=1;
        DEBUG_PRINT("SM: Internal Transition to Catching\n");
    }
    // else if (timeout)
    // {
    //     DEBUG_PRINT("SM: Prepare for Catch Timed-out\n");
    //     SM_State_var = SM_hover_N_avoid;
    //     state_entry=1;
    //     allw_OA = 1;
    //     DEBUG_PRINT("SM: Internal Transition to Hover&Avoid\n");   
    // }
}

static void hover_N_avoid(){
    height = height_sp;
    velFront = 0;
    velSide = 0;
    yawSpeed = 0;

    if ((allw_OA < 1) || (allw_hover < 1))
    {
        DEBUG_PRINT("SM: Attempted External Transition to Takeoff&Hover Before Takeoff Sequence was Completed!!\n");
        SM_State_var = SM_State_var_prev;
        state_entry = 1;
        DEBUG_PRINT("SM: Internal Transition back to Prevoius State in order to Complete Takeoff!!\n");
    }
}

static void guided(){
    height = height_sp; 
    
    SlwDwnFct = logGetFloat(id_SlwDwnFct);
    velFront = vx_sp*SlwDwnFct;
    velSide = vy_sp*SlwDwnFct;

    psi_cmd = PNG_k_psi*psi_sp;
    yawSpeed = CLAMP(psi_cmd, max_psi_cmd);
}

static void cruise(){
        //ARON place code here
}

static void slowing_down(){
        //ARON place code here
}

static void turning_around(){
        //ARON place code here
}


static void stateArmPrep(State *state, int8_t *safety_fa)
{
    safety_tof = logGetUint(id_safety_tof);
    stab_stop = logGetUint(id_stab_stop);
    stab_ready = logGetUint(id_stab_ready);

    if ((stab_stop == 1) & (*safety_fa == 1))
    {
        DEBUG_PRINT("Warning!!! Stabilizer Stopped, Release and Try Again, Otherwise Reboot Bolt!\n");
        *safety_fa = -1;
    }
    /* //ARON: REMOVE BEFORE MERGE or when integrating with TOF
    // else if (LD_Mode & (safety_tof < safety_tof_th) & (safety_fa == 1)) // ToDo: Consider similar logic for LU in future 
    {
        DEBUG_PRINT("Warning!!! Cap On, Remove Cap and Resend Safety_FA!\n");
        safety_fa = -7;
    }
    */
    else if ((stab_ready == 0) & (*safety_fa == 1))
    {
        DEBUG_PRINT("Warning!!! Stabilizer.StabReady==0, Complete PBIT and Try Again!\n");
        *safety_fa = -3;
    }
    else if (*safety_fa == 0)
    {
        *safety_fa = -8;
        BIT1();
        if (*safety_fa == -8)
        {
            *safety_fa = 0;
            DEBUG_PRINT("BIT1 Not Yet Confimed, Repeating BIT1, Update Safety_FA to 1 When Ready to ARM!\n");
        }
    }
    else if (*safety_fa == 1) // ((*safety_fa == 1) && (BIT1_PASS)), removed condition for now to allow auto-transition
    {
        DEBUG_PRINT("BIT1 External Confirmation!\n");
        DEBUG_PRINT("Safety Received From USER, ARMED!\n");
        *state = armed;
    }
}

static void stateArmed(State *state, int8_t *safety_fa, 
const uint8_t BIT2_PASS, uint8_t led_power, uint16_t idle_throttle)
{
    if (BIT2_PASS==0)
    {
        BIT2();
    }
    else if (BIT2_PASS==1)
    {
        paramSetInt(id_led_power, (uint8_t) led_power);
        paramSetInt(id_m1_power, (uint16_t) idle_throttle);
        paramSetInt(id_m2_power, (uint16_t) idle_throttle);
        paramSetInt(id_m3_power, (uint16_t) idle_throttle);
        paramSetInt(id_m4_power, (uint16_t) idle_throttle);
    }

    if (*safety_fa < 1)
    {
        DEBUG_PRINT("Warning!!! Tried to ARM without ARM_PREP!\n");
        *state = idle;
        DEBUG_PRINT("Returning to IDLE and DISARMING\n");
    }
    else if (timeout)
    {
        DEBUG_PRINT("ARMED Timed-out\n");
        *state = idle;
        *safety_fa = -6;
        DEBUG_PRINT("Returning to IDLE and DISARMING\n");
    }
}


static void stateHandHeldArmPrep(State *state, int8_t *safety_fa)
{
    safety_tof = logGetUint(id_safety_tof);
    stab_stop = logGetUint(id_stab_stop);
    stab_ready = logGetUint(id_stab_ready);

    if ((stab_stop == 1) & (*safety_fa == 1))
    {
        DEBUG_PRINT("Warning!!! Stabilizer Stopped, Release and Try Again, Otherwise Reboot Bolt!\n");
        *safety_fa = -1;
    }
    else if ((safety_tof > safety_tof_th) & (*safety_fa == 1))
    {
        DEBUG_PRINT("Warning!!! Cap Missing, Return Cap and Resend Safety_FA!\n");
        *safety_fa = -2;
    }
    else if ((stab_ready == 0) & (*safety_fa == 1))
    {
        DEBUG_PRINT("Warning!!! Stabilizer.StabReady==0, Complete PBIT and Try Again!\n");
        *safety_fa = -3;
    }
    else if (safety_fa == 0)
    {
        *safety_fa = -8;
        BIT1();
        if (*safety_fa == -8)
        {
            *safety_fa = 0;
            DEBUG_PRINT("BIT1 Not Yet Confimed, Repeating BIT1, Update Safety_FA to 1 When Ready to ARM!\n");
        }
    }
    else if (*safety_fa == 1) // ((safety_fa == 1) && (BIT1_PASS)), removed condition for now to allow auto-transition
    {
        DEBUG_PRINT("BIT1 External Confirmation!\n");
        for(int8_t jj = 0; jj < 2; jj++)
        {
            strobe(strobe_power_med, rise_time_fast, wait_time_fast);
        }  
        DEBUG_PRINT("Safety Received From USER, ARMED!\n");
        *state = hand_held_armed;
    }
}

static void stateHandHeldArmed(State *state, int8_t *safety_fa, uint8_t *cap_counter, 
uint32_t *trigger_time, uint8_t *throttleCoef, uint8_t cap_counter_th)
{
    memset(&setpoint, 0, sizeof(setpoint_t));
    commanderSetSetpoint(&setpoint, Bolt_Prio);
    paramSetInt(id_led_power, (uint8_t) led_power);

    safety_tof = logGetUint(id_safety_tof);

    // accumulate consecutive cap_off events
    if (safety_tof > safety_tof_th)
    {
        *cap_counter++;
    }
    else
    {
        *cap_counter=0;
    }

    // 
    if (*safety_fa < 1)
    {
        DEBUG_PRINT("Warning!!! Tried to ARM without ARM_PREP!\n");
        *state = idle;
        DEBUG_PRINT("Returning to IDLE and DISARMING\n");
    }
    else if (timeout)
    {
        DEBUG_PRINT("ARMED Timed-out\n");
        *state = idle;
        *safety_fa = -6;
        *cap_counter = 0;
        DEBUG_PRINT("Returning to IDLE and DISARMING\n");
    }
    else if (*cap_counter > cap_counter_th)
    {
        DEBUG_PRINT("Cap Removed, waiting for clearance!\n");
        vTaskDelay(M2T(700));
        *trigger_time = xTaskGetTickCount();
        *throttleCoef=0;
        *cap_counter=0;
        *state = hand_held_cap_off;
        DEBUG_PRINT("Cap Cleared!\n");
        paramSetInt(id_Reset_Est, (uint8_t) 1);
        DEBUG_PRINT("Cap Triggered Reset Estimation!\n");
    }
}

static void stateHandHeldCapOff(State *state, int8_t *safety_fa, int8_t *launch_flag, int8_t *throttleCoef)
{
    memset(&setpoint, 0, sizeof(setpoint_t));
    commanderSetSetpoint(&setpoint, Bolt_Prio);
    
    safety_tof = logGetUint(id_safety_tof);
    stab_stop = logGetUint(id_stab_stop);
    d_time = xTaskGetTickCount() - trigger_time;

    z_est = logGetFloat(id_z_est);
    height_sp = z_est + HH_h_sp_bias;

    if (safety_tof < safety_tof_th)
    {
        DEBUG_PRINT("Cap Abort!!! Return Cap and Try Again!\n");
        *safety_fa = -4;
        *state = idle;
    }
    else if (stab_stop == 1)
    {
        DEBUG_PRINT("Stabilizer Stop Abort!!! Release and Try Again, Otherwise Reboot Bolt!\n");
        *safety_fa = -5;
        *state = idle;
    }
    else if (d_time > trigger_dt)
    {
        DEBUG_PRINT("Hand-Held Takeoff Triggered!\n");
        *launch_flag = 1;
    }
    else if (d_time > (uint32_t) (1000 + 2*(trigger_dt/3)))
    {
        DEBUG_PRINT("Hand-Held Idle Throttle On!\n");
        paramSetInt(id_m1_power, hd_idle_throttle);
        paramSetInt(id_m2_power, hd_idle_throttle);
        paramSetInt(id_m3_power, hd_idle_throttle);
        paramSetInt(id_m4_power, hd_idle_throttle);
    }
    else if (d_time > (uint32_t) (1000 + *throttleCoef*(trigger_dt/3)))
    {   
        for (uint8_t jj = 0; jj < (*throttleCoef + 1); jj++)
        {
            DEBUG_PRINT("Hand-Held Pre-Idle Throttle On!\n");
            paramSetInt(id_m1_power, (uint16_t) (10000 + *throttleCoef*2500));
            paramSetInt(id_m2_power, (uint16_t) (10000 + *throttleCoef*2500));
            paramSetInt(id_m3_power, (uint16_t) (10000 + *throttleCoef*2500));
            paramSetInt(id_m4_power, (uint16_t) (10000 + *throttleCoef*2500));

            vTaskDelay(M2T(300)); //msec

            DEBUG_PRINT("Hand-Held Pre-Idle Throttle Off!\n");
            paramSetInt(id_m1_power, (uint16_t) 0);
            paramSetInt(id_m2_power, (uint16_t) 0);
            paramSetInt(id_m3_power, (uint16_t) 0);
            paramSetInt(id_m4_power, (uint16_t) 0);

            vTaskDelay(M2T(300)); //msec
        }
        *throttleCoef++;
    }
}

static void stateStopping(State *state, int8_t *safety_fa, uint8_t *emergencyStop,
uint8_t *launch_flag, uint8_t* allw_hover, uint8_t* allw_OA, SM_State* SM_State_var,  SM_State* SM_State_var_prev,
Phase1_SubState* Phase1_SubState_var, Phase1_SubState* RandomExp_SubState_var)
{
    if (*emergencyStop == 2) 
    {
        DEBUG_PRINT("EMERGENCY STOP RQ!\n");
        estop_time = xTaskGetTickCount();
        d_time = 0;
        while (d_time < 5000)  
        {
            d_time = xTaskGetTickCount() - estop_time;
            setHoverSetpoint(&setpoint, 0, 0, -2, 0);
            commanderSetSetpoint(&setpoint, Bolt_Prio);
            vTaskDelay(M2T(60));
        }
        *launch_flag = 0;
        *emergencyStop = 0;
        *state = idle;
        DEBUG_PRINT("EMERGENCY STOP Release, returned to IDLE!\n");
        *SM_State_var = SM_takeoff_N_stab;
        *SM_State_var_prev = SM_takeoff_N_stab;
        *Phase1_SubState_var = starting;
        *RandomExp_SubState_var = EXP_go;
        *allw_hover = 0;
        *allw_OA = 0;
        *safety_fa = 0;
    }

    velFront = 0.0;
    velSide = 0.0;
    yawSpeed = 0.0;

    while (height > landing_height) 
    {
        height -= 0.015f;
        vTaskDelay(M2T(60));
        setHoverSetpoint(&setpoint, velFront, velSide, height, yawSpeed);
        commanderSetSetpoint(&setpoint, Bolt_Prio);
    }
    setHoverSetpoint(&setpoint, 0, 0, -2, 0);
    commanderSetSetpoint(&setpoint, Bolt_Prio);
}


static void stateUnlocked(State *state, uint8_t *state_entry, uint8_t *TstSlwDwn, 
 uint8_t *ExtKalman, uint32_t* scan_time, uint32_t* trigger_time,
 Phase1_SubState* Phase1_SubState_var,  SM_State SM_State_var )
{
    if (emergencyStop == 0) 
    {
        //begin new state machine
        if (SM_State_var != SM_State_var_prev)
        {
            *state_entry=1;
            DEBUG_PRINT("SM: External Transition\n");
        }

        switch(SM_State_var)
        {
        case SM_takeoff_N_hover: // Transition from takeoff_N_stab
            if (*state_entry == 1)
            {
                DEBUG_PRINT("SM: Entered takeoff_N_hover\n");
                *state_entry = 0;
                *TstSlwDwn = 0;
            }
            takeoff_N_hover();
            break;

        case SM_hover_N_avoid:
            if (*state_entry == 1)
            {
                DEBUG_PRINT("SM: Entered hover_N_avoid\n");
                *state_entry = 0;
                *TstSlwDwn = 0;
            }
            hover_N_avoid();
            break;

        case SM_cruise: // NOT YET IMPLEMENTED
            if (*state_entry == 1)
            {
                DEBUG_PRINT("SM: Entered cruise\n");
                *state_entry = 0;
                *TstSlwDwn = 0;
            }
            cruise();
            break;

        case SM_slowing_down: // NOT YET IMPLEMENTED
            if (*state_entry == 1)
            {
                DEBUG_PRINT("SM: Entered slowing_down\n");
                *state_entry = 0;
                *TstSlwDwn = 0;
            }
            slowing_down();
            break;

        case SM_turning_around: // NOT YET IMPLEMENTED
            if (*state_entry == 1)
            {
                DEBUG_PRINT("SM: Entered turning_around\n");
                *state_entry = 0;
                *TstSlwDwn = 0;
            }
            turning_around();
            break;

        case SM_phase_1:
            if (*state_entry == 1)
            {
                DEBUG_PRINT("SM: Entered autonomous_phase_1\n");
                *state_entry = 0;
                *scan_time = xTaskGetTickCount();
                *Phase1_SubState_var = starting;
                *TstSlwDwn = 0;
            }
            autonomous_phase_1();
            break;

        case SM_random_exp: // NOT YET IMPLEMENTED
            if (*state_entry == 1)
            {
                DEBUG_PRINT("SM: Entered random_exp\n");
                *state_entry = 0;
                *TstSlwDwn = 0;
            }
            // random_exp();
            break;

        case SM_guided:
            if (*state_entry == 1)
            {
                DEBUG_PRINT("SM: Entered guided\n");
                *state_entry = 0;
                *TstSlwDwn = 1;
            }
            guided();
            break;

        case SM_takeoff_N_stab:
            if (*state_entry == 1)
            {
                DEBUG_PRINT("SM: Entered Takeoff&Stab\n");
                *state_entry = 0;
                *TstSlwDwn = 0;
            }
            takeoff_N_stab();
            break;

        case SM_stab_N_avoid:
            if (*state_entry == 1)
            {
                DEBUG_PRINT("SM: Entered Stab&Avoid\n");
                *state_entry = 0;
                *TstSlwDwn = 0;
            }
            stab_N_avoid();
            break;

        case SM_prep_4_catch:
            if (*state_entry == 1)
            {
                DEBUG_PRINT("SM: Entered Prepare For Catch\n");
                *state_entry = 0;
                *TstSlwDwn = 0;
            }
            prep_4_catch();
            break;

        case SM_catching:
            if (*state_entry == 1)
            {
                DEBUG_PRINT("SM: Entered Catching\n");
                *state_entry = 0;
                *TstSlwDwn = 0;
            }
            catching();
            break;

        case SM_loco_rtn_home:
            if (*state_entry == 1)
            {
                DEBUG_PRINT("SM: Entered LOCO Return Home\n");
                *trigger_time = xTaskGetTickCount(); // timer for motor cutoff (safety requirement)
                *state_entry = 0;
                *TstSlwDwn = 0;
            }
            loco_rtn_home();
            break;

        case SM_loco_goto_wp:
            if (*state_entry == 1)
            {
                DEBUG_PRINT("SM: LOCO Go To Waypoint\n");
                *state_entry = 0;
                *TstSlwDwn = 0;
            }
            loco_goto_wp();
            break;

        default:
            DEBUG_PRINT("SM: no such state\n");
            break;
        }
        //end new state machine

        // Loop Closure
        if (PositionLoop == 1)
        {
            *ExtKalman = 1;
            // ToDo: Command position Loop
            setPositionSetpoint(&setpoint, loco_x_sp, loco_y_sp, loco_z_sp, yawSpeed);
            commanderSetSetpoint(&setpoint, Bolt_Prio);
        }
        else
        {
            *ExtKalman = 1;
            setHoverSetpoint(&setpoint, velFront, velSide, height, yawSpeed);
            commanderSetSetpoint(&setpoint, Bolt_Prio);
        }

        SM_State_var_prev=SM_State_var;
    }
    else 
    {
        *state = stopping;
        DEBUG_PRINT("STOPPED\n");
    }
}

static void stateLaunch(State *state, uint8_t* launch_flag, uint32_t* launch_time)
{
    stab_stop = logGetUint(id_stab_stop);
    stab_ready = logGetUint(id_stab_ready);

    if (stab_stop == 1)
    {
        DEBUG_PRINT("ERROR: Cannot Launch!\n");
        DEBUG_PRINT("Stabilizer.Stop==1, Release and Try Again!\n");
        *launch_flag = 0;
    }
    else if (stab_ready == 0)
    {
        DEBUG_PRINT("ERROR: Cannot Launch!\n");
        DEBUG_PRINT("Stabilizer.StabReady==0, Complete PBIT and Try Again!\n");
        *launch_flag = 0;                
    }
    else
    {
        DEBUG_PRINT("Unlocked!\n");
        DEBUG_PRINT("SM: takeoff started\n");
        *launch_time = xTaskGetTickCount();
        paramSetInt(id_motor_power_enb, (uint8_t) 0); // switch to controller power
        *state = unlocked;               
    }
}

static void stateIdle(State *state)
{
    // memset(&setpoint, 0, sizeof(setpoint_t));
    // commanderSetSetpoint(&setpoint, Bolt_Prio);

    uint8_t ffIndicator = logGetUint(idFF);
    if (ffIndicator > 0)// || forceStart)
    {
        DEBUG_PRINT("**********  SFF: Free fall detected, **************\n");
        *state = falling;
    }
}

static void stateFalling(State *state)
{
    uint32_t fallTimems = T2M(xTaskGetTickCount());
 
    logVarId_t idroll = logGetVarId("stateEstimate", "roll");
    logVarId_t idpitch = logGetVarId("stateEstimate", "pitch");
 
    float xCur, yCur, zCur, vzCur, height, rollCur, pitchCur, yawCur, zTar = logGetFloat(id_ExtState_z);
    uint16_t upCur;
    zCur = logGetFloat(logGetVarId("stateEstimate", "z"));
    uint32_t curTimems = T2M(xTaskGetTickCount());
    DEBUG_PRINT("** Hover OK, z=%f, TOF = %d [ms] **\n", zCur, (curTimems - fallTimems));
    uint8_t hoverFlag = 0;
    static setpoint_t setpoint;
    memset(&setpoint, 0, sizeof(setpoint_t));
    while (!hoverFlag)
    {
        rollCur = logGetFloat(idroll);
        pitchCur = logGetFloat(idpitch);
        if (fabs(rollCur) > 10 || fabs(pitchCur) > 10)
        {
          setpoint.mode.z = modeVelocity;
          setpoint.velocity.z = 0;
          // setHoverSetpoint(&setpoint, 0, 0, zTar, 0);
          commanderSetSetpoint(&setpoint, 3);
          DEBUG_PRINT("**********  Pitch or Roll are still big **************\n");
          vTaskDelay(M2T(10));
          break;
       }
        else
        {
          hoverFlag = 1;
          DEBUG_PRINT("**********  Continue to hover **************\n");
        }
     }

     zCur = logGetFloat(logGetVarId("stateEstimate", "z"));
     curTimems = T2M(xTaskGetTickCount());
     DEBUG_PRINT("*1* Position flat, start hovering, z=%f, TOF = %d [ms] **\n", zCur, (curTimems - fallTimems));

     memset(&setpoint, 0, sizeof(setpoint_t));
     fallTimems = T2M(xTaskGetTickCount());
     uint8_t fallStabilized = 0, first = 1;
     while (!fallStabilized)
     {
        zCur = logGetFloat(logGetVarId("stateEstimate", "z"));
        curTimems = T2M(xTaskGetTickCount());
        if ((curTimems - fallTimems) < hover_time)
        {
          zTar = zCur;
          setHoverSetpoint(&setpoint, 0, 0, zTar, 0);
          commanderSetSetpoint(&setpoint, 3);
          if (first)
          {
                DEBUG_PRINT("*2* Trying to hover **\n");
                DEBUG_PRINT("*3* z=%f, TOF = %d [ms] **\n", zCur, (curTimems - fallTimems));
                first = 0;
          }
          vTaskDelay(M2T(10));
        }
        else
        {
          DEBUG_PRINT("*4*********  Stabilization time is out **************\n");
          if (fabs(zCur - zTar) < 0.1)
          {
                fallStabilized = 1;
                DEBUG_PRINT("*5* Hover OK, z=%f, ztar =%f TOF = %d [ms] **\n", zCur, zTar, (curTimems - fallTimems));
                break;
          }
          else
          {
                DEBUG_PRINT("*6* Hover BAD, Stopping z=%f-->%f, TOF = %d [ms]**\n", zCur, zTar, (curTimems - fallTimems));
                *state = stopping;
                memset(&setpoint, 0, sizeof(setpoint_t));
                commanderSetSetpoint(&setpoint, 3);
                vTaskDelay(M2T(1000));
                *state = idle;  
                return ;      
          }
        }
    }

    memset(&setpoint, 0, sizeof(setpoint_t));
    zCur = logGetFloat(logGetVarId("stateEstimate", "z"));
    while (zCur > 0.1f)
    {
        height = zCur * 4 / 10;

        DEBUG_PRINT("** Landing, z=%f-->%f**\n", zCur, height);
        setHoverSetpoint(&setpoint, 0, 0, height, 0);
        commanderSetSetpoint(&setpoint, 3);
        vTaskDelay(M2T(100));
        zCur = logGetFloat(logGetVarId("stateEstimate", "z"));
    }
////////////////////////////////////////////////////////////////////////
    memset(&setpoint, 0, sizeof(setpoint_t));
    commanderSetSetpoint(&setpoint, 3);
    vTaskDelay(M2T(1000));
    *state = idle;
}

static void autonomous_phase_1()
{
    height = height_sp;
    velFront = 0;
    velSide = 0;
    yawSpeed = 0;

    psi_cmd = logGetFloat(id_psi_cmd);
    vx_cmd = logGetFloat(id_vx_cmd);
    vy_cmd = logGetFloat(id_vy_cmd);
    vx_nom = logGetFloat(id_vx_nom);
    vy_nom = logGetFloat(id_vy_nom);
    uint8_t Stopped_Current = logGetUint(id_Stopped);
    uint8_t OF_OL_event = logGetUint(id_OF_OL);
    uint8_t OF_OL_recovery = logGetUint(id_OF_IL);
    
    // oper_nominal: starting --> go_straight --> turn_around --> return_home
    // dev_nominal: starting --> go_straight --> turn_around --> go_straight ...

    if (Phase1_SubState_var == starting) {
        vnom_sign = 0.0F;//freeze
        vx_cmd = 0.0F;   //override centering during initial scan
        vy_cmd = 0.0F;   //override centering during initial scan
        psi_cmd = 0.0F;  //override parallelization during initial scan
        d_time = xTaskGetTickCount() - scan_time;

        // Check transition conditions
        if ( d_time > 2000) {
            Phase1_SubState_var = go_straight;
            DEBUG_PRINT("SM: Completed Scan, Begin FWD\n");
            scan_time = xTaskGetTickCount();
        }
        psi_cmd = SIN_AMP*sinf(6.2832F*(float)(d_time)*0.001F*SIN_FREQ);  // 15 deg/sec amplitude in 1Hz for 2 seconds
    }

    if (Phase1_SubState_var == go_straight) {
        vnom_sign = 1;

        // turn around triggers
        if (Stopped_Prev < Stopped_Current){
            Stopped_Rise_Flag = 1;
            DEBUG_PRINT("SM: reached dead-end\n");
        }
        if (xTaskGetTickCount() - scan_time > lag_timer){
            leg_completed = 1;
            DEBUG_PRINT("SM: leg completed\n");
        }

        // retreat triggers

        if (allw_OF_ret & OF_OL_event){
            OF_issue=1;
            OF_issue_recovery = 0;
            DEBUG_PRINT("SM: OF issue occurred\n");
        }
        if (allw_W_ret & 0){ // Aron: replace 0 with actual event
            sudden_w_change=1;
            sudden_w_change_recovery = 0;
            DEBUG_PRINT("SM: sudden change of width occurred\n");
        }
        if (allw_Z_ret & 0){ // Aron: replace 0 with actual event
            sudden_z_change=1;
            sudden_z_change_recovery = 0;
            DEBUG_PRINT("SM: sudden change of height occurred\n");
        }

        // Check transition conditions
        if (sudden_w_change || sudden_z_change|| OF_issue || go_bkwrd_fl){
            scan_time = xTaskGetTickCount();
            DEBUG_PRINT("SM: go backwards\n");
            Phase1_SubState_var = go_backwards;
        }
        else if (leg_completed || Stopped_Rise_Flag || trn_arnd_fl) {
            scan_time = xTaskGetTickCount();
            DEBUG_PRINT("SM: turn_around\n");
            Phase1_SubState_var = turn_around;
        }
    }

    if (Phase1_SubState_var == turn_around) {
        vnom_sign = 0.0F;
        vx_cmd = 0.0F; // consider allowing centering during turn around
        vy_cmd = 0.0F; // consider allowing centering during turn around
        psi_cmd = 45.0F;
        d_time = xTaskGetTickCount() - scan_time;
        // reset turn around triggers
        Stopped_Rise_Flag = 0;
        leg_completed = 0;
        trn_arnd_fl = 0;

        // Check transition conditions
        if (d_time > 4000) {
            scan_time = xTaskGetTickCount();
            if (OPER_flag==1){
                //OPER state
                DEBUG_PRINT("SM: Return Home\n");
                Phase1_SubState_var = return_home;
            }
            else {
                //DEV state
                DEBUG_PRINT("SM: Go Straight\n");
                Phase1_SubState_var = go_straight;
            }
        }
    }

    //RETURN HOME
    if (Phase1_SubState_var == return_home) {
        vnom_sign = 1;
    }

    //GO BACKWARDS
    if (Phase1_SubState_var == go_backwards) 
    {
        vnom_sign = -1.0F;
        d_time = xTaskGetTickCount() - scan_time;
        // reset go backward flags
        go_bkwrd_fl = 0;
        sudden_w_change = 0;
        sudden_z_change = 0;
        OF_issue = 0;

        // recover from retreat triggers
        if (allw_OF_ret & (OF_OL_recovery)){
            OF_issue_recovery=1;
            DEBUG_PRINT("SM: OF recovery occurred\n");
        }
        if (allw_W_ret & 0){ // implement recovery
            sudden_w_change_recovery=1;
            DEBUG_PRINT("SM: W recovery occurred\n");
        }
        if (allw_Z_ret & 0){ // implement recovery
            sudden_z_change_recovery=1;
            DEBUG_PRINT("SM: Z recovery occurred\n");
        }

        // Check transition conditions
        if ((d_time > 2000) & (OF_issue_recovery & sudden_w_change_recovery & sudden_z_change_recovery)) { // Aron: replace 1's with actual recovery triggers
            scan_time = xTaskGetTickCount();
            DEBUG_PRINT("SM: prep_turn\n");
            Phase1_SubState_var = prep_turn;
        }
    }

        // PREPARE FOR TURN
    if (Phase1_SubState_var == prep_turn) {
        vnom_sign = -1.0F;
        d_time = xTaskGetTickCount() - scan_time;
        // Check transition conditions
        if (d_time > prep_turn_dt) {
            scan_time = xTaskGetTickCount();
            DEBUG_PRINT("SM: turn_around\n");
            Phase1_SubState_var = turn_around;
        }
    }

    vnom_flt = (1.0F-VNOM_FLT_ALPHA)*vnom_flt+VNOM_FLT_ALPHA*vnom_sign;
    velFront = vnom_flt * vx_nom + vx_cmd;
    velSide = vnom_flt * vy_nom + vy_cmd;
    yawSpeed = psi_cmd;
    Stopped_Prev = Stopped_Current;
}  

static void strobe(const int16_t strobe_power, const uint16_t rise_time, const uint16_t wait_time)
{
    uint16_t dt = rise_time/(255-strobe_power);

    for(int16_t jj = 255; jj > strobe_power; jj--)
    {
        paramSetInt(id_led_power, (uint8_t) jj);
        vTaskDelay(M2T(dt));
    }
    vTaskDelay(M2T(wait_time));
    for(int16_t jj = strobe_power; jj < 256; jj++)
    {
        paramSetInt(id_led_power, (uint8_t) jj);
        vTaskDelay(M2T(dt));
    }
    vTaskDelay(M2T(wait_time));
}

static void BIT1()
{
    DEBUG_PRINT("BIT1 Started!\n");
    strobe(strobe_power_high, rise_time_slow, wait_time_slow);

    // TODO: Add BIT Tasks Here
    // MR18
    DEBUG_PRINT("BIT1 Completed!\n");
    
    // TODO: Add BIT Logics Here
    BIT1_PASS = 1;
    DEBUG_PRINT("BIT1 PASSED!\n");
}  

static void BIT2()
{
    DEBUG_PRINT("BIT2 Started!\n");   
    paramSetInt(id_startPropTest, (uint8_t) 1);
    // TODO: Add BIT Tasks Here
    // MR18
    // H_EST
    // V_EST

    scan_time = xTaskGetTickCount();
    d_time = 0;

    while (d_time < 12000)
    {
        strobe(strobe_power_med, rise_time_fast, wait_time_fast);
        d_time = xTaskGetTickCount() - scan_time;
    }
    DEBUG_PRINT("BIT2 Completed!\n");   
    
    // TODO: Add BIT Logics Here
    motorPass = logGetUint(id_motorPass);
    motorTestCount = logGetUint(id_motorTestCount);
    BIT2_PASS = 1;
    DEBUG_PRINT("BIT2 PASSED!\n");
    DEBUG_PRINT("BIT2 Internal Confirmation!\n");
}

static void test_kalman1_converged()
{
    // Aron ToDO: Implement
    kalman1_converged = 1;
}

static void test_kalman2_converged()
{
    // Aron ToDO: Implement
    kalman2_converged = 1;
}

static void advance_loco_wp()
{
    loco_ctl_err = sqrtf(powf((ExtState_x-loco_x_sp),2)+powf((ExtState_y-loco_y_sp),2)+powf((ExtState_z-loco_z_sp),2));
    if ((loco_ctl_err < loco_ctl_radius) || (1 == waypoint_timer_on)) 
    {
        if (waypoint_timer_on == 0)
        {
            waypoint_entry_time = xTaskGetTickCount();
            waypoint_timer_on = 1;
            DEBUG_PRINT("Entered LOCO waypoint radius\n");
        }

        d_time = xTaskGetTickCount() - waypoint_entry_time;

        if (d_time > waypoint_delay)
        {
            loco_ii++;
            waypoint_timer_on = 0;
            DEBUG_PRINT("Continuing to next LOCO waypoint\n");
        }
    }
}

static void check_geo_fence()
{
    if ((ExtState_x > geofence_x_max) || (ExtState_y > geofence_y_max) || (ExtState_z > geofence_z_max) ||
        (ExtState_x < geofence_x_min) || (ExtState_y < geofence_y_min) || (ExtState_z < geofence_z_min)) 
    {
        geofence_counter++;
    }
    else
    {
        geofence_counter=0;
    }

    // Aron: in the future implement an internal geofence which also attempts to land or return home, should transition to proper state

    if (geofence_counter > geofence_counter_th)
    {
        geofence_breached = 1;

        paramSetInt(id_m1_power, (uint16_t) 0);
        paramSetInt(id_m2_power, (uint16_t) 0);
        paramSetInt(id_m3_power, (uint16_t) 0);
        paramSetInt(id_m4_power, (uint16_t) 0);
        paramSetInt(id_motor_power_enb, (uint8_t) 1); // switch to const. power
        DEBUG_PRINT("SM: Motor Cutoff by State Machine, Reached LOCO Geofence Limit\n");

        launch_flag = 0;
        emergencyStop = 0;
        state = idle;                   
        SM_State_var = SM_takeoff_N_stab;
        SM_State_var_prev = SM_takeoff_N_stab;
        Phase1_SubState_var = starting;
        RandomExp_SubState_var = EXP_go;
        allw_hover = 0;
        allw_OA = 0;
        safety_fa = 0;
        state_entry=1;
    }
}

static void loco_rtn_home()
{
    PositionLoop = 1;
    paramSetInt(id_Auto_Flight, (uint8_t) 0);

    ExtState_x = logGetInt(id_ExtState_x)*0.001;
    ExtState_y = logGetInt(id_ExtState_y)*0.001;
    ExtState_z = logGetInt(id_ExtState_z)*0.001;

    // check_geo_fence(); No geo fence when no loco
    advance_loco_wp();

    if (loco_ii >= loco_rtn_home_vec_len)
    {
        loco_ii = loco_rtn_home_vec_len-1;
        reached_final_wp = 1;
        DEBUG_PRINT("Reached final LOCO waypoint\n");
    }

    loco_x_sp = loco_rtn_home_x[loco_ii];
    loco_y_sp = loco_rtn_home_y[loco_ii];
    loco_z_sp = loco_rtn_home_z[loco_ii];
    yawSpeed = 0.0;

    d_time = xTaskGetTickCount() - trigger_time;
    d_time = xTaskGetTickCount() - launch_time; //ARON: REMOVE BEFORE MERGE or when integrating with TOF
    if (d_time > safety_loco_timeout)
    {
        paramSetInt(id_m1_power, (uint16_t) 0);
        paramSetInt(id_m2_power, (uint16_t) 0);
        paramSetInt(id_m3_power, (uint16_t) 0);
        paramSetInt(id_m4_power, (uint16_t) 0);
        paramSetInt(id_motor_power_enb, (uint8_t) 1); // switch to const. power
        DEBUG_PRINT("SM: Motor Cutoff by State Machine, Reached LOCO Safety Timer Limit\n");

        launch_flag = 0;
        emergencyStop = 0;
        state = idle;                   
        SM_State_var = SM_takeoff_N_stab;
        SM_State_var_prev = SM_takeoff_N_stab;
        Phase1_SubState_var = starting;
        RandomExp_SubState_var = EXP_go;
        allw_hover = 0;
        allw_OA = 0;
        safety_fa = 0;
        state_entry=1;
    }

    test_kalman1_converged();
    if (kalman1_converged && reached_final_wp)
    {
        DEBUG_PRINT("SM: Completed LOCO Sequence\n");
        SM_State_var = SM_hover_N_avoid;
        DEBUG_PRINT("SM: Internal Transition to Hover&Avoid\n");
        state_entry=1;
        paramSetInt(id_Auto_Flight, (uint8_t) 1);
        reached_final_wp = 0;
        loco_ii = 0;
    }
}

static void loco_goto_wp(){
        // duplicate or merge with loco_rtn_home
}


static void autonomous_pilot() 
{
    vTaskDelay(M2T(500));

    DEBUG_PRINT("***** AUTONOMOUS PILOT Waiting for activation ...\n");   

    while (1) 
    {
        vTaskDelay(M2T(10));

         uint8_t ffIndicator = logGetUint(idFF);

        if (MAN_AUTO_PROPTEST == 0) // 0 = Manual / 1 = Auto / 2 = Prop. Test
        {
            state = manual;
            Bolt_Prio = 2;
            paramSetInt(id_Auto_Flight, (uint8_t) 0);
            paramSetInt(id_led_power, (uint8_t) led_power);
        }
        else if (MAN_AUTO_PROPTEST == 2)
        {
            state = prop_test;
        }

        // Two Possible Nominal Pre-TOs:
        //1.  idle -(user)-> arm_prep             -(bolt)-> armed                                       -(user)-> unlocked -(user)-> stopping --> idle
        //2.  idle -(user)-> hand_held_arm_prep   -(bolt)-> hand_held_armed -(user)-> hand_held_cap_off -(bolt)-> unlocked              --> idle

        if (state == unlocked) 
        {
            stateUnlocked(&state, &state_entry, &TstSlwDwn, 
                            &ExtKalman, &scan_time, &trigger_time,
                            &Phase1_SubState_var, SM_State_var);
        } 
        else // state not unlocked
        {
////////////////////////////////////////////////////////////////////////
            if ((launch_flag == 1) && ((state == armed) || (state == hand_held_cap_off)))
            {
                stateLaunch(&state, &launch_flag, &launch_time);
            }
////////////////////////////////////////////////////////////////////////
            if (state == idle) 
            {
                stateIdle(&state);
            }
           if (state == falling) 
            {
                stateFalling(&state);
            }
////////////////////////////////////////////////////////////////////////
            if (state == arm_prep) 
            {
                stateArmPrep(&state, &safety_fa);               
            }
  ////////////////////////////////////////////////////////////////////////              
            if (state == hand_held_arm_prep) 
            {
                stateHandHeldArmPrep(&state, &safety_fa);
            }
////////////////////////////////////////////////////////////////////////
            if (state == hand_held_armed)
            {
                stateHandHeldArmed(&state, &safety_fa, &cap_counter,&trigger_time, &throttleCoef, cap_counter_th);
            }
////////////////////////////////////////////////////////////////////////
            if (state == armed)
            {
                stateArmed(&state, &safety_fa, BIT2_PASS, led_power, idle_throttle);
            }
////////////////////////////////////////////////////////////////////////
            if (state == hand_held_cap_off) 
            {
                stateHandHeldCapOff(&state, &safety_fa, &launch_flag, &throttleCoef);
            }
//////////////////////////////////////////////////////////////////////
            if (state == stopping) 
            {
                stateStopping(&state, &safety_fa, &emergencyStop,
                    &launch_flag, &allw_hover, &allw_OA, &SM_State_var, &SM_State_var_prev,
                    &Phase1_SubState_var, &RandomExp_SubState_var);
            }
////////////////////////////////////////////////////////////////////////        
            if (state == manual)
            {
                memset(&setpoint, 0, sizeof(setpoint_t));
                commanderSetSetpoint(&setpoint, Bolt_Prio);
                paramSetInt(id_motor_power_enb, (uint8_t) 0); // switch to controller power
            }
////////////////////////////////////////////////////////////////////////            
            if (state == prop_test)
            {
                memset(&setpoint, 0, sizeof(setpoint_t));
                commanderSetSetpoint(&setpoint, Bolt_Prio);
                paramSetInt(id_motor_power_enb, (uint8_t) 1); // switch to const. power
            }
        }
    }
    DEBUG_PRINT("Warning!!! FINISHED! Should never get here\n");
}

void appMain() 
{
    DEBUG_PRINT("***** state machine function ***** \n");
    if (mr_init_val == 1) 
    {
        DEBUG_PRINT("** Using MR5 \n");
    } 
    else if (mr_init_val == 2) 
    {
        DEBUG_PRINT("** Using MR18\n");
    
    	STATIC_MEM_TASK_ALLOC(mr18_driver, MR18_APP_STACKSIZE);
        STATIC_MEM_TASK_CREATE(mr18_driver, mr18_driver, "mr18_drv", NULL, MR18_APP_PRIORITY);

        id_psi_cmd = logGetVarId("feat2cmd", "psi_cmd");
        id_vx_cmd = logGetVarId("feat2cmd", "Vxcmd");
        id_vy_cmd = logGetVarId("feat2cmd", "Vycmd");
        id_vx_nom = logGetVarId("feat2cmd", "Vxnom");
        id_vy_nom = logGetVarId("feat2cmd", "Vynom");
        id_Stopped = logGetVarId("feat2cmd", "stopped");
        id_SlwDwnFct = logGetVarId("feat2cmd", "SlwDwnFct");
    }
    else if (mr_init_val == 3) 
    {
        DEBUG_PRINT("** No Multi-Ranger of and kind \n");
    }

    STATIC_MEM_TASK_ALLOC(telem, TELEMETRY_APP_STACKSIZE);
    STATIC_MEM_TASK_CREATE(telem, telem, "telem", NULL, TELEMETRY_APP_PRIORITY);

    id_OF_OL = logGetVarId("motion", "max_OL");
    id_OF_IL = logGetVarId("motion", "OF_valid");
    id_stab_stop = logGetVarId("stabilizer", "stop");
    id_stab_ready = logGetVarId("stabilizer", "stabReady");
    id_z_est = logGetVarId("kalman", "stateZ");

    #ifdef INVERTED_FLIGHT_MODE
        id_bottom_tof = logGetVarId("mr18" , "m16");
    #else
        id_bottom_tof = logGetVarId("range" , "zrange");
    #endif

    id_safety_tof = logGetVarId("mr18" , "m16");
    id_motorTestCount = logGetVarId("health" , "motorTestCount");
    id_motorPass = logGetVarId("health" , "motorPass");

    id_ExtState_x = logGetVarId("stateEstimateZ" , "x");
    id_ExtState_y = logGetVarId("stateEstimateZ" , "y");
    id_ExtState_z = logGetVarId("stateEstimateZ" , "z");
    id_ExtState_vz = logGetVarId("stateEstimate", "vz");
    id_ExtState_roll = logGetVarId("stateEstimate", "roll");
    id_ExtState_pitch = logGetVarId("stateEstimate", "pitch");
    id_ExtState_yaw = logGetVarId("stateEstimate", "yaw");

    // Param. Var.
    id_motor_power_enb = paramGetVarId("motorPowerSet", "enable");
    id_m1_power = paramGetVarId("motorPowerSet", "m1");
    id_m2_power = paramGetVarId("motorPowerSet", "m2");
    id_m3_power = paramGetVarId("motorPowerSet", "m3");
    id_m4_power = paramGetVarId("motorPowerSet", "m4");
    id_led_power = paramGetVarId("sound", "ratio");
    id_Auto_Flight = paramGetVarId("FF_Angle", "AUTO_FLIGHT");
    id_Reset_Est = paramGetVarId("kalman", "resetEstimation");
    id_startPropTest = paramGetVarId("health", "startPropTest");

    idFF = logGetVarId("sitAw", "FFAccWZDetected");
    autonomous_pilot();
}

LOG_GROUP_START(state_machine)
LOG_ADD(LOG_UINT8, P1_St_log, &Phase1_SubState_var)
LOG_ADD(LOG_FLOAT, vnom_sign, &vnom_sign)
LOG_ADD(LOG_UINT8, SM_St_log, &SM_State_var)
LOG_ADD(LOG_UINT8, ST_St_log, &state)
LOG_ADD(LOG_UINT8, allw_OA, &allw_OA)
LOG_ADD(LOG_UINT8, allw_hover, &allw_hover)
LOG_ADD(LOG_UINT8, TstSlwDwn, &TstSlwDwn)
LOG_ADD(LOG_FLOAT, height_sp, &height_sp)
LOG_ADD(LOG_FLOAT, vx_sp, &vx_sp)
LOG_ADD(LOG_FLOAT, vy_sp, &vy_sp)
LOG_ADD(LOG_INT8, safety_fa, &safety_fa)
LOG_ADD(LOG_UINT8, ExtKalman, &ExtKalman)
LOG_ADD(LOG_FLOAT, loco_err, &loco_ctl_err)
LOG_ADD(LOG_UINT8, rch_fnl_wp, &reached_final_wp)
LOG_ADD(LOG_UINT8, Gfnc_brch, &geofence_breached)
LOG_ADD(LOG_UINT8, Gfnc_cntr, &geofence_counter)
LOG_ADD(LOG_FLOAT, ExtState_x, &ExtState_x)
LOG_ADD(LOG_FLOAT, ExtState_y, &ExtState_y)
LOG_ADD(LOG_FLOAT, ExtState_z, &ExtState_z)
LOG_ADD(LOG_UINT8, loco_ii, &loco_ii)
LOG_ADD(LOG_UINT8, mainState, &state)
LOG_GROUP_STOP(state_machine)

PARAM_GROUP_START(state_machine)
PARAM_ADD(PARAM_FLOAT, lag_timer, &lag_timer)
PARAM_ADD(PARAM_FLOAT, height_sp, &height_sp)
PARAM_ADD(PARAM_FLOAT, vx_sp, &vx_sp)
PARAM_ADD(PARAM_FLOAT, vy_sp, &vy_sp)
PARAM_ADD(PARAM_FLOAT, psi_sp, &psi_sp)
PARAM_ADD(PARAM_FLOAT, PNG_k_psi, &PNG_k_psi)
PARAM_ADD(PARAM_FLOAT, max_psi_cmd, &max_psi_cmd)
PARAM_ADD(PARAM_UINT8, estop, &emergencyStop)
PARAM_ADD(PARAM_UINT8, launch, &launch_flag)
PARAM_ADD(PARAM_INT8, Bolt_Prio, &Bolt_Prio)
PARAM_ADD(PARAM_INT8, OPER_flag, &OPER_flag)
PARAM_ADD(PARAM_UINT8, SM_St_param, &SM_State_var)
PARAM_ADD(PARAM_UINT8, ST_St_param, &state)
PARAM_ADD(PARAM_UINT8, trn_arnd_fl, &trn_arnd_fl)
PARAM_ADD(PARAM_UINT8, go_bkwrd_fl, &go_bkwrd_fl)
PARAM_ADD(PARAM_UINT8, allw_OF_ret, &allw_OF_ret)
PARAM_ADD(PARAM_UINT8, allw_Z_ret, &allw_Z_ret)
PARAM_ADD(PARAM_UINT8, allw_W_ret, &allw_W_ret)
PARAM_ADD(PARAM_UINT8, allw_OA, &allw_OA)
PARAM_ADD(PARAM_UINT8, MN_AUTO_PRP, &MAN_AUTO_PROPTEST)
PARAM_ADD(PARAM_UINT32, prep_trn_dt, &prep_turn_dt)
PARAM_ADD(PARAM_UINT16, flr_clrnce, &flr_clrnce)
PARAM_ADD(PARAM_INT8, safety_fa, &safety_fa)
PARAM_ADD(PARAM_INT8, timeout, &timeout)
PARAM_ADD(PARAM_UINT8, led_power, &led_power)
PARAM_ADD(PARAM_FLOAT, loco_crad, &loco_ctl_radius)
PARAM_GROUP_STOP(state_machine)
