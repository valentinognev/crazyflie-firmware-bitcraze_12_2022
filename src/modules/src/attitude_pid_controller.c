/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * attitude_pid_controller.c: Attitude controller using PID correctors
 */
#include <stdbool.h>

#include "stabilizer_types.h"
#include "FreeRTOS.h"
#include "task.h"

#include "attitude_controller.h"
#include "pid.h"
#include "param.h"
#include "log.h"
#include "commander.h"
#include "platform_defaults.h"
#include "debug.h"
#include "math.h" // fmax, fmin

#define DEBUG_MODULE "ATTITUDE_PID_CONTROLLER"

#define ATTITUDE_LPF_CUTOFF_FREQ      15.0f
#define ATTITUDE_LPF_ENABLE false
#define ATTITUDE_RATE_LPF_CUTOFF_FREQ 30.0f
#define ATTITUDE_RATE_LPF_ENABLE false

static bool attFiltEnable = ATTITUDE_LPF_ENABLE;
static bool rateFiltEnable = ATTITUDE_RATE_LPF_ENABLE;
static float attFiltCutoff = ATTITUDE_LPF_CUTOFF_FREQ;
static float omxFiltCutoff = ATTITUDE_ROLL_RATE_LPF_CUTOFF_FREQ;
static float omyFiltCutoff = ATTITUDE_PITCH_RATE_LPF_CUTOFF_FREQ;
static float omzFiltCutoff = ATTITUDE_YAW_RATE_LPF_CUTOFF_FREQ;
static float yawMaxDelta = YAW_MAX_DELTA;


static inline int16_t saturateSignedInt16(float in)
{
  // don't use INT16_MIN, because later we may negate it, which won't work for that value.
  if (in > INT16_MAX)
    return INT16_MAX;
  else if (in < -INT16_MAX)
    return -INT16_MAX;
  else
    return (int16_t)in;
}

PidObject pidRollRate = {
  .kp = PID_ROLL_RATE_KP,
  .ki = PID_ROLL_RATE_KI,
  .kd = PID_ROLL_RATE_KD,
  .kff = PID_ROLL_RATE_KFF,
};

PidObject pidPitchRate = {
  .kp = PID_PITCH_RATE_KP,
  .ki = PID_PITCH_RATE_KI,
  .kd = PID_PITCH_RATE_KD,
  .kff = PID_PITCH_RATE_KFF,
};

PidObject pidYawRate = {
  .kp = PID_YAW_RATE_KP,
  .ki = PID_YAW_RATE_KI,
  .kd = PID_YAW_RATE_KD,
  .kff = PID_YAW_RATE_KFF,
};

PidObject pidRoll = {
  .kp = PID_ROLL_KP,
  .ki = PID_ROLL_KI,
  .kd = PID_ROLL_KD,
  .kff = PID_ROLL_KFF,
};

PidObject pidPitch = {
  .kp = PID_PITCH_KP,
  .ki = PID_PITCH_KI,
  .kd = PID_PITCH_KD,
  .kff = PID_PITCH_KFF,
};

PidObject pidYaw = {
  .kp = PID_YAW_KP,
  .ki = PID_YAW_KI,
  .kd = PID_YAW_KD,
  .kff = PID_YAW_KFF,
};

static int16_t rollOutput;
static int16_t pitchOutput;
static int16_t yawOutput;

// FF / OVD
static uint8_t TEST_PITCH = 0;
static uint8_t TEST_PITCH_PREV = 0;
static uint8_t TEST_ROLL = 0;
static uint8_t TEST_ROLL_PREV = 0;
static uint32_t scan_time = 0;
static uint32_t scan_time_cov = 0;
static uint32_t d_time = 0;
// 0 = Vel Control Only // 1 = Vel Control + Angle Avoidance ML&LL
// 2 = Only Angle Avoidance ML&LL // 3 = Only Angle Avoidance ML // 4 = Only Angle Avoidance LL
static uint8_t OVD_ANG_CMD = 1;
static uint8_t AUTO_FLIGHT = 1;
static uint16_t id_pitch = 0;
static uint16_t id_of_il;
static uint16_t id_roll = 0;
static uint16_t id_ml_pitch = 0;
static uint16_t id_ml_roll = 0;
static uint16_t id_allw_OA = 0;
static uint16_t id_allw_hover = 0;
static uint16_t id_var_vx = 0;
static uint16_t id_var_vy = 0;
static uint16_t id_z = 0;
static uint8_t allw_OA = 0;
static uint8_t allw_hover = 0;
static uint8_t LOTF_TO = 0;
static float pitch_FF = 0;
static uint16_t OF_il_16 = 0U;
static float roll_FF = 0;
static float pitch_ml_FF = 0;
static float roll_ml_FF = 0;
static float var_vx = 0;
static float var_avg = 0;
static float diff_gap_inv = 0;
static float var_vy = 0;
static float estZ = 0;
static float HL_ratio = 0.0f;
static float cov_pass = 3e-5f;
static float cov_cutOff = 6e-5f;
// expected Vxy covarianve as a function of height (@std(flow)==0.25)
static float pp[2] = {0.000210875636385623, -2.76639216536554e-05};

static float min_cmd_pass = 0.0f;
static float pitch_HL = 0;
static float roll_HL = 0;
static float pitch_cmd = 0;
static float roll_cmd = 0;
static float ptch_HL_flt = 0;
static float roll_HL_flt = 0;
static float TO_angle = 5.0; // deg
static bool isInit;

void attitudeControllerInit(const float updateDt)
{
  if(isInit)
    return;

  //TODO: get parameters from configuration manager instead - now (partly) implemented
  pidInit(&pidRollRate,  0, pidRollRate.kp,  pidRollRate.ki,  pidRollRate.kd,
       pidRollRate.kff,  updateDt, ATTITUDE_RATE, omxFiltCutoff, rateFiltEnable);
  pidInit(&pidPitchRate, 0, pidPitchRate.kp, pidPitchRate.ki, pidPitchRate.kd,
       pidPitchRate.kff, updateDt, ATTITUDE_RATE, omyFiltCutoff, rateFiltEnable);
  pidInit(&pidYawRate,   0, pidYawRate.kp,   pidYawRate.ki,   pidYawRate.kd,
       pidYawRate.kff,   updateDt, ATTITUDE_RATE, omzFiltCutoff, rateFiltEnable);

  pidSetIntegralLimit(&pidRollRate,  PID_ROLL_RATE_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&pidPitchRate, PID_PITCH_RATE_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&pidYawRate,   PID_YAW_RATE_INTEGRATION_LIMIT);

  pidInit(&pidRoll,  0, pidRoll.kp,  pidRoll.ki,  pidRoll.kd,  pidRoll.kff,  updateDt,
      ATTITUDE_RATE, attFiltCutoff, attFiltEnable);
  pidInit(&pidPitch, 0, pidPitch.kp, pidPitch.ki, pidPitch.kd, pidPitch.kff, updateDt,
      ATTITUDE_RATE, attFiltCutoff, attFiltEnable);
  pidInit(&pidYaw,   0, pidYaw.kp,   pidYaw.ki,   pidYaw.kd,   pidYaw.kff,   updateDt,
      ATTITUDE_RATE, attFiltCutoff, attFiltEnable);

  pidSetIntegralLimit(&pidRoll,  PID_ROLL_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&pidPitch, PID_PITCH_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&pidYaw,   PID_YAW_INTEGRATION_LIMIT);

  id_pitch = logGetVarId("LLAVD", "pitch_LLAVD");
  id_of_il = logGetVarId("motion", "numIL_16");
  id_roll = logGetVarId("LLAVD", "roll_LLAVD");
  id_ml_pitch = logGetVarId("LLAVD", "pitch_MLAVD");
  id_ml_roll = logGetVarId("LLAVD", "roll_MLAVD");
  id_allw_OA = logGetVarId("state_machine", "allw_OA");
  id_allw_hover = logGetVarId("state_machine", "allw_hover");
  id_var_vx = logGetVarId("kalman", "varPX");
  id_var_vy = logGetVarId("kalman", "varPY");
  id_z = logGetVarId("kalman", "stateZ");

  diff_gap_inv = 1.0f / (cov_cutOff - cov_pass);
  scan_time_cov = xTaskGetTickCount();
  isInit = true;
}

bool attitudeControllerTest()
{
  return isInit;
}

void attitudeControllerCorrectRatePID(
       float rollRateActual, float pitchRateActual, float yawRateActual,
       float rollRateDesired, float pitchRateDesired, float yawRateDesired)
{
  pidSetDesired(&pidRollRate, rollRateDesired);
  rollOutput = saturateSignedInt16(pidUpdate(&pidRollRate, rollRateActual, true));

  pidSetDesired(&pidPitchRate, pitchRateDesired);
  pitchOutput = saturateSignedInt16(pidUpdate(&pidPitchRate, pitchRateActual, true));

  pidSetDesired(&pidYawRate, yawRateDesired);

  yawOutput = saturateSignedInt16(pidUpdate(&pidYawRate, yawRateActual, true));
}

int attitudeControllerCorrectAttitudePID(
       float eulerRollActual, float eulerPitchActual, float eulerYawActual,
       float eulerRollDesired, float eulerPitchDesired, float eulerYawDesired,
       float* rollRateDesired, float* pitchRateDesired, float* yawRateDesired)
{

  int resetPosPid = 0;
  // // Check if HL should be overridden
  // if ((allw_hover == 0) & (AUTO_FLIGHT))
  // {
  //   if (LOTF_TO)
  //   {
  //     eulerPitchDesired = -TO_angle; // Minus is Pitch Down (for LU)
  //     eulerRollDesired = 0.0f;
  //   }
  //   else
  //   {
  //     eulerPitchDesired = 0.0f;
  //     eulerRollDesired = 0.0f;
  //   }
  // }

  // //////////////////
  // // LL Avoidance //
  // //////////////////
  // pitch_HL = eulerPitchDesired;
  // roll_HL = eulerRollDesired;

  // // Profile Testing
  // if (TEST_PITCH == 1)
  // {
  //   if (TEST_PITCH != TEST_PITCH_PREV)
  //   {
  //     DEBUG_PRINT("FF_TEST: Began Pitch Test\n");
  //     scan_time = xTaskGetTickCount();
  //   }
  //   d_time = xTaskGetTickCount() - scan_time;
  //   if (d_time <= 500)
  //   {
  //     eulerPitchDesired = 4;
  //     eulerRollDesired = 0;
  //   }
  //   else if ((d_time > 500) && (d_time <= 1000))
  //   {
  //     eulerPitchDesired = -5;
  //     eulerRollDesired = 0;
  //   }
  //   else if (d_time > 1000)
  //   {
  //     eulerPitchDesired = 0;
  //     eulerRollDesired = 0;
  //     DEBUG_PRINT("FF_TEST: Completed Pitch Test\n");
  //     TEST_PITCH = 0;
  //   }
  // }

  // if (TEST_ROLL == 1)
  // {
  //   if (TEST_ROLL != TEST_ROLL_PREV)
  //   {
  //     DEBUG_PRINT("FF_TEST: Began Roll Test\n");
  //     scan_time = xTaskGetTickCount();
  //   }
  //   d_time = xTaskGetTickCount() - scan_time;
  //   if (d_time <= 500)
  //   {
  //     eulerPitchDesired = 0;
  //     eulerRollDesired = 4;
  //   }
  //   else if ((d_time > 500) && (d_time <= 1000))
  //   {
  //     eulerPitchDesired = 0;
  //     eulerRollDesired = -5;
  //   }
  //   else if (d_time > 1000)
  //   {
  //     eulerPitchDesired = 0;
  //     eulerRollDesired = 0;
  //     DEBUG_PRINT("FF_TEST: Completed Roll Test\n");
  //     TEST_ROLL = 0;
  //   }
  // }
  // TEST_PITCH_PREV = TEST_PITCH;
  // TEST_ROLL_PREV = TEST_ROLL;

  // OF_il_16 = logGetUint(id_of_il);
  // // Fetch Pitch-Roll Commands
  // pitch_FF = logGetFloat(id_pitch);
  // roll_FF = logGetFloat(id_roll);
  // pitch_ml_FF = logGetFloat(id_ml_pitch);
  // roll_ml_FF = logGetFloat(id_ml_roll);
  // allw_OA = logGetUint(id_allw_OA);
  // allw_hover = logGetUint(id_allw_hover);
  // var_vx = logGetFloat(id_var_vx);
  // var_vy = logGetFloat(id_var_vy);
  // estZ = fmaxf(fabsf(logGetFloat(id_z)), 0.1f);
  // var_avg = 0.5f * (var_vx + var_vy) - (pp[0] * estZ + pp[1]);

  // // diff_gap_inv = 1.0f/(cov_cutOff-cov_pass);
  // // Trigger OVD/OPER according to PARAM

  // if (OF_il_16 < 12U)
  // {
  //   HL_ratio = 0.0f;
  //   scan_time_cov = xTaskGetTickCount();
  // }
  // else if (OF_il_16 < 16U)
  // {
  //   HL_ratio = 0.8f;
  //   scan_time_cov = xTaskGetTickCount();
  // }
  // else
  // {
  //   HL_ratio = fmin(1.0f, HL_ratio);
  // }

  // // if (var_avg<cov_pass){
  // //   HL_ratio = fmin(1.0f,HL_ratio);
  // // }else if (var_avg<cov_cutOff){
  // //   HL_ratio = fmin((cov_cutOff-var_avg)*diff_gap_inv+min_cmd_pass*(var_avg-cov_pass)*diff_gap_inv,HL_ratio);
  // //   scan_time_cov = xTaskGetTickCount();
  // // }else{
  // //   HL_ratio = min_cmd_pass;
  // //   scan_time_cov = xTaskGetTickCount();
  // // }

  // if (T2M(xTaskGetTickCount() - scan_time_cov) > 40)
  // {
  //   scan_time_cov = xTaskGetTickCount();
  //   HL_ratio = fmin(HL_ratio + 0.03f, 1.0f);
  // }

  // if ((HL_ratio < 0.8f) & AUTO_FLIGHT)
  // {
  //   resetPosPid = 1;
  // }

  // ptch_HL_flt = eulerPitchDesired * HL_ratio;
  // roll_HL_flt = eulerRollDesired * HL_ratio;

  // if (AUTO_FLIGHT)
  // {
  //   if (allw_OA)
  //   {
  //     if (OVD_ANG_CMD == 1)
  //     {
  //       // FF = HL + ML + LL
  //       eulerPitchDesired = ptch_HL_flt + pitch_FF + pitch_ml_FF;
  //       eulerRollDesired = roll_HL_flt + roll_FF + roll_ml_FF;
  //     }
  //     else if (OVD_ANG_CMD == 2)
  //     {
  //       // OVD = ML + FL
  //       eulerPitchDesired = pitch_FF + pitch_ml_FF;
  //       eulerRollDesired = roll_FF + roll_ml_FF;
  //     }
  //     else if (OVD_ANG_CMD == 3)
  //     {
  //       // OVD = ML
  //       eulerPitchDesired = pitch_ml_FF;
  //       eulerRollDesired = roll_ml_FF;
  //     }

  //     else if (OVD_ANG_CMD == 4)
  //     {
  //       // OVD = LL
  //       eulerPitchDesired = pitch_FF;
  //       eulerRollDesired = roll_FF;
  //     }
  //   }
  //   else // in case the HL command is released before avoidance
  //   {
  //     if (LOTF_TO)
  //     {
  //       eulerPitchDesired = pitch_HL;
  //       eulerRollDesired = roll_HL;
  //     }
  //     else
  //     {
  //       eulerPitchDesired = ptch_HL_flt;
  //       eulerRollDesired = roll_HL_flt;
  //     }
  //   }
  // }

  // pitch_cmd = eulerPitchDesired;
  // roll_cmd = eulerRollDesired;
  /////////////////////////
  // End of LL Avoidance //
  /////////////////////////

  pidSetDesired(&pidRoll, eulerRollDesired);
  *rollRateDesired = pidUpdate(&pidRoll, eulerRollActual, true);

  // Update PID for pitch axis
  pidSetDesired(&pidPitch, eulerPitchDesired);
  *pitchRateDesired = pidUpdate(&pidPitch, eulerPitchActual, true);

  // Update PID for yaw axis
  float yawError;
  yawError = eulerYawDesired - eulerYawActual;
  if (yawError > 180.0f)
    yawError -= 360.0f;
  else if (yawError < -180.0f)
    yawError += 360.0f;
  pidSetError(&pidYaw, yawError);
  *yawRateDesired = pidUpdate(&pidYaw, eulerYawActual, false);

  return resetPosPid;
}

void attitudeControllerResetRollAttitudePID(void)
{
    pidReset(&pidRoll);
}

void attitudeControllerResetPitchAttitudePID(void)
{
    pidReset(&pidPitch);
}

void attitudeControllerResetAllPID(void)
{
  pidReset(&pidRoll);
  pidReset(&pidPitch);
  pidReset(&pidYaw);
  pidReset(&pidRollRate);
  pidReset(&pidPitchRate);
  pidReset(&pidYawRate);
}

void attitudeControllerGetActuatorOutput(int16_t* roll, int16_t* pitch, int16_t* yaw)
{
  *roll = rollOutput;
  *pitch = pitchOutput;
  *yaw = yawOutput;
}

float attitudeControllerGetYawMaxDelta(void)
{
  return yawMaxDelta;
}

/**
 *  Log variables of attitude PID controller
 */ 
LOG_GROUP_START(pid_attitude)
/**
 * @brief Proportional output roll
 */
LOG_ADD(LOG_FLOAT, roll_outP, &pidRoll.outP)
/**
 * @brief Integral output roll
 */
LOG_ADD(LOG_FLOAT, roll_outI, &pidRoll.outI)
/**
 * @brief Derivative output roll
 */
LOG_ADD(LOG_FLOAT, roll_outD, &pidRoll.outD)
/**
 * @brief Feedforward output roll
 */
LOG_ADD(LOG_FLOAT, roll_outFF, &pidRoll.outFF)
/**
 * @brief Proportional output pitch
 */
LOG_ADD(LOG_FLOAT, pitch_outP, &pidPitch.outP)
/**
 * @brief Integral output pitch
 */
LOG_ADD(LOG_FLOAT, pitch_outI, &pidPitch.outI)
/**
 * @brief Derivative output pitch
 */
LOG_ADD(LOG_FLOAT, pitch_outD, &pidPitch.outD)
/**
 * @brief Feedforward output pitch
 */
LOG_ADD(LOG_FLOAT, pitch_outFF, &pidPitch.outFF)
/**
 * @brief Proportional output yaw
 */
LOG_ADD(LOG_FLOAT, yaw_outP, &pidYaw.outP)
/**
 * @brief Intergal output yaw
 */
LOG_ADD(LOG_FLOAT, yaw_outI, &pidYaw.outI)
/**
 * @brief Derivative output yaw
 */
LOG_ADD(LOG_FLOAT, yaw_outD, &pidYaw.outD)
/**
 * @brief Feedforward output yaw
 */
LOG_ADD(LOG_FLOAT, yaw_outFF, &pidYaw.outFF)
LOG_GROUP_STOP(pid_attitude)

/**
 *  Log variables of attitude rate PID controller
 */
LOG_GROUP_START(pid_rate)
/**
 * @brief Proportional output roll rate
 */
LOG_ADD(LOG_FLOAT, roll_outP, &pidRollRate.outP)
/**
 * @brief Integral output roll rate
 */
LOG_ADD(LOG_FLOAT, roll_outI, &pidRollRate.outI)
/**
 * @brief Derivative output roll rate
 */
LOG_ADD(LOG_FLOAT, roll_outD, &pidRollRate.outD)
/**
 * @brief Feedforward output roll rate
 */
LOG_ADD(LOG_FLOAT, roll_outFF, &pidRollRate.outFF)
/**
 * @brief Proportional output pitch rate
 */
LOG_ADD(LOG_FLOAT, pitch_outP, &pidPitchRate.outP)
/**
 * @brief Integral output pitch rate
 */
LOG_ADD(LOG_FLOAT, pitch_outI, &pidPitchRate.outI)
/**
 * @brief Derivative output pitch rate
 */
LOG_ADD(LOG_FLOAT, pitch_outD, &pidPitchRate.outD)
/**
 * @brief Feedforward output pitch rate
 */
LOG_ADD(LOG_FLOAT, pitch_outFF, &pidPitchRate.outFF)
/**
 * @brief Proportional output yaw rate
 */
LOG_ADD(LOG_FLOAT, yaw_outP, &pidYawRate.outP)
/**
 * @brief Integral output yaw rate
 */
LOG_ADD(LOG_FLOAT, yaw_outI, &pidYawRate.outI)
/**
 * @brief Derivative output yaw rate
 */
LOG_ADD(LOG_FLOAT, yaw_outD, &pidYawRate.outD)
/**
 * @brief Feedforward output yaw rate
 */
LOG_ADD(LOG_FLOAT, yaw_outFF, &pidYawRate.outFF)
LOG_GROUP_STOP(pid_rate)

LOG_GROUP_START(FF_Angle)
LOG_ADD(LOG_FLOAT, pitch_HL, &pitch_HL)
LOG_ADD(LOG_FLOAT, roll_HL, &roll_HL)
LOG_ADD(LOG_FLOAT, pitch_cmd, &pitch_cmd)
LOG_ADD(LOG_FLOAT, roll_cmd, &roll_cmd)
LOG_ADD(LOG_FLOAT, HL_ratio, &HL_ratio)
LOG_ADD(LOG_FLOAT, var_avg, &var_avg)
LOG_ADD(LOG_FLOAT, ptch_HL_flt, &ptch_HL_flt)
LOG_ADD(LOG_FLOAT, roll_HL_flt, &roll_HL_flt)
LOG_GROUP_STOP(FF_Angle)

PARAM_GROUP_START(FF_Angle)
PARAM_ADD(PARAM_UINT8, TEST_PITCH, &TEST_PITCH)
PARAM_ADD(PARAM_UINT8, TEST_ROLL, &TEST_ROLL)
PARAM_ADD(PARAM_UINT8, OVD_ANG_CMD, &OVD_ANG_CMD)
PARAM_ADD(PARAM_FLOAT, covPass, &cov_pass)
PARAM_ADD(PARAM_FLOAT, covCutOff, &cov_cutOff)
PARAM_ADD(PARAM_FLOAT, pp0, &pp[0])
PARAM_ADD(PARAM_FLOAT, pp1, &pp[1])
PARAM_ADD(PARAM_FLOAT, minCmdPass, &min_cmd_pass)
PARAM_ADD(PARAM_UINT8, AUTO_FLIGHT, &AUTO_FLIGHT)
PARAM_ADD(PARAM_UINT8, LOTF_TO, &LOTF_TO)
PARAM_GROUP_STOP(FF_Angle)

/**
 * Tuning settings for the gains of the PID
 * controller for the attitude of the Crazyflie which consists
 * of the Yaw Pitch and Roll 
 */
PARAM_GROUP_START(pid_attitude)
/**
 * @brief Proportional gain for the PID roll controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, roll_kp, &pidRoll.kp)
/**
 * @brief Integral gain for the PID roll controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, roll_ki, &pidRoll.ki)
/**
 * @brief Derivative gain for the PID roll controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, roll_kd, &pidRoll.kd)
/**
 * @brief Feedforward gain for the PID roll controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, roll_kff, &pidRoll.kff)
/**
 * @brief Proportional gain for the PID pitch controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, pitch_kp, &pidPitch.kp)
/**
 * @brief Integral gain for the PID pitch controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, pitch_ki, &pidPitch.ki)
/**
 * @brief Derivative gain for the PID pitch controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, pitch_kd, &pidPitch.kd)
/**
 * @brief Feedforward gain for the PID pitch controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, pitch_kff, &pidPitch.kff)
/**
 * @brief Proportional gain for the PID yaw controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, yaw_kp, &pidYaw.kp)
/**
 * @brief Integral gain for the PID yaw controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, yaw_ki, &pidYaw.ki)
/**
 * @brief Derivative gain for the PID yaw controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, yaw_kd, &pidYaw.kd)
/**
 * @brief Feedforward gain for the PID yaw controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, yaw_kff, &pidYaw.kff)
/**
 * @brief If nonzero, yaw setpoint can only be set within +/- yawMaxDelta from the current yaw
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, yawMaxDelta, &yawMaxDelta)
/**
 * @brief Low pass filter enable
 */
PARAM_ADD(PARAM_INT8 | PARAM_PERSISTENT, attFiltEn, &attFiltEnable)
/**
 * @brief Low pass filter cut-off frequency (Hz)
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, attFiltCut, &attFiltCutoff)
PARAM_GROUP_STOP(pid_attitude)

/**
 * Tuning settings for the gains of the PID controller for the rate angles of
 * the Crazyflie, which consists of the yaw, pitch and roll rates 
 */
PARAM_GROUP_START(pid_rate)
/**
 * @brief Proportional gain for the PID roll rate controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, roll_kp, &pidRollRate.kp)
/**
 * @brief Integral gain for the PID roll rate controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, roll_ki, &pidRollRate.ki)
/**
 * @brief Derivative gain for the PID roll rate controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, roll_kd, &pidRollRate.kd)
/**
 * @brief Feedforward gain for the PID roll rate controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, roll_kff, &pidRollRate.kff)
/**
 * @brief Proportional gain for the PID pitch rate controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, pitch_kp, &pidPitchRate.kp)
/**
 * @brief Integral gain for the PID pitch rate controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, pitch_ki, &pidPitchRate.ki)
/**
 * @brief Derivative gain for the PID pitch rate controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, pitch_kd, &pidPitchRate.kd)
/**
 * @brief Feedforward gain for the PID pitch rate controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, pitch_kff, &pidPitchRate.kff)
/**
 * @brief Proportional gain for the PID yaw rate controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, yaw_kp, &pidYawRate.kp)
/**
 * @brief Integral gain for the PID yaw rate controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, yaw_ki, &pidYawRate.ki)
/**
 * @brief Derivative gain for the PID yaw rate controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, yaw_kd, &pidYawRate.kd)
/**
 * @brief Feedforward gain for the PID yaw rate controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, yaw_kff, &pidYawRate.kff)
/**
 * @brief Low pass filter enable
 */
PARAM_ADD(PARAM_INT8 | PARAM_PERSISTENT, rateFiltEn, &rateFiltEnable)
/**
 * @brief Low pass filter cut-off frequency, roll axis (Hz)
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, omxFiltCut, &omxFiltCutoff)
/**
 * @brief Low pass filter cut-off frequency, pitch axis (Hz)
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, omyFiltCut, &omyFiltCutoff)
/**
 * @brief Low pass filter cut-off frequency, yaw axis (Hz)
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, omzFiltCut, &omzFiltCutoff)
PARAM_GROUP_STOP(pid_rate)
