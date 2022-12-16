/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--'  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2021 Bitcraze AB
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
 */

#include "mm_flow.h"
#include "log.h"

// TODO remove the temporary test variables (used for logging)
static float predictedNX;
static float predictedNY;
static float measuredNX;
static float measuredNY;
// static float estFlowVarX=0.25f;
// static float estFlowVarY=0.25f;
static float residual_flowX = 0.0f;
// static float measNoise_flowX = 0.0f;
static float flow_dt;
static uint8_t flow_update;

void kalmanCoreUpdateWithFlow(kalmanCoreData_t* this, const flowMeasurement_t *flow, const Axis3f *gyro)
{
  // Inclusion of flow measurements in the EKF done by two scalar updates

  // ~~~ Camera constants ~~~
  // The angle of aperture is guessed from the raw data register and thankfully look to be symmetric
  float Npix = 30.0;                      // [pixels] (same in x and y)
  //float thetapix = DEG_TO_RAD * 4.0f;     // [rad]    (same in x and y)
  float thetapix = DEG_TO_RAD * 4.2f;
  //~~~ Body rates ~~~
  // TODO check if this is feasible or if some filtering has to be done
  #ifdef INVERTED_FLIGHT_MODE
    float omegax_b = gyro->x * DEG_TO_RAD * -1;  // Rotate angular axes (x, y) ZVUV
    float omegay_b = gyro->y * DEG_TO_RAD * -1;  // Rotate angular axes (x, y) ZVUV
  #else
    float omegax_b = gyro->x * DEG_TO_RAD ;  // Rotate angular axes (x, y) ZVUV
    float omegay_b = gyro->y * DEG_TO_RAD ;  // Rotate angular axes (x, y) ZVUV
  #endif

  // ~~~ Moves the body velocity into the global coordinate system ~~~
  // [bar{x},bar{y},bar{z}]_G = R*[bar{x},bar{y},bar{z}]_B
  //
  // \dot{x}_G = (R^T*[dot{x}_B,dot{y}_B,dot{z}_B])\dot \hat{x}_G
  // \dot{x}_G = (R^T*[dot{x}_B,dot{y}_B,dot{z}_B])\dot \hat{x}_G
  //
  // where \hat{} denotes a basis vector, \dot{} denotes a derivative and
  // _G and _B refer to the global/body coordinate systems.

  // Modification 1
  //dx_g = R[0][0] * S[KC_STATE_PX] + R[0][1] * S[KC_STATE_PY] + R[0][2] * S[KC_STATE_PZ];
  //dy_g = R[1][0] * S[KC_STATE_PX] + R[1][1] * S[KC_STATE_PY] + R[1][2] * S[KC_STATE_PZ];


  float dx_g = this->S[KC_STATE_PX];
  float dy_g = this->S[KC_STATE_PY];
  float z_g = 0.0;
  // Saturate elevation in prediction and correction to avoid singularities
  #ifdef INVERTED_FLIGHT_MODE

  if ( fabsf(this->S[KC_STATE_Z]) < 0.1f ) { // [[ZVUV]]
      z_g = 0.1;
  } else {
      z_g = fabsf(this->S[KC_STATE_Z]); // ZVUV
  }
  #else

  if ( (this->S[KC_STATE_Z]) < 0.1f ) { // [[ZVUV]]
      z_g = 0.1;
  } else {
      z_g = (this->S[KC_STATE_Z]); // ZVUV
  }

  #endif

  // ~~~ X velocity prediction and update ~~~
  // predics the number of accumulated pixels in the x-direction
  float omegaFactor = 1.25f;
  float hx[KC_STATE_DIM] = {0};
  arm_matrix_instance_f32 Hx = {1, KC_STATE_DIM, hx};
  predictedNX = (flow->dt * Npix / thetapix ) * ((dx_g * this->R[2][2] / z_g) - omegaFactor * omegay_b);
  measuredNX = flow->dpixelx;
  flow_dt = flow->dt;

  // derive measurement equation with respect to dx (and z?)
  hx[KC_STATE_Z] = (Npix * flow->dt / thetapix) * ((this->R[2][2] * dx_g) / (-z_g * z_g));
  hx[KC_STATE_PX] = (Npix * flow->dt / thetapix) * (this->R[2][2] / z_g);
  float std = flow->stdDevX;
  float innov;
  //First update
  if (flow->update)
  {
    flow_update = 1;
    innov = fabsf(measuredNX - predictedNX);
    if (flow->stdDevX < 2.0f && innov > 15.0f)
    {
      flow_update = 3;
      std = 2.0f;
    }
    else if (flow->stdDevX < 2.0f && innov > 10.0f)
    {
      flow_update = 2;
      std = 0.25f + 1.75f * (innov - 5.0f) / 5.0f;
    }
    this->measMotionVar = kalmanCoreScalarUpdate(this, &Hx, measuredNX - predictedNX, std);
    residual_flowX = (flow->dt * Npix / thetapix) * ((this->S[KC_STATE_PX] * this->R[2][2] / fabsf(this->S[KC_STATE_Z])) - omegaFactor * omegay_b) - flow->dpixelx;
    residual_flowX *= residual_flowX;
    this->estMotionVar = 0.99f * this->estMotionVar + 0.01f * (residual_flowX + this->measMotionVar);
  }
  else
  {
    flow_update = 0;
  }

  // ~~~ Y velocity prediction and update ~~~
  float hy[KC_STATE_DIM] = {0};
  arm_matrix_instance_f32 Hy = {1, KC_STATE_DIM, hy};
  predictedNY = (flow->dt * Npix / thetapix ) * ((dy_g * this->R[2][2] / z_g) + omegaFactor * omegax_b);
  #ifdef INVERTED_FLIGHT_MODE
    measuredNY = -flow->dpixely; //[[ZVUV]]
  #else
    measuredNY = flow->dpixely; //[[ZVUV]]
  #endif

  // derive measurement equation with respect to dy (and z?)
  hy[KC_STATE_Z] = (Npix * flow->dt / thetapix) * ((this->R[2][2] * dy_g) / (-z_g * z_g));
  hy[KC_STATE_PY] = (Npix * flow->dt / thetapix) * (this->R[2][2] / z_g);

  std = flow->stdDevY;
  // Second update
  if (flow->update)
  {
    innov = fabsf(measuredNY - predictedNY);
    if (flow->stdDevY < 2.0f && innov > 15.0f)
    {
      std = 2.0f;
    }
    else if (flow->stdDevY < 2.0f && innov > 10.0f)
    {
      std = 0.25f + 1.75f * (innov - 10.0f) / 5.0f;
    }
    kalmanCoreScalarUpdate(this, &Hy, measuredNY - predictedNY, flow->stdDevY);
  }
}

/**
 * Predicted and measured values of the X and Y direction of the flowdeck
 */
LOG_GROUP_START(kalman_pred)

/**
 * @brief Flow sensor predicted dx  [pixels/frame]
 * 
 *  note: rename to kalmanMM.flowX?
 */
  LOG_ADD(LOG_FLOAT, predNX, &predictedNX)
/**
 * @brief Flow sensor predicted dy  [pixels/frame]
 * 
 *  note: rename to kalmanMM.flowY?
 */
  LOG_ADD(LOG_FLOAT, predNY, &predictedNY)
/**
 * @brief Flow sensor measured dx  [pixels/frame]
 * 
 *  note: This is the same as motion.deltaX, so perhaps remove this?
 */
  LOG_ADD(LOG_FLOAT, measNX, &measuredNX)
/**
 * @brief Flow sensor measured dy  [pixels/frame]
 * 
 *  note: This is the same as motion.deltaY, so perhaps remove this?
 */
  LOG_ADD(LOG_FLOAT, measNY, &measuredNY)
LOG_GROUP_STOP(kalman_pred)

