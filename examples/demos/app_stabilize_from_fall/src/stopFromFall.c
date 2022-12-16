/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 Bitcraze AB
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
 *
 * push.c - App layer application of the onboard push demo. The crazyflie 
 * has to have the multiranger and the flowdeck version 2.
 */

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"

#include "commander.h"

#include "FreeRTOS.h"
#include "task.h"
#include "portmacro.h"

#include "debug.h"

#include "log.h"
#include "param.h"
#include "estimator.h"

#define DEBUG_MODULE "PUSH"

static void setHoverSetpoint(setpoint_t *setpoint, float vx, float vy, float z, float yawrate)
{
  setpoint->mode.z = modeAbs;
  setpoint->position.z = z;


  setpoint->mode.yaw = modeVelocity;
  setpoint->attitudeRate.yaw = yawrate;


  setpoint->mode.x = modeVelocity;
  setpoint->mode.y = modeVelocity;
  setpoint->velocity.x = vx;
  setpoint->velocity.y = vy;

  setpoint->velocity_body = true;
}


static void setAngleSetpoint(setpoint_t *setpoint, float roll, float pitch)
{
  setpoint->mode.x = modeDisable;
  setpoint->mode.y = modeDisable;
  setpoint->mode.z = modeDisable;

  setpoint->mode.roll = modeAbs;
  setpoint->mode.pitch = modeAbs;
  setpoint->attitude.roll = roll;
  setpoint->attitude.pitch = pitch;
  //setpoint->thrust = 60000;
}

typedef enum {
    idle,
    falling,
    hovering,
    landing,
    stopping,
    climb
} State;

static State state = idle;

static const uint16_t unlockThLow = 100;
static const uint16_t unlockThHigh = 300;
static const uint16_t stoppedTh = 500;

static float topHeight = 0.700f;

static const float velMax = 1.0f;
static const uint16_t radius = 300;

static uint32_t curTimems = 0;
static uint16_t waitTime=2000;
static uint32_t hoverTime = 3000;
static bool forceStart = false;  
static bool landHard = false;
static uint16_t fallTime = 100;  // [ms]

#define MAX(a,b) ((a>b)?a:b)
#define MIN(a,b) ((a<b)?a:b)

void goAndWait(const float zTar, const float waitTime)
{
  static setpoint_t setpoint;
  memset(&setpoint, 0, sizeof(setpoint_t));
  uint32_t curTime = T2M(xTaskGetTickCount());
  uint32_t endTime = curTime+waitTime;

  while (curTime <endTime)
  {
    setHoverSetpoint(&setpoint, 0, 0, zTar, 0);
    commanderSetSetpoint(&setpoint, 3);
    vTaskDelay(M2T(10));
    curTime = T2M(xTaskGetTickCount());
  }
}

void appMain()
{
  static setpoint_t setpoint;
  uint32_t fallTimems=0, landTimems=0;


  vTaskDelay(M2T(500));

  logVarId_t idFF = logGetVarId("sitAw", "FFAccWZDetected");
  logVarId_t idx = logGetVarId("stateEstimate", "x");
  logVarId_t idy = logGetVarId("stateEstimate", "y");
  logVarId_t idz = logGetVarId("stateEstimate", "z");
  logVarId_t idvz = logGetVarId("stateEstimate", "vz");
  logVarId_t idroll = logGetVarId("stateEstimate", "roll");
  logVarId_t idpitch = logGetVarId("stateEstimate", "pitch");
  logVarId_t idyaw = logGetVarId("stateEstimate", "yaw");
  logVarId_t idup = logGetVarId("range", "up");
  logVarId_t idtumbled = logGetVarId("tumbledFlight", "forceStart");
  
  fallTimems = landTimems = curTimems = T2M(xTaskGetTickCount()); 

  // paramSetInt(paramGetVarId("stabilizer","estimator"),complementaryEstimator); //Estimator type Any(0), complementary(1), kalman(2) 
  // paramSetInt(paramGetVarId("valCompEsti","useTOF"),false); //Estimator type Any(0), complementary(1), kalman(2) 
  // paramSetInt(paramGetVarId("motion","disable"),true);  // disable flowdeck

  //paramSetInt(paramGetVarId("stabilizer","estimator"),kalmanEstimator); //Estimator type Any(0), complementary(1), kalman(2) 
  //paramSetInt(paramGetVarId("valCompEsti","useTOF"),true); //Estimator type Any(0), complementary(1), kalman(2) 
  //paramSetInt(paramGetVarId("motion","disable"),false);  // enable flowdeck

  float xTar, yTar, zTar;
  float xCur, yCur, zCur, vzCur, height, rollCur, pitchCur, yawCur;
  uint16_t upCur;
    int times[3]={0,0,0};

  bool singleFlag[] = {false, false, true, false, false};
  DEBUG_PRINT("**********  STOP FROM FALL: Waiting for activation **************\n");
 
  while(1) 
  {
    vTaskDelay(M2T(10));
    xCur = logGetFloat(idx);
    yCur = logGetFloat(idy);
    zCur = logGetFloat(idz);
    vzCur = logGetFloat(idvz);
    rollCur = logGetFloat(idroll);
    pitchCur = logGetFloat(idpitch);
    yawCur = logGetFloat(idyaw);
    upCur = logGetUint(idup);

    if (!singleFlag[2])
    {
      singleFlag[2] = true;
      zTar=zCur;
      DEBUG_PRINT("** TOF = %d [ms], zTar=%f  ", (curTimems-fallTimems), zTar);
    }

    curTimems = T2M(xTaskGetTickCount());
    //DEBUG_PRINT(".");

    uint8_t ffIndicator = logGetUint(idFF);
    if (forceStart)
    {
        forceStart = false;
        zTar = zCur+topHeight;
        paramSetInt(paramGetVarId("usd","logging"),1);
        DEBUG_PRINT("**********  climb from Z = %f to %f **************\n", zCur, zTar);
        DEBUG_PRINT("**********  topHeight %f waitTime %d fallTime %d **************\n", topHeight, waitTime, fallTime);
        goAndWait(zTar, waitTime);
        times[0]= T2M(xTaskGetTickCount());
        DEBUG_PRINT("**********  time before fall %d **************\n", times[0]);
        memset(&setpoint, 0, sizeof(setpoint_t));
        commanderSetSetpoint(&setpoint, 3);
        vTaskDelay(M2T(fallTime));
        times[1]= T2M(xTaskGetTickCount());
        DEBUG_PRINT("**********  time after fall %d **************\n", times[1]);
        
        fallTimems=T2M(xTaskGetTickCount());
        // xTar = logGetFloat(idx);
        // yTar = logGetFloat(idy);
        zTar = logGetFloat(idz);
        state = falling; // hovering
    }
    if (state == idle) 
    {
     if (ffIndicator > 0)// || forceStart)
      {
        times[2]= T2M(xTaskGetTickCount());
        DEBUG_PRINT("**********  SFF: Free fall detected, Target is z=%f, up=%d, Time=%d [ms]**************\n", zTar, upCur, times[2]);
        if (times[0] > 0 && (times[2]-times[1])>900)
        {
          DEBUG_PRINT("**********  Too late to start hovering **************\n");
          continue;
        }  
        state = falling;
        fallTimems = curTimems;
        xTar = xCur;
        yTar = yCur;
        zTar = zCur;
        
        memset(&setpoint, 0, sizeof(setpoint_t));
      }
    }
    if (state == falling)
    {
       if (fabs(rollCur) > 10 || fabs(pitchCur) > 10)
       {
         //setAngleSetpoint(&setpoint, 0, 0);
         //zTar = zCur;
         setpoint.mode.z = modeVelocity;
         setpoint.velocity.z = 0;
         //setHoverSetpoint(&setpoint, 0, 0, zTar, 0);
         commanderSetSetpoint(&setpoint, 3);
         //DEBUG_PRINT("* r=%f, p=%f TOF = %d [ms] **\n",  rollCur, pitchCur, (curTimems-fallTimems));
       }
       else
       {
         state = hovering;
         fallTimems=curTimems;
         memset(&setpoint, 0, sizeof(setpoint_t));
         DEBUG_PRINT("**Falling roll=%f, pitch=%f, z=%f TOF = %d [ms] **\n",  rollCur, pitchCur, zCur, T2M(xTaskGetTickCount()));
       }
    }
    if (state == hovering)
    {
      if ((curTimems-fallTimems) < hoverTime )
      {
        zTar = zCur;
        setHoverSetpoint(&setpoint, 0, 0, zTar, 0);
        //setpoint.mode.z = modeVelocity;
        //setpoint.velocity.z = 0;

        commanderSetSetpoint(&setpoint, 3);
      }
      else
      {
       if (fabs(zCur-zTar)<0.1)
        {
          state = landHard?stopping:landing;
          DEBUG_PRINT("** Hover OK, Landing z=%f, TOF = %d [ms] **\n",  zCur, (curTimems-fallTimems));
        }
        else
        {
          state = stopping;
          DEBUG_PRINT("** Hover BAD, Stopping z=%f-->%f, TOF = %d [ms]**\n", zCur, zTar, (curTimems-fallTimems));
        }
        memset(&setpoint, 0, sizeof(setpoint_t));
      }
    }
    if (state == landing)
    {       
        height = zCur*4/10;
               
        DEBUG_PRINT("** Landing, z=%f-->%f, TOF = %d [ms]**\n", zCur, height,(curTimems-fallTimems));
        setHoverSetpoint(&setpoint, 0, 0, height, 0);
        commanderSetSetpoint(&setpoint, 3);
        if (zCur < 0.1f) 
        {
          state = stopping;
          memset(&setpoint, 0, sizeof(setpoint_t));
          DEBUG_PRINT("X\n");
        }
        vTaskDelay(M2T(100));
    }
    if (state == stopping)
    {
        DEBUG_PRINT("** Stopping, z=%f-->%f, TOF = %d [ms]**\n", zCur, zTar,(curTimems-fallTimems));
        memset(&setpoint, 0, sizeof(setpoint_t));
        commanderSetSetpoint(&setpoint, 3);
        state = idle;

        singleFlag[0]=false;    singleFlag[1]=false;    singleFlag[2]=false;    singleFlag[3]=false;    singleFlag[4]=false;
        // paramSetInt(paramGetVarId("motion","disable"),true);  //disable flowdeck
        // paramSetInt(paramGetVarId("stabilizer","estimator"),complementaryEstimator); //Estimator type Any(0), complementary(1), kalman(2) 
        // paramSetInt(paramGetVarId("valCompEsti","useTOF"),false); //Estimator type Any(0), complementary(1), kalman(2) 
        paramSetInt(paramGetVarId("usd","logging"),0);
        vTaskDelay(M2T(10));
    }
  }
}
       
PARAM_GROUP_START(valSFall)   ///!!! VALENTIN ADD  sum of lengths of paramgroup and param names must not exceed 24 letters 
PARAM_ADD(PARAM_UINT8, forceStart, &forceStart)
PARAM_ADD(PARAM_UINT8, landHard, &landHard)
PARAM_ADD(PARAM_UINT16, fallTime, &fallTime)
PARAM_ADD(PARAM_UINT16, waitTime, &waitTime)
PARAM_ADD(PARAM_UINT16, hoverTime, &hoverTime)
PARAM_ADD(PARAM_FLOAT, topHeight, &topHeight)
PARAM_GROUP_STOP(valSFall)
