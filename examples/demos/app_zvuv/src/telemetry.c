/**
ZVUV
 */

#include "param.h"
#include "log.h"
#include "debug.h"
#include "FreeRTOS.h"
#include "task.h"


#define DEBUG_MODULE "TELEM"

static bool isInit = pdFALSE;

static uint8_t compPollPeriod = 50; //msec

static uint16_t id_vx = 0;
static uint16_t id_vy = 0;
static uint16_t id_z = 0;
static uint16_t id_squal = 0;
static uint16_t id_shutter = 0;
static uint16_t id_pitch = 0;
static uint16_t id_roll = 0;

static int8_t active = 0;
static uint8_t squal = 0;
static uint16_t shutter = 0;
static float vx = 0.0f;
static float vy = 0.0f;
static float z = 0.0f;
static float pitch = 0.0f;
static float roll = 0.0f;



void telem_init()
{
    if(isInit)
		return;
    
    id_vx = logGetVarId("kalman" , "statePX");
    id_vy = logGetVarId("kalman" , "statePY");
    id_z = logGetVarId("kalman" , "stateZ");
    id_squal = logGetVarId("motion" , "squal");
    id_shutter = logGetVarId("motion" , "shutter");
    id_pitch = logGetVarId("stateEstimate" , "roll");
    id_roll = logGetVarId("stateEstimate" , "pitch");

    isInit = pdTRUE;
}

void telem()
{
    telem_init();

    TickType_t lastWakeTime = xTaskGetTickCount();

    while (1)
    {
        vTaskDelayUntil(&lastWakeTime, M2T(compPollPeriod));
        
        if (active)
        {
            vx = logGetFloat(id_vx);
            vy = logGetFloat(id_vy);
            z = logGetFloat(id_z);
            squal = logGetUint(id_squal); //8
            shutter = logGetUint(id_shutter); //16
            pitch = logGetFloat(id_pitch);
            roll = logGetFloat(id_roll);
        }
    }

    // DEBUG_PRINT("Test= %f\n", test);
}

PARAM_GROUP_START(telem)
PARAM_ADD(PARAM_FLOAT, vx_est, &vx)
PARAM_ADD(PARAM_FLOAT, vy_est, &vy)
PARAM_ADD(PARAM_FLOAT, z_est, &z)
PARAM_ADD(PARAM_UINT8, squal, &squal)
PARAM_ADD(PARAM_UINT16, shutter, &shutter)
PARAM_ADD(PARAM_FLOAT, pitch, &pitch)
PARAM_ADD(PARAM_FLOAT, roll, &roll)
PARAM_ADD(PARAM_INT8, active, &active)
PARAM_GROUP_STOP(telem)
