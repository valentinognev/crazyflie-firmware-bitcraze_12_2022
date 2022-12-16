/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2021 BitCraze AB
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
 * vl53l0x.c: Time-of-flight distance sensor driver
 */

#define DEBUG_MODULE "ZR2"

#include "FreeRTOS.h"
#include "task.h"

#include "config.h"
#include "deck.h"
#include "system.h"
#include "debug.h"
#include "log.h"
#include "param.h"
#include "range.h"
#include "static_mem.h"

#include "i2cdev.h"
#include "zranger2.h"
#include "vl53l1x.h"

#include "cf_math.h"

// Measurement noise model
static const float expPointA = 2.5f;
static const float expStdA = 0.0025f; // STD at elevation expPointA [m]
static const float expPointB = 4.0f;
static const float expStdB = 0.2f;    // STD at elevation expPointB [m]
static float expCoeff;

#define RANGE_OUTLIER_LIMIT 5000 // the measured range is in [mm]

static uint16_t range_last = 0;

static bool isInit;
static int8_t statusReadMr18;
static uint8_t rangeStatus;
static const uint8_t status_rtn[24] = {25, 25, 25, 5, 2, 4, 1, 7, 3, 0,
                                       25, 25, 9, 13, 25, 25, 25, 25, 10, 6,
                                       25, 25, 11, 12};

NO_DMA_CCM_SAFE_ZERO_INIT static VL53L1_Dev_t dev;

static uint16_t zRanger2GetMeasurementAndRestart(VL53L1_Dev_t *dev, bool *valid)
{
    VL53L1_Error status = VL53L1_ERROR_NONE;
    VL53L1_RangingMeasurementData_t rangingData;
    uint8_t dataReady = 0;
    uint16_t range;

    while (dataReady == 0)
    {
        status = VL53L1_GetMeasurementDataReady(dev, &dataReady);
        vTaskDelay(M2T(1));
    }
    // Read range status:
    uint8_t RgSt;
    statusReadMr18 = (int8_t)VL53L1_RdByte(dev, VL53L1_RESULT__RANGE_STATUS, &RgSt);
    RgSt = RgSt & 0x1F;

    rangeStatus = 25;
    status = VL53L1_RdByte(dev, VL53L1_RESULT__RANGE_STATUS, &RgSt);
    RgSt = RgSt & 0x1F;
    if (RgSt < 24)
      rangeStatus = status_rtn[RgSt];

    *valid = (rangeStatus == 0U) ? true : false;

    status = VL53L1_GetRangingMeasurementData(dev, &rangingData);
    range = rangingData.RangeMilliMeter;

    VL53L1_StopMeasurement(dev);
    status = VL53L1_StartMeasurement(dev);
    status = status;

    return range;
}

void zRanger2Init(DeckInfo* info)
{
  if (isInit)
    return;

  if (vl53l1xInit(&dev, I2C1_DEV))
  {
      DEBUG_PRINT("Z-down sensor [OK]\n");
  }
  else
  {
    DEBUG_PRINT("Z-down sensor [FAIL]\n");
    return;
  }

  xTaskCreate(zRanger2Task, ZRANGER2_TASK_NAME, ZRANGER2_TASK_STACKSIZE, NULL, ZRANGER2_TASK_PRI, NULL);

  // pre-compute constant in the measurement noise model for kalman
  expCoeff = logf(expStdB / expStdA) / (expPointB - expPointA);

  isInit = true;
}

bool zRanger2Test(void)
{
  if (!isInit)
    return false;

  return true;
}

void zRanger2Task(void* arg)
{
  TickType_t lastWakeTime;

  systemWaitStart();

  // Restart sensor
  VL53L1_StopMeasurement(&dev);
  VL53L1_SetDistanceMode(&dev, VL53L1_DISTANCEMODE_MEDIUM);
  // VL53L1_SetDistanceMode(&dev, VL53L1_DISTANCEMODE_LONG);
  VL53L1_SetMeasurementTimingBudgetMicroSeconds(&dev, 25000);

  VL53L1_StartMeasurement(&dev);

  lastWakeTime = xTaskGetTickCount();

  while (1) {
    vTaskDelayUntil(&lastWakeTime, M2T(25));
    bool valid = false;

    range_last = zRanger2GetMeasurementAndRestart(&dev, &valid);
    rangeSet(rangeDown, range_last / 1000.0f);

    // check if range is feasible and push into the estimator
    // the sensor should not be able to measure >5 [m], and outliers typically
    // occur as >8 [m] measurements
    if (range_last < RANGE_OUTLIER_LIMIT) 
    {
      float distance = (float)range_last * 0.001f; // Scale from [mm] to [m]
      float stdDev = expStdA * (1.0f  + expf( expCoeff * (distance - expPointA)));
  //    rangeEnqueueDownRangeInEstimator(distance, stdDev, xTaskGetTickCount());
      rangeEnqueueDownRangeInEstimatorNew(distance, stdDev, xTaskGetTickCount(), valid);
    }
    else
    {
      rangeEnqueueDownRangeInEstimatorNew(-1.0f, 100.f, xTaskGetTickCount(), valid);
    }
  }
}

static const DeckDriver zranger2_deck = {
  .vid = 0xBC,
  .pid = 0x0E,
  .name = "bcZRanger2",
  .usedGpio = 0,
  .usedPeriph = DECK_USING_I2C,

  .init = zRanger2Init,
  .test = zRanger2Test,
};

DECK_DRIVER(zranger2_deck);

PARAM_GROUP_START(deck)

/**
 * @brief Nonzero if [Z-ranger deck v2](%https://store.bitcraze.io/collections/decks/products/z-ranger-deck-v2) is attached
 */
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcZRanger2, &isInit)

PARAM_GROUP_STOP(deck)

LOG_GROUP_START(zrngInf2)
LOG_ADD(LOG_UINT8, rngStat, &rangeStatus)
LOG_GROUP_STOP(zrngInf2)

/*
    rangeStatus = VL53L1_RANGESTATUS_RANGE_VALID;  // 0
    rangeStatus = VL53L1_RANGESTATUS_SIGMA_FAIL;  // 1
    rangeStatus = VL53L1_RANGESTATUS_SIGNAL_FAIL;  // 2
    rangeStatus = VL53L1_RANGESTATUS_RANGE_VALID_MIN_RANGE_CLIPPED;  // 3
    rangeStatus = VL53L1_RANGESTATUS_OUTOFBOUNDS_FAIL;  // 4
    rangeStatus = VL53L1_RANGESTATUS_RANGE_VALID_NO_WRAP_CHECK_FAIL;  // 6
    rangeStatus = VL53L1_RANGESTATUS_WRAP_TARGET_FAIL;  // 7
    rangeStatus = VL53L1_RANGESTATUS_XTALK_SIGNAL_FAIL;  // 9
    rangeStatus = VL53L1_RANGESTATUS_SYNCRONISATION_INT;  // 10
    rangeStatus = VL53L1_RANGESTATUS_NONE;  // 255
*/