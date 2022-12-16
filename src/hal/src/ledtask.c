 // ledtask.c - LED frequency update 
 // led frequency update loop runs here
 // all leds that arent blue havnt been completely applied and should be upon future need

#include <stdint.h>
#include <stdbool.h>
#include <led.h>
#include <ledseq.h>
#include <param.h>
#include <log.h>
#include <debug.h>
#include <FreeRTOS.h>
#include <task.h>
#include <math.h>


#define DEBUG_MODULE "LedTask"

static bool isInit = pdFALSE;

static uint8_t compPollPeriod = 232; //msec
static int8_t active = 0;

static uint16_t id_vx = 0;
static uint16_t id_vy = 0;
static uint16_t id_shutter = 0;


void LedTask_init()
{
    if(isInit)
		return;
    
    id_vx = logGetVarId("kalman" , "statePX");
    id_vy = logGetVarId("kalman" , "statePY");
    id_shutter = logGetVarId("motion" , "shutter");
    //Firstly "clean" all previous activity in the leds
    ledseqStop(&seq_charged);  //Might be possible to omit
    ledseqStop(&seq_charging); //Might be possible to omit
    ledseqRunBlocking(&seq_gen_blu);             //Initiate base sequence
    isInit = pdTRUE;
}


void LedTask() {  

    DEBUG_PRINT("Ledtask init begins\n");
    LedTask_init();
    DEBUG_PRINT("Ledtask init is done\n");

    TickType_t lastWakeTime = xTaskGetTickCount();

    while (1) {

        vTaskDelayUntil(&lastWakeTime, M2T(compPollPeriod));
        if (active==1) {
            uint16_t val = logGetUint(id_shutter);
            if (val<800) {
                LedSeqUpdate(500);
            }
            if (val>=800 && val<2000) {
                LedSeqUpdate(200);
            }
            if (val>=2000 && val<4000) {
                LedSeqUpdate(100);
            }
            if (val>=4000 && val<6000) {
                LedSeqUpdate(75);
            }
            if (val>6000) {
                LedSeqUpdate(dtMIN);
            }           
        }

        if (active==2) {
            float valx = fabsf(logGetFloat(id_vx));
            //DEBUG_PRINT("Speed Estimation Y Sampled is %f !\n", valx);
            float valy = fabsf(logGetFloat(id_vy));
            //DEBUG_PRINT("Speed Estimation X Sampled is %f !\n", valy);
            float val;
            

            if (valx > valy) {
                val = valx;
            }
            else {
                val = valy;
            }

            //DEBUG_PRINT("Speed Estimation Sampled is %f !\n", val);
            
            if (val<0.15f) {
                LedSeqUpdate(700);
            }
            if (val>=0.15f && val<1.0f) {
                LedSeqUpdate(150); 
            }   
            if (val>=1.0f) {
                LedSeqUpdate(50); 
            }   
        }

        if (active==0) {
            LedDutyCycle(0); //Blank out the led   
        }

        if (active==3){
            LedDutyCycle(0); //Blank out the led
            LedDutyCycle_LB(0); //Blank out the led
            LedDutyCycle_CALIB(0); //Blank out the led
            LedDutyCycle_ALV(0); //Blank out the led
            LedDutyCycle_LINKU(0); //Blank out the led
            LedDutyCycle_LINKD(0); //Blank out the led
        }
    }
}

PARAM_GROUP_START(LedTaskG)
PARAM_ADD(PARAM_INT8, active, &active)
PARAM_GROUP_STOP(LedTaskG)