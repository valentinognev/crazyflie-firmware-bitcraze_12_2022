
#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"

#include "system.h"
#include <string.h> //to enable memset command

#include "app.h"

#include "log.h"
#include "debug.h"
#define DEBUG_MODULE "MR18"

#include "i2cdev.h"
#include "cf_feat_ext.h"
#include "mr18_driver.h"
#include "log.h"

#include "feat2cmd.h"

#include "param.h"
#include <math.h>

// Log Variable IDs
static int psiId;
static int xId;
static int yId;
static int sgnId;
static int vx_spId;
static int vy_spId;
static int TstSlwDwnId;

static float modelRANSAC[3]= {0};
unsigned short ranges_cm[16]={0};
static boolean_T validData=0;
unsigned char slowDwn=0;
unsigned char slowDwn_feat2cmd=0;
static boolean_T stopped=0;
static boolean_T allow_90deg=0;

// LL Avoidance
static uint32_t TT = 0;
static float LLAvdCmd[2]={0};
static float rej_Cmd[2]={0};

// Slow-Down as a Service
static uint8_t TstSlwDwn = 0;
static float Vin[2]={0};
static boolean_T byPass = 0;
static float slowDwnFilt = 1.0f;

#ifdef INVERTED_FLIGHT_MODE
	static uint8_t	j2i[] = {4, 3, 2, 1, 0, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5};
#else
	static uint8_t	j2i[] = {4, 5 , 6 ,7,8,9,10,11,12,13,14,15,0,1,2,3};
#endif	

#define COMMAND_START_MEASURE 1
#define COMMAND_END_MEASURE 2
#define COMMAND_SET_PARAMS 3
#define COMMAND_SET_READBACK 4
#define TimingBudgetID 1

#define PARAM0_DISTANCE_MODE 1
#define PARAM0_TIMING_BUDGET 2
#define PARAM0_INTER_MEASUREMENT 3
#define PARAM0_SOFTWARE_VERSION 4
#define PARAM0_FREEZE_DISTANCES 200
/*
TimingBudgetID
    0 – 15mS    - NOT WORKING PROPERLY
    1- 20mS == 50hz
    2- 33mS
    3- 50mS
    4- 100mS
    5- 200mS
    6- 500mS
*/

static uint16_t result_logger[18] = {0};
//static uint16_t last_good_reading[18] = {0};

static uint8_t devAddr = 0x70;
// static uint16_t result=0;
static I2C_Dev *I2Cx;

static Opt_type Opt;
static Opt_LLAvd_type Opt_LLAvd;

static Cntrl_type Cntrl;
static float psi_cmd;
static float Vxcmd;
static float Vycmd;
static float Vxnom;
static float Vynom;

#define D_HALF 25
#define CLAMP(x, TH)  (((x) > (TH)) ? (TH) : (((x) < (-TH)) ? (-TH) : (x)))
#define IS_ZERO(x, TH)  ((((x) < (TH)) ? true : false ) && (((x) > (-TH)) ? true : false ))

static float pitch_LLAVD=0;
static float roll_LLAVD=0;
static float pitch_MLAVD=0;
static float roll_MLAVD=0;
// 15 deg = 3 m/s

//static bool isInit = false;

void send_mr18_packet(unsigned char command, uint8_t sensorId, uint16_t param0,uint16_t param1,uint16_t param2,uint16_t param3)
{
	  I2Cx = I2C1_DEV; //i2c port 1 = deck bus

		char buffer[11] = {0};
		int length=11;
		char checksum = 0;

	    buffer[0]=command;
	    buffer[1]=sensorId;
	    buffer[2] = (param0 >> 8) & 0xFF;
	    buffer[3] = param0 & 0xFF;

	    buffer[4] = (param1 >> 8) & 0xFF;
	    buffer[5] = param1 & 0xFF;

	    buffer[6] = (param2 >> 8) & 0xFF;
	    buffer[7] = param2 & 0xFF;

	    buffer[8] = (param3 >> 8) & 0xFF;
	    buffer[9] = param3 & 0xFF;
	    for(int i=0; i<10;i++)
	    {
	        checksum^=buffer[i];
//	        DEBUG_PRINT("loc %i value %i, " , i , buffer[i]);
	    }
	    buffer[10]=checksum;
//	    DEBUG_PRINT("checksum %i, " , buffer[10]);

		bool success;
		success=i2cdevWrite(I2Cx, devAddr, length,(uint8_t *)&buffer );

	    if(success) {
	    	// DEBUG_PRINT("mr18 packet %i sent\n", buffer[3]);
	    }
	    else {
	    	DEBUG_PRINT("mr18 packet send fail\n");
	    }
}

void get_readback_response()
{

	  I2Cx = I2C1_DEV; //i2c port 1 = deck bus

	char buffer_read[11] = {0};

    int length_to_read = 6;
    bool success;
    success=i2cdevRead(I2Cx, devAddr, length_to_read,(uint8_t *)&buffer_read );
    if (!success)
    {
		//DEBUG_PRINT("i2c read fail\n");

    }
    else {
    	DEBUG_PRINT("mr18 readback ");
		for(int i=0;i<length_to_read;i++)
		{
			DEBUG_PRINT("%i,",buffer_read[i]);
		}
		DEBUG_PRINT("\n");
    }
}

void readMeasure()
{

	  I2Cx = I2C1_DEV; //i2c port 1 = deck bus

	bool success;
    unsigned char buffer_read[40] = {0};
    uint8_t length_to_read = 37;
    success=i2cdevRead(I2Cx, devAddr, length_to_read,(uint8_t *)&buffer_read );
    if(!success) {
    	//DEBUG_PRINT("mr18 read fail\n");
    }
    else {
        uint8_t j=0;
        for (uint8_t sensor=0; sensor < 35; sensor= sensor + 2)
        {
        	result_logger[j]=(((uint16_t)buffer_read[sensor]) << 8) | (uint16_t)(buffer_read[sensor+1]);
        	//DEBUG_PRINT("s %i v %i,", j , result_logger[j]);
        	j++;
        }

    	//DEBUG_PRINT("\n");

    }


}


void mr18_init()
{
	vTaskDelay(M2T(2000));

	send_mr18_packet(COMMAND_SET_PARAMS,255,PARAM0_DISTANCE_MODE,2,0,0);
	vTaskDelay(M2T(2));
	get_readback_response();
	vTaskDelay(M2T(2));
	send_mr18_packet(COMMAND_SET_PARAMS,255,PARAM0_TIMING_BUDGET,20,0,0);
	vTaskDelay(M2T(2));
	get_readback_response();
	vTaskDelay(M2T(2));
	send_mr18_packet(COMMAND_SET_PARAMS,255,PARAM0_INTER_MEASUREMENT,20,0,0);
	get_readback_response();
	vTaskDelay(M2T(2));
	
	cf_feat_ext_initialize();
	init(15, 100, &Opt, &Cntrl,&Opt_LLAvd);
	Opt.nCyc_lowFreq = 2;
}


void mr18_driver()
{

	short navXY[2];
	float psi;
	float sign_vnom;
	mr18_init();
	send_mr18_packet(COMMAND_START_MEASURE,255,0,0,0,0);
	psiId = logGetVarId("stateEstimate", "yaw");
	xId = logGetVarId("stateEstimate", "x");
	yId = logGetVarId("stateEstimate", "y");
	sgnId = logGetVarId("state_machine", "vnom_sign");
	vx_spId = logGetVarId("state_machine", "vx_sp");
	vy_spId = logGetVarId("state_machine", "vy_sp");
	TstSlwDwnId = logGetVarId("state_machine", "TstSlwDwn");

	float ransac_tmp[3]={0};

	while (1)
	{
		send_mr18_packet(COMMAND_SET_READBACK,0,PARAM0_FREEZE_DISTANCES,0,0,0);
		vTaskDelay(M2T(1));
		readMeasure();
		psi = logGetFloat(psiId)*((float)(0.0174533));
		sign_vnom = logGetFloat(sgnId);
		navXY[0] = (short)(logGetFloat(xId)*100);
		navXY[1] = (short)(logGetFloat(yId)*100);
		for (uint8_t kk=0; kk < 16 ; kk++){
			if (result_logger[j2i[kk]] < 3000){
				ranges_cm[kk] = (unsigned short)(result_logger[j2i[kk]]*0.1);
			}else{
				ranges_cm[kk]=(unsigned short)(300);
			}
		}
		// DEBUG_PRINT("x,y,psi=%i, %i, %f, %i\n", navXY[0], navXY[1], (double)psi, ranges_cm[0]);
		vTaskDelay(M2T(10));

		TT = xTaskGetTickCount();

		// Slow-Down as a Service
		TstSlwDwn = logGetUint(TstSlwDwnId);
		if (TstSlwDwn)
		{
			byPass=1;
			Vin[0] = logGetFloat(vx_spId);
			Vin[1] = logGetFloat(vy_spId);
			float VinSqrt = sqrtf(Vin[0]*Vin[0]+Vin[1]*Vin[1]);
			if (!IS_ZERO(VinSqrt, 0.001f))
			{
				VinSqrt = 1/VinSqrt;
				Vin[0]*=VinSqrt;
				Vin[1]*=VinSqrt;
			}
		}
		else
		{
			byPass=0;
			Vin[0]=0;
			Vin[1]=0;
		}

		cf_feat_ext(navXY, psi, ranges_cm, &Opt, &Opt_LLAvd , TT, Vin, byPass, ransac_tmp, &validData, &slowDwn, LLAvdCmd, rej_Cmd);

		pitch_LLAVD = LLAvdCmd[0];
		roll_LLAVD = LLAvdCmd[1];
        pitch_MLAVD = rej_Cmd[0];
        roll_MLAVD  = rej_Cmd[1];

		if ((IS_ZERO(sign_vnom, 0.01f)) && (byPass == 0)){
			slowDwn = (unsigned char)1;
		}
		
		slowDwn_feat2cmd=slowDwn;
		// Logic to Allow/Cancel 90 deg turn patch
		if(allow_90deg==0)
		{
			if (slowDwn == (unsigned char)3)
			{
				slowDwn_feat2cmd=(unsigned char)2;
			}
		}

		if (validData){
			//feat2cmd(modelRANSAC, 30, &Cntrl, &psi_cmd, &Vxcmd, &Vycmd, &Vxnom, &Vynom);
			feat2cmd(ransac_tmp, D_HALF, slowDwn_feat2cmd, &Cntrl, &psi_cmd, &Vxcmd, &Vycmd, &Vxnom, &Vynom, &stopped, &slowDwnFilt);

			psi_cmd = CLAMP(psi_cmd, (float)10);
			Vxcmd = CLAMP(Vxcmd, (float)0.2);
			Vycmd = CLAMP(Vycmd, (float)0.2);
			Vxnom = CLAMP(Vxnom, (float)2);
			Vynom = CLAMP(Vynom, (float)2);

			modelRANSAC[0] = ransac_tmp[0];
			modelRANSAC[1] = ransac_tmp[1];
			modelRANSAC[2] = ransac_tmp[2];
		}



		vTaskDelay(M2T(10));

		

		//DEBUG_PRINT("heap: %d bytes\n", xPortGetFreeHeapSize());
		//DEBUG_PRINT("RANSAC[2]=%f\n", (double)modelRANSAC[2]);

		vTaskDelay(M2T(13));
	}
}


LOG_GROUP_START(mr18)
LOG_ADD(LOG_UINT16, m0, &result_logger[0])
LOG_ADD(LOG_UINT16, m1, &result_logger[1])
LOG_ADD(LOG_UINT16, m2, &result_logger[2])
LOG_ADD(LOG_UINT16, m3, &result_logger[3])
LOG_ADD(LOG_UINT16, m4, &result_logger[4])
LOG_ADD(LOG_UINT16, m5, &result_logger[5])
LOG_ADD(LOG_UINT16, m6, &result_logger[6])
LOG_ADD(LOG_UINT16, m7, &result_logger[7])
LOG_ADD(LOG_UINT16, m8, &result_logger[8])
LOG_ADD(LOG_UINT16, m9, &result_logger[9])
LOG_ADD(LOG_UINT16, m10, &result_logger[10])
LOG_ADD(LOG_UINT16, m11, &result_logger[11])
LOG_ADD(LOG_UINT16, m12, &result_logger[12])
LOG_ADD(LOG_UINT16, m13, &result_logger[13])
LOG_ADD(LOG_UINT16, m14, &result_logger[14])
LOG_ADD(LOG_UINT16, m15, &result_logger[15])
LOG_ADD(LOG_UINT16, m16, &result_logger[16])
LOG_ADD(LOG_UINT16, m17, &result_logger[17])
LOG_GROUP_STOP(mr18)

LOG_GROUP_START(ransac)
LOG_ADD(LOG_FLOAT, cpsi, &modelRANSAC[0])
LOG_ADD(LOG_FLOAT, spsi, &modelRANSAC[1])
LOG_ADD(LOG_FLOAT, rho, &modelRANSAC[2])
LOG_ADD(LOG_UINT8, valid, &validData)
LOG_ADD(LOG_UINT8, slowDwn, &slowDwn)
LOG_ADD(LOG_FLOAT, slwDwnFlt, &slowDwnFilt)
LOG_ADD(LOG_FLOAT, Vx_LLAVD, &LLAvdCmd[0])
LOG_ADD(LOG_FLOAT, Vy_LLAVD, &LLAvdCmd[1])
LOG_ADD(LOG_FLOAT, rejCmdVx, &rej_Cmd[0])
LOG_ADD(LOG_FLOAT, rejCmdVy, &rej_Cmd[1])
LOG_GROUP_STOP(ransac)

LOG_GROUP_START(feat2cmd)
LOG_ADD(LOG_FLOAT, psi_cmd, &psi_cmd)
LOG_ADD(LOG_FLOAT, Vxcmd, &Vxcmd)
LOG_ADD(LOG_FLOAT, Vycmd, &Vycmd)
LOG_ADD(LOG_FLOAT, Vxnom, &Vxnom)
LOG_ADD(LOG_FLOAT, Vynom, &Vynom)
LOG_ADD(LOG_UINT8, stopped, &stopped)
LOG_ADD(LOG_FLOAT, SlwDwnFct, &slowDwnFilt)
LOG_GROUP_STOP(feat2cmd)

LOG_GROUP_START(LLAVD)
LOG_ADD(LOG_FLOAT, pitch_LLAVD, &pitch_LLAVD)
LOG_ADD(LOG_FLOAT, roll_LLAVD, &roll_LLAVD)
LOG_ADD(LOG_FLOAT, pitch_MLAVD, &pitch_MLAVD)
LOG_ADD(LOG_FLOAT, roll_MLAVD, &roll_MLAVD)
LOG_GROUP_STOP(LLAVD)

PARAM_GROUP_START(ransac)
PARAM_ADD(PARAM_FLOAT, measHist, &Opt.measHistMinDist_p2)
PARAM_ADD(PARAM_FLOAT, posHist, &Opt.posHistMinStep_p2)
PARAM_ADD(PARAM_UINT16, xyPosLen, &Opt.xyPosLen)
PARAM_ADD(PARAM_UINT16, xyMeasLen, &Opt.xyMeasLen)
PARAM_ADD(PARAM_UINT8, nCyc_low, &Opt.nCyc_lowFreq)
PARAM_ADD(PARAM_FLOAT, SF_frd, &Opt.SF_frwrd)
PARAM_ADD(PARAM_FLOAT, exp_LP, &Opt.exp_LP)
PARAM_ADD(PARAM_UINT16, PF_rmax, &Opt.PF_radius_max)
PARAM_ADD(PARAM_UINT16, PF_rmin, &Opt.PF_radius_min)
PARAM_ADD(PARAM_UINT16, LF_dmax, &Opt.LF_dist_max)
PARAM_ADD(PARAM_UINT16, stp_d, &Opt.stopped_dist)
PARAM_ADD(PARAM_FLOAT, vel_th_cmps, &Opt_LLAvd.vel_noise_th_cmps)
PARAM_ADD(PARAM_UINT16, rad_max_cm, &Opt_LLAvd.TOF_radius_max_cm)
PARAM_ADD(PARAM_FLOAT, avoid_clamp, &Opt_LLAvd.rej_avoidance_clamp)
PARAM_ADD(PARAM_FLOAT, avoid_k, &Opt_LLAvd.rej_k_avoid)
PARAM_ADD(PARAM_UINT16, avoid_radius, &Opt_LLAvd.rej_radius_mm)
PARAM_GROUP_STOP(ransac)


PARAM_GROUP_START(feat2cmd)
PARAM_ADD(PARAM_FLOAT, vnom, &Cntrl.Vnom)
PARAM_ADD(PARAM_FLOAT, kdx, &Cntrl.Kdx)
PARAM_ADD(PARAM_FLOAT, kdy, &Cntrl.Kdy)
PARAM_ADD(PARAM_FLOAT, kpsi, &Cntrl.Kpsi)
PARAM_ADD(PARAM_FLOAT, Vn_min, &Cntrl.Vnom_min_rtio)
PARAM_ADD(PARAM_FLOAT, Vn_slw, &Cntrl.Vnom_slw_filt)
PARAM_ADD(PARAM_FLOAT, Vn_acc, &Cntrl.Vnom_acc_filt)
PARAM_ADD(PARAM_UINT8, sf_dAcc, &Cntrl.sf_deaccel_cnt)
PARAM_ADD(PARAM_UINT8, stpCnt, &Cntrl.stopped_consec_cnt)
PARAM_GROUP_STOP(feat2cmd)

PARAM_GROUP_START(LLAVD)
PARAM_ADD(PARAM_FLOAT, kFF_Ang, &Opt_LLAvd.kFF_ang)
PARAM_ADD(PARAM_FLOAT, llDeadB, &Opt_LLAvd.llav_deadB)
PARAM_ADD(PARAM_FLOAT, mps2Deg, &Opt_LLAvd.mps2Deg)
PARAM_GROUP_STOP(LLAVD)
