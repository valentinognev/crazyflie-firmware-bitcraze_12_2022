//ledtask.h defining functions for ledtask.c run
//


#ifndef __LEDTASK_H__
#define __LEDTASK_H__



#include <stdint.h>
#include <stdbool.h>
#include <led.h>

/**
 * @brief Constantly update frequency in the selected led to a selected value to be Evaluated
 *        LedSelect = 0,1,2,3,4 depending on the specific led, elaborated in led.h
 *        
 *
 * 
 */
void LedTask();

/**
 * @brief LedTask init
 *        
 *
 * 
 */
void LedTask_init();


#endif