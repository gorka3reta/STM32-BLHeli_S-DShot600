/*
 *    11 bits throttle | 1 bit telemetry | 4 bits crc -> 16 bit package
 *
 *    1'67 us per bit
 *
 *    625 ns HIGH -> '0' bit
 *
 *    1250 ns HIGH -> '1' bit
 *
 *    read readme to see the arming sequence
 */

#ifndef __DSHOT600_H__
#define __DSHOT600_H__

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#define BIT_LENGTH 20 //It depends on APB's clock and the prescaler
#define BIT_ONE    14
#define BIT_ZERO    7

#define BYTE_LENGTH 18

#define ESC_POWER_UP 3000
#define TROTTLE_MAX 500
#define TROTTLE_MIN 48

typedef struct{//Strings containing timer's pulse values (7 or 14)

	uint32_t motor1[BYTE_LENGTH];
	uint32_t motor2[BYTE_LENGTH];
	uint32_t motor3[BYTE_LENGTH];
	uint32_t motor4[BYTE_LENGTH];
}MOTOR_t;

typedef enum{
  
  DSHOT600_OK,
  DSHOT600_FAIL,
}RESULT_t;

uint16_t value = TROTTLE_MIN, value1 = TROTTLE_MIN, value2 = TROTTLE_MIN, value3 = TROTTLE_MIN, value4 = TROTTLE_MIN;				
uint16_t ctr = 0;						// arming sequence counter
uint8_t trottle_down=0;			// dunnot change
uint8_t armed = 0;          //when armed turns to 1 means that the arming sequence is over

RESULT_t init_dshot (TIM_HandleTypeDef *htim, uint32_t Channel);
RESULT_t send_dshot (TIM_HandleTypeDef *htim, uint32_t Channel, uint16_t throttle);
