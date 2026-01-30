/*
 * acc_driver.c
 *
 *  Created on: 30 Jan 2022
 *      Author: zhuangzhuangdai
 */

#include "acc_driver.h"

const uint8_t who_am_i = 0x0F;
const uint8_t who_am_i_val = 0x3F;
const uint8_t ctrl_reg = 0x20;
const uint8_t ctrl_reg_val = 0x87;
const uint8_t x_reg = 0x28;
const uint8_t y_reg = 0x2A;
const uint8_t z_reg = 0x2C;


uint8_t AccInit(){
	uint8_t tmp_buf;
	SPIAcc_Init();
	SPIAcc_Get(who_am_i, &tmp_buf, 1);
	//printf("0x%02x\n", tmp_buf);

	SPIAcc_Send(ctrl_reg, &ctrl_reg_val, 1);
	if (tmp_buf == who_am_i_val){
		return 1;
	}
	else{
		return 0;
	}
}

void AccRead(acc3_t * reading){
	uint8_t buffer[6];
	SPIAcc_Get(x_reg, buffer, 2);
	SPIAcc_Get(y_reg, &buffer[2], 2);
	SPIAcc_Get(z_reg, &buffer[4], 2);

	reading->x = (buffer[1]<<8) + buffer[0];
	reading->y = (buffer[3]<<8) + buffer[2];
	reading->z = (buffer[5]<<8) + buffer[4];
}
