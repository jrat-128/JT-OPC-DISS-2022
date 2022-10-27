/*
 * max30100.c
 *
 *  Created on: Aug 1, 2022
 *      Author: jrat2
 */

#include "max30100.h"
#include "main.h"

HAL_StatusTypeDef ret;

void MAX30100_WRITE(I2C_HandleTypeDef h, uint16_t target, uint8_t pData)
{
	HAL_I2C_Mem_Write(&h, MAX_ADDR, target, 1, &pData, 1, HAL_MAX_DELAY);
}

uint8_t MAX30100_READ(I2C_HandleTypeDef h, uint16_t target)
{
	uint8_t buf[32];
	buf[0] = target;
	ret = HAL_I2C_Master_Receive(&h, MAX_ADDR, buf, 2, HAL_MAX_DELAY);
	if ( ret != HAL_OK ) {
		strcpy((char*)buf, "ErrorRqst\r\n");
	}
	return buf[0];
}

void MAX30100_DATA_REQUEST(I2C_HandleTypeDef h)
{
	uint8_t pData = 0x00;
	HAL_I2C_Mem_Write(&h, MAX_ADDR, REG_SpO2_WRPT, 1, &pData, 1, HAL_MAX_DELAY);
	pData = 0x00;
	HAL_I2C_Mem_Write(&h, MAX_ADDR, REG_SpO2_OVFC, 1, &pData, 1, HAL_MAX_DELAY);
	pData = 0x00;
	HAL_I2C_Mem_Write(&h, MAX_ADDR, REG_SpO2_RDPT, 1, &pData, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef MAX30100_TRANSMIT(I2C_HandleTypeDef h, uint16_t target)
{
	uint8_t buf[32];
	buf[0] = target;
	HAL_StatusTypeDef ret =	HAL_I2C_Master_Transmit(&h, MAX_ADDR, buf, 1, HAL_MAX_DELAY);
	return ret;
}



struct spo2vals MAX30100_SPO2_EXTRACT(I2C_HandleTypeDef h)
{
	uint8_t pDataI[64];
	float ir_s1, ir_s2, ir_s3, ir_s4;
	float vr_s1, vr_s2, vr_s3, vr_s4;
	struct spo2vals to_return;

	HAL_I2C_Mem_Read(&h, MAX_ADDR, REG_SpO2_DATA, 1, pDataI, 16, HAL_MAX_DELAY);

	ir_s1 = (pDataI[0] << 8) + pDataI[1];
	vr_s1 = (pDataI[2] << 8) + pDataI[3];

	ir_s2 = (pDataI[4] << 8) + pDataI[5];
	vr_s2 = (pDataI[6] << 8) + pDataI[7];

	ir_s3 = (pDataI[8] << 8) + pDataI[9];
	vr_s3 = (pDataI[10] << 8) + pDataI[11];

	ir_s4 = (pDataI[12] << 8) + pDataI[13];
	vr_s4 = (pDataI[14] << 8) + pDataI[15];

	to_return.ir_val = (float)((ir_s1 + ir_s2 + ir_s3 + ir_s4) / 4);
	to_return.vr_val = (float)((vr_s1 + vr_s2 + vr_s3 + vr_s4) / 4);

	return to_return;
}

float MAX30100_SPO2_AVRG(float array[])
{
	float med = 0;
	for(int i = 0; i < DEPTH; i++)
	{
		med = med + array[i];
	}
	med = med/DEPTH;
	return med;
}

