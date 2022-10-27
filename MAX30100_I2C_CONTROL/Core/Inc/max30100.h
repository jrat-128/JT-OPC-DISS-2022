/*
 * max30100.h
 *
 *  Created on: Aug 1, 2022
 *      Author: jrat2
 */

#ifndef INC_MAX30100_H_
#define INC_MAX30100_H_

#include "main.h"
#include <string.h>

// The following values are required to communicate with various registers of the MAX30100 oximeter
#define MAX_ADDR				0x57<<1

#define REG_SpO2_WRPT			0x02
#define REG_SpO2_OVFC			0x03
#define REG_SpO2_RDPT			0x04
#define REG_SpO2_DATA			0x05

#define REG_INTERRUPT_CONFIG	0x40
#define REG_INTERRUPTS			0x00
#define REG_CONFIG_MODE			0x06
#define REG_CONFIG_SpO2			0x07
#define REG_CONFIG_LEDS			0x09

// A structure to extract sample values
struct spo2vals {
    float ir_val, vr_val;
};

void MAX30100_WRITE(I2C_HandleTypeDef h, uint16_t target, uint8_t pData); // Write to a register
uint8_t MAX30100_READ(I2C_HandleTypeDef h, uint16_t target); // Read a register
void MAX30100_DATA_REQUEST(I2C_HandleTypeDef h); // Order the acquisition of a sample
HAL_StatusTypeDef MAX30100_TRANSMIT(I2C_HandleTypeDef h, uint16_t target); // Communicate with the oximeter, used to request samples
struct spo2vals MAX30100_SPO2_EXTRACT(I2C_HandleTypeDef h); // Transfer a full set of 4 samples
float MAX30100_SPO2_AVRG(float array[]); // Calculate the average off an array of values

#endif /* INC_MAX30100_H_ */
