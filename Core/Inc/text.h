/*
 * text.h
 *
 *  Created on: Dec 16, 2024
 *      Author: TALHA ERDEM
 */

#ifndef INC_TEXT_H_
#define INC_TEXT_H_

#include <stdint.h>



//Register Definitions for LIS302DL
#define LIS302DL_ADDR     (0x3B)
#define WHO_AM_I          (0x0F)
#define CTRL_REG1         (0x20)
#define CTRL_REG2         (0x21)
#define CTRL_REG3         (0x22)
#define HP_FILTER_RESET   (0x23)
#define STATUS_REG        (0x27)
#define OUT_X             (0x29)
#define OUT_Y             (0x2B)
#define OUT_Z             (0x2D)

// Calibration constants
#define X_OFFSET 18
#define Y_OFFSET 18
#define THRESH_LOW -120
#define THRESH_HIGH 120

#define TIMEOUT 100

//User-defined Function Declarations

void GPIO_Init(void);
void SPI_Init(void);
uint16_t SPI_Transmit(uint8_t data);
uint16_t SPI_Receive(uint8_t addr);
void LIS_Init(void);
void LIS_Write(uint8_t addr, uint8_t data);
void LIS_Read(void);
int16_t Convert_To_Val(uint16_t val);

int16_t map(int16_t value,int16_t min,int16_t max,int16_t valmin,int16_t valmax);



#endif /* INC_TEXT_H_ */
