/******************************************************************************
 * This is the register-level code for interfacing the on-board LIS302DL
 * accelerometer using the SPI Protocol.
 *
 * Configurations are as follows:
 * CS        - PE3
 * SCK       - PA5
 * MOSI      - PA7
 * MISO      - PA6
 *

 *****************************************************************************/
#include "text.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <string.h>

extern SPI_HandleTypeDef hspi1;

extern TIM_HandleTypeDef htim6;

extern UART_HandleTypeDef huart2;

//User-defined variables
uint16_t x, y, z;
int16_t x_final = 0, y_final = 0, z_final;
uint16_t rxd, rxdf;
int16_t mapvalue = 0, xangle = 0, yangle = 0;
uint8_t rxdata[3] = { '\0' };
char txdata[12] = { 0 };
uint8_t txleft[] = "closed\n";

void GPIO_Init() {
	// Enable GPIOA clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

	// Configuring PA5, PA6, PA7 in alternate function mode
	GPIOA->MODER |= (GPIO_MODER_MODER5_1 | GPIO_MODER_MODER6_1
			| GPIO_MODER_MODER7_1);

	// Select AF5 for SPI on PA5, PA6, PA7
	GPIOA->AFR[0] |= (GPIO_AFRL_AFSEL5_2 | GPIO_AFRL_AFSEL5_0
			| GPIO_AFRL_AFSEL6_2 | GPIO_AFRL_AFSEL6_0 | GPIO_AFRL_AFSEL7_2
			| GPIO_AFRL_AFSEL7_0);

	// Enable GPIOE clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;

	// Since PE3 is CS, it needs to be configured in Output Mode
	GPIOE->MODER |= GPIO_MODER_MODER3_0;

	GPIOA->OSPEEDR |= ( GPIO_OSPEEDER_OSPEEDR5_0 | GPIO_OSPEEDER_OSPEEDR6_0 |GPIO_OSPEEDER_OSPEEDR7_0);

	GPIOA->PUPDR |= (GPIO_PUPDR_PUPD5_1 | GPIO_PUPDR_PUPD6_1| GPIO_PUPDR_PUPD7_1);

	// Enable clock for GPIOD and Configure PD12 in output mode
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
	GPIOD->MODER |= (GPIO_MODER_MODER12_0 | GPIO_MODER_MODER13_0| GPIO_MODER_MODER14_0 | GPIO_MODER_MODER15_0);
}

void SPI_Init() {
	// Enable SPI clock
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

	// Select the Master Configuration
	SPI1->CR1 |= SPI_CR1_MSTR;

	SPI1->CR1 &= ~SPI_CR1_BIDIMODE;

	SPI1->CR1 &= ~SPI_CR1_RXONLY;

	// Set the Data Frame Format (DFF) to '0' or 8-bit.
	SPI1->CR1 &= ~SPI_CR1_DFF;

	// SSI and SSM bits in the SP1->CR1 register need to be set
	// to '1'
	SPI1->CR1 |= (SPI_CR1_SSI | SPI_CR1_SSM);

	// Setting Baud Rate
	SPI1->CR1 &= ~SPI_CR1_BR;

	// Set the transmission to MSB First Mode
	SPI1->CR1 &= ~SPI_CR1_LSBFIRST;

	// Configure CPOL and CPHASE to '0' and '0', respectively.
	// i.e. Clock is at '0' when idle, and data capture is done
	// on the first clock transition which is the rising edge.
	SPI1->CR1 &= ~SPI_CR1_CPHA;
	SPI1->CR1 &= ~SPI_CR1_CPOL;

	// Enable CRC
	SPI1->CR1 |= SPI_CR1_CRCEN;

	// Enable SPI
	SPI1->CR1 |= SPI_CR1_SPE;

	// Selecting Motorola Format
	SPI1->CR2 = 0x0000;
}

uint16_t SPI_Transmit(uint8_t data) {
	//  Wait until the TX buffer is empty, i.e. data is transmitted
	while (!((SPI1->SR) & SPI_SR_TXE)) {
	}
	// Load the data into the data register
	SPI1->DR = data;

	while (!(SPI1->SR & SPI_SR_RXNE)) {
	}
	// If reception is intended, read the value from the data register
	rxd = SPI1->DR;

	return rxd;
}

uint16_t SPI_Receive(uint8_t addr) {
	GPIOE->BSRR |= GPIO_BSRR_BR3;
	addr |= 0x80;
	SPI_Transmit(addr);
	rxdf = SPI_Transmit(0);
	GPIOE->BSRR |= GPIO_BSRR_BS3;
	return rxdf;
}

void LIS_Write(uint8_t addr, uint8_t data) {
	// Selecting the LIS accelerometer
	GPIOE->BSRR |= GPIO_BSRR_BR3;

	// Send the Register Address
	SPI_Transmit(addr);

	// Send the data to be written
	SPI_Transmit(data);

	// De-select the accelerometer
	GPIOE->BSRR |= GPIO_BSRR_BS3;
}

void LIS_Init() {
	// Powering on the accelerometer and Enabling the x,y and z axis for acceleration capture
	LIS_Write(CTRL_REG1, 0x47);
}

void LIS_Read() {
	// Reading the data for x-axis
	x = SPI_Receive(OUT_X);

	// Reading the data for y-axis
	y = SPI_Receive(OUT_Y);

	// Reading the data for z-axis
	z = SPI_Receive(OUT_Z);
}

int16_t Convert_To_Val(uint16_t val) {
	if ((val & 0x80) == 0x80) {
		val = ~val;
		val += 1;
		val &= 0x00FF;
		val = (val * 2300) / 127;
		return (-1 * val);
	} else
		return ((val * 2300) / 127);
}

int16_t map(int16_t value, int16_t min, int16_t max, int16_t valmin,
		int16_t valmax) {
	mapvalue = value * (valmax - valmin) / (max - min);
	return mapvalue;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	HAL_UART_Receive_IT(&huart2, rxdata, sizeof(rxdata));

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	if (rxdata[0] == 'o' && rxdata[1] == 'n' && rxdata[2] == 'n') {

		HAL_UART_Transmit(&huart2, (uint8_t*) txdata, sizeof(txdata), TIMEOUT);
	} else if (rxdata[0] == 'o' && rxdata[1] == 'f' && rxdata[2] == 'f') {
		HAL_UART_Transmit(&huart2, (uint8_t*) txleft, sizeof(txleft), TIMEOUT);
	} else {
		HAL_UART_Transmit(&huart2, (uint8_t*) txdata, sizeof(txdata), TIMEOUT);
	}

}

