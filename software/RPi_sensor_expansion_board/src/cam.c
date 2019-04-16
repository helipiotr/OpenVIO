/*
 * cam.c
 *
 *  Created on: 08.02.2019
 *      Author: helipiotr
 */

#include "cam.h"


void cam_setup(void){
	//Initiating GPIOs & SPI
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); //GPIOB
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE); //SYSCFG
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_EXTIT, ENABLE); //EXTIIT

	// Configure an interrupt on PB6
	GPIO_InitTypeDef portB;

	portB.GPIO_Pin = 6;
	portB.GPIO_Mode = GPIO_Mode_IN;
	portB.GPIO_OType = GPIO_OType_PP;
	portB.GPIO_PuPd = GPIO_PuPd_DOWN;
	portB.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOB, &portB);


}
