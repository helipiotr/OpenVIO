#include "system.h"
#include "stm32f4xx.h"

void RCC_Conf(){
	int i=0;
	//uint8_t clockSource;
	//uint8_t readValue;
	int wait_period = 10000;
	ErrorStatus HSE_status;

	RCC_DeInit();
	RCC_HSEConfig(RCC_HSE_ON); //turning on HSE clock
	//clockSource = RCC_GetSYSCLKSource();

	// waiting for clock being on
	while( i < wait_period){
		++i;
		HSE_status = RCC_WaitForHSEStartUp();
		if (HSE_status == SUCCESS){
			i=wait_period;
		}
	}

	//clock failure
	if(HSE_status == ERROR){
		while(1){};
	}
	// 100MHz as the system frequency
	//uint32_t PLL_clock_freq = 100000000;
	RCC_PLLConfig(RCC_PLLSource_HSE , 8 , 400 , 4 , 7);

	RCC_PLLCmd(ENABLE);

	//waiting for PLL to turn on
	while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY)==RESET){};

	//choosing PLL as a system clock
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

	//choosing HSE as a system clock
	//RCC_SYSCLKConfig(RCC_SYSCLKSource_HSE);

}


