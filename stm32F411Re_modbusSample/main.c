#include "main.h"

int main(void){
	RCC_init();
	usart2_init(115200);
	DMA_init();
	__enable_irq();		//global interrupts en
	
	while(1)
	{
		
	}
}

void RCC_init(void){
	/*clock config
	HSI RC == 16Mhz by default.
	System clock(SYSCLK) = HSI / PLLM * PLLN / PLLP = 16 / 16 * 192 / 2 = 96Mhz. If PLLCLK is source for SYSCLK*/
	RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLSRC;						//HSI source for PLL
	RCC->PLLCFGR |= (16 << RCC_PLLCFGR_PLLM_Pos);			//PLLM (PLL source Mux divider == 16)
	RCC->PLLCFGR |= (192 << RCC_PLLCFGR_PLLN_Pos);			//PLLN (Main PLL mult. == 192)
	RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLP;							//PLLP (Main PLL divider == 2)
	
	/*DMA at AHB bus == 96Mhz; APB1 peripheral max clock is 50Mhz, so current is 48Mhz(96 / 2 = 48Mhz)*/
	RCC->CFGR &= ~RCC_CFGR_HPRE;									//AHB1PERIPH_BASE without prescaller
	RCC->CFGR |= RCC_CFGR_PPRE1_2;								//APB1 prescaller = 2
	
	/*enable PLL and choose it as SYSCLK source, disable HSI as system clock to prevent dispute*/
	FLASH->ACR |= FLASH_ACR_LATENCY_3WS;						//flash access period = 3 
	RCC->CR |= RCC_CR_PLLON;										//PLL en
	while(!(RCC->CR & RCC_CR_PLLON));							//whait until PLL en
	RCC->CFGR |= RCC_CFGR_SW_1;									//PLL source
	while(!(RCC->CFGR & RCC_CFGR_SWS_1));						//wait until PLL source on
	RCC->CR &= ~RCC_CR_HSION;										//HSI dis
	//while(RCC->CR & RCC_CR_HSIRDY);								//wait 6 cycles until HSI goes low(could be problem with this, shoul check)
	/*Resume: all peripheral clocks == 96Mhz, APB1 = 48Mhz*/
	
	//PORT A(usart2: A2(TX), A3(RX))
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; 						//port A en

	//USART
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;						//usart2 clock en
	
	//DMA
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;							//DMA1 en
}

void DMA_init(void){
	//DMA1 USART2_RX
	DMA1_Stream5->PAR = (uint32_t)&USART2->DR;				//peref addr
	DMA1_Stream5->M0AR = (uint32_t)&modbusReceiveBuf[0];	//mem addr
	DMA1_Stream5->NDTR = MaxModbusBufLen;						//MaxAnalogRegVal len buffer to receive
	
	DMA1_Stream5->CR |= DMA_SxCR_CHSEL_2;			//channel 4 selected
	DMA1_Stream5->CR &= ~DMA_SxCR_DIR;				//perif to mem direction
	DMA1_Stream5->CR &= ~DMA_SxCR_MSIZE;			//Memory data size = 8 bit
	DMA1_Stream5->CR &= ~DMA_SxCR_PSIZE;			//Peripheral data size = 8 bit
	DMA1_Stream5->CR |= DMA_SxCR_MINC;				//memory incremental mode on
	
	/*
	DMA1->HIFCR |= DMA_HIFCR_CTCIF5;					//clearing TCIF5 flag
	DMA1_Stream5->CR |= DMA_SxCR_TCIE;				//transfer complete interrupt enable
	NVIC_EnableIRQ(DMA1_Stream5_IRQn);				//interrupt en
	*/
	
	DMA1_Stream5->CR |= DMA_SxCR_EN;					//Stream en
	
	//DMA USART2_TX
	DMA1_Stream6->PAR = (uint32_t)&USART2->DR;
	DMA1_Stream6->M0AR = (uint32_t)&modbusTransferBuf[0];
	
	DMA1_Stream6->CR |= DMA_SxCR_CHSEL_2;			//channel 4 selected
	DMA1_Stream6->CR |= DMA_SxCR_DIR_0;				//memory to peripheral direction
	DMA1_Stream6->CR &= ~DMA_SxCR_MSIZE;			//Memory data size = 8 bit
	DMA1_Stream6->CR &= ~DMA_SxCR_PSIZE;			//Peripheral data size = 8 bit
	DMA1_Stream6->CR |= DMA_SxCR_MINC;				//memory incremental mode on
	
	DMA1->HIFCR |= DMA_HIFCR_CTCIF6;					//clearing TCIF6 flag
	DMA1_Stream6->CR |= DMA_SxCR_TCIE;				//transfer complete interrupt enable
	NVIC_EnableIRQ(DMA1_Stream6_IRQn);				//interrupt en
}
