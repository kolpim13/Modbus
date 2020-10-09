#include "usart.h"

//USART2
void usart2_init(int baudRate){
	//GPIO
	GPIOA->MODER |= GPIO_MODER_MODE2_1;						//alternate function mode PA2(usart2_TX)
	GPIOA->MODER |= GPIO_MODER_MODE3_1;						//alternate function mode PA3(usart2_RX)
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR2;				//very high port speed
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR3;				//very high port speed

	GPIOA->AFR[0] |= GPIO_AFRL_AFRL2;						//alternate function 7 - USART2_TX
	GPIOA->AFR[0] &= ~GPIO_AFRL_AFRL2_3;
	GPIOA->AFR[0] |= GPIO_AFRL_AFRL3;						//alternate function 7 - USART2_RX
	GPIOA->AFR[0] &= ~GPIO_AFRL_AFRL3_3;
	
	//USART2
	USART2->CR1 |= USART_CR1_UE;								//en
	USART2->CR1 &= ~USART_CR1_M;								//8 data len
	USART2->CR2 &= ~USART_CR2_STOP;							//1 stop bit

	USART2->CR3 |= USART_CR3_DMAT;							//DMA transmitter en
	USART2->CR3 |= USART_CR3_DMAR;							//DMA receiver en
	
	/*baud rate. BRR = peripheral clock / baudRate(rounded) = 48Mhz / x.*/
	switch(baudRate){
		case 9600:{
			USART2->BRR = 5000;
			break;
		}		
		case 115200:{
			USART2->BRR = 417;
			break;
		}
		default:{
			USART2->BRR = 417;
			break;
		}
	}
	
	USART2->CR1 |= USART_CR1_TE;								//transmission en
	USART2->CR1 |= USART_CR1_RE;								//receiv en
	USART2->CR1 |= USART_CR1_IDLEIE;							//idle line occur interrupt
	USART2->CR3 |= USART_CR3_EIE;								//ERROR interrupt en
	USART2->CR1 &= ~USART_CR1_OVER8;							//over8 off
	NVIC_EnableIRQ(USART2_IRQn);								//usart2 interrupts en
	
	modbusTransferBuf[0] = currentModbusAddress;			//this byte will always be the sam
}

//Modbus RTU
/*poly 0x8005 CRC table*/
static const uint16_t Crc16Table[256] = {
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};
/*buffers & auxiliary vars*/
volatile int8_t modbusReceiveBuf[MaxModbusBufLen];						
volatile uint8_t modbusReceiveBufLen;
volatile int8_t modbusTransferBuf[MaxModbusBufLen];
volatile uint8_t modbusTransferBufLen;

const uint8_t currentModbusAddress = 255;
void volatile (*modbusFunction)(void) = NULL;

/*registers*/
volatile HoldingRegGroup holdingRegGroup_1 = { 1, 2, 3, 4, 5 };
volatile HoldingRegGroup holdingRegGroup_2 = { -1, -2, -3, -4, -5 };

/*functions*/
inline void modbusReadDMA(void){
	//DMA1_Stream5->CR &= ~DMA_SxCR_EN;					//is needed?
	DMA1->HIFCR |= DMA_HIFCR_CTCIF5;						//RESET complete transfer bit to start new transfer
	DMA1_Stream5->CR |= DMA_SxCR_EN;
}
inline void modbusWriteDMA(void){
	//DMA1_Stream6->CR &= ~DMA_SxCR_EN;					//is needed?
	DMA1->HIFCR |= DMA_HIFCR_CTCIF6;						//RESET complete transfer bit to start new transfer
	DMA1_Stream6->NDTR = modbusTransferBufLen;		
	DMA1_Stream6->CR |= DMA_SxCR_EN;
}
int16_t CRC_calc(uint8_t *buffer, uint8_t bufferLen){
	uint16_t crc = 0xFFFF;
	while( bufferLen--){
		crc = (crc >> 8) ^ Crc16Table[(crc & 0xFF) ^ *buffer++];
	}
	return crc;
}

uint8_t validateQuery(void){
	return TRUE;
}
uint8_t chooseModbusFunction(void){
	switch(modbusReceiveBuf[1]){
		case READ_HOLDINGS:{
			modbusFunction = modbusReadHoldings;
			return TRUE;
		}
		case WRITE_HOLDING:{
			modbusFunction = modbusWriteHolding;
			return TRUE;
		}
		default:{
			modbusFunction = NULL;
			return FALSE;
		}
	}
}
uint8_t *chooseModbusReg(uint16_t regNumb, int16_t *reg, uint8_t *regType){
	switch(regNumb){
		case GROUP4_REG001:{*reg = holdingRegGroup_1.reg1; *regType = HOLDING_REG; return TRUE;}
		case GROUP4_REG002:{*reg = holdingRegGroup_1.reg2; *regType = HOLDING_REG; return TRUE;}
		case GROUP4_REG003:{*reg = holdingRegGroup_1.reg3; *regType = HOLDING_REG; return TRUE;}
		case GROUP4_REG004:{*reg = holdingRegGroup_1.reg4; *regType = HOLDING_REG; return TRUE;}
		case GROUP4_REG005:{*reg = holdingRegGroup_1.reg5; *regType = HOLDING_REG; return TRUE;}
		
		case GROUP4_REG101:{*reg = holdingRegGroup_2.reg1; *regType = HOLDING_REG; return TRUE;}
		case GROUP4_REG102:{*reg = holdingRegGroup_2.reg2; *regType = HOLDING_REG; return TRUE;}
		case GROUP4_REG103:{*reg = holdingRegGroup_2.reg3; *regType = HOLDING_REG; return TRUE;}
		case GROUP4_REG104:{*reg = holdingRegGroup_2.reg4; *regType = HOLDING_REG; return TRUE;}
		case GROUP4_REG105:{*reg = holdingRegGroup_2.reg5; *regType = HOLDING_REG; return TRUE;}
		default: return FALSE;
	}
}
uint8_t getRegType(uint16_t regNumb){
	if(regNumb >= GROUP4_REG001 && regNumb <= GROUP4_REG005)
		return HOLDING_REG;
	if(regNumb >= GROUP4_REG101 && regNumb <= GROUP4_REG105)
		return HOLDING_REG;
	
	return UNKNOWN_REG;
}

void modbusReadHoldings(void){
	int16_t address = (modbusReceiveBuf[2] << 8) | (modbusReceiveBuf[3] & 0xFF);
	int16_t amount = (modbusReceiveBuf[4] << 8) | (modbusReceiveBuf[5] & 0xFF);
	int8_t dataLen = (int8_t)amount * 2;

	int16_t temp;
	uint8_t regType;
	for(int i=0, j=0; i<amount; i++){
		if(chooseModbusReg(address + i, &temp, &regType) == FALSE){
			//provide error 2(wrong data address)
			modbusTransferBuf[1] = modbusReceiveBuf[1] + 0x80;
			modbusTransferBuf[2] = 2;
			modbusTransferBufLen = 3;
			modbusWriteDMA();
		}
		else{
			modbusTransferBuf[3 + j++] = temp >> 8;
			modbusTransferBuf[3 + j++] = (int8_t)temp;
		}
	}
	
	modbusTransferBuf[1] = modbusReceiveBuf[1];
	modbusTransferBuf[2] = dataLen;
	modbusTransferBufLen = dataLen+3+2;
	
	int16_t crc = CRC_calc(modbusTransferBuf, modbusTransferBufLen-2);
	modbusTransferBuf[modbusTransferBufLen - 2] = (int8_t)(crc);
	modbusTransferBuf[modbusTransferBufLen - 1] = (crc >> 8);
}	
void modbusWriteHolding(void){
	int a = 5+3;
}

//INTERRUPTS
/*usart2 handle*/
void USART2_IRQHandler(void){
	
	DMA1_Stream5->CR &= ~DMA_SxCR_EN;
	
	/*if error occur(overrun, framing, noise)*/
	if(USART2->SR & USART_SR_ORE){					//if ovverrun error occured
		(void)USART2->DR;									//RESET ORE bit
		
		/*reset DMA_usart2_rx settings*/
		//DMA1_Stream5->CR &= ~DMA_SxCR_EN;
		DMA1->HIFCR |= DMA_HIFCR_CTCIF5;
		DMA1_Stream5->M0AR = (uint32_t)&modbusTransferBuf[0];
		DMA1_Stream5->NDTR = MaxModbusBufLen;
		DMA1_Stream5->CR |= DMA_SxCR_EN;	
	}
	
	/*if IDLE interrupt occur*/
	if(USART2->SR & USART_SR_IDLE){					//to start reset IDLE bit	
		(void)USART2->DR;									//RESET IDLE bit
		
		/*calculate parsel length and disable DMA*/
		//DMA1_Stream5->CR &= ~DMA_SxCR_EN;
		modbusReceiveBufLen = MaxModbusBufLen - DMA1_Stream5->NDTR;
		
		if(validateQuery() == TRUE){
			/*if such function is exist, so do it*/
			if(chooseModbusFunction() == TRUE){
				modbusFunction();
				modbusWriteDMA();
			}
			/*if not, send an report*/
			else{
				modbusTransferBuf[1] = modbusReceiveBuf[1] + 0x80;
				modbusTransferBuf[2] = 1;
				modbusTransferBufLen = 3;
				modbusWriteDMA();
			}
		}
	}
}
/*DMA1_USART2_RX handle*/
/*
void DMA1_Srteam5_IRQHandler(void){
	if(DMA1->HISR & DMA_HISR_TCIF5){
		DMA1->HIFCR |= DMA_HIFCR_CTCIF5;	
	}
	modbusReadDMA();
}
*/
/*DMA1_USART2_TX handle*/
void DMA1_Stream6_IRQHandler(void){
	if(DMA1->HISR & DMA_HISR_TCIF6){
		DMA1->HIFCR |= DMA_HIFCR_CTCIF6;				//clear interrupt flag
		
		modbusReadDMA();
	}
}

