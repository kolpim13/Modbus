#ifndef _USART_H
#define _USART_H

#include "main.h"

//USART2
void usart2_init(int baudRate);

//Modbus RTU
#define MaxModbusBufLen			255		//max buffer size(in bytes) to receive and transmit by modbus
#define MaxModbusRegCount		125		//max registers amount to transfer their values((255 - 5(addr[1], func[1], len[1], CRC[2])) / 2(because regs consist of 2 bytes))
#define MinHoldingRegsValue	-5000		//min holding registers value
#define MaxHoldingRegsValue	5000		//max holding registers value

/*vars*/
extern volatile int8_t modbusReceiveBuf[MaxModbusBufLen];						
extern volatile uint8_t modbusReceiveBufLen;
extern volatile int8_t modbusTransferBuf[MaxModbusBufLen];
extern volatile uint8_t modbusTransferBufLen;

extern const uint8_t currentModbusAddress;
extern void volatile (*modbusFunction)(void);

/*functions*/
extern inline void modbusReadDMA(void);
extern inline void modbusWriteDMA(void);
int16_t CRC_calc(uint8_t *buffer, uint8_t bufferLen);

uint8_t validateQuery(void);
uint8_t chooseModbusFunction(void);
uint8_t *chooseModbusReg(uint16_t regNumb, int16_t *reg, uint8_t *regType);
uint8_t getRegType(uint16_t regNumb);

void modbusReadHoldings(void);				//0x03
void modbusWriteHolding(void);				//0x06

/*current modbus register`s numbers*/
enum ModbusRegisters{
	//GROUPx_REGzyy: x - reg group(4 - holdings, etc.), z - subGroup, yy - reg number in subGroup.(parametr)
	GROUP4_REG001	= 40001,
	GROUP4_REG002	= 40002,
	GROUP4_REG003	= 40003,
	GROUP4_REG004	= 40004,
	GROUP4_REG005	= 40005,
	GROUP4_REG101	= 40101,
	GROUP4_REG102	= 40102,
	GROUP4_REG103	= 40103,
	GROUP4_REG104	= 40104,
	GROUP4_REG105	= 40105
};
/*implemented function`s codes enum*/
enum ModbusFunctions{
	READ_HOLDINGS	= 0x03,
	WRITE_HOLDING	= 0x06
};
/*modbus register types*/
enum ModbusRegisterTypes{
	HOLDING_REG		= 1,			//for holding regs type
	UNKNOWN_REG		= 0			//uknown reg type
};
/*modbus errors codes*/
enum ModbusErrorsCodes{
	ILLEGAL_FUNCTION 			= 1,		
	ILLEGAL_DATA_ADDRES 		= 2,		
	ILLEGAL_DATA_VALUE  		= 3,		
	SLAVE_DAVICE_FAILURE 	= 4,		
	ACKNOWLEDGE 				= 5,		
	SLAVE_DEVICE_BUSY 		= 6,		
	NEGATIVE_ACKNOWLEDGE 	= 7,		
	MEMORY_PARITY_ERROR 		= 8,		
	GATEWAY_PATH_ERROR 		= 10,		
	GATEWAY_TARGET_DEVICE 	= 11		
};

/*sample struct to represent some group of registers*/
typedef struct{
	int16_t reg1;
	int16_t reg2;
	int16_t reg3;
	int16_t reg4;
	int16_t reg5;
}volatile HoldingRegGroup;

/*registers*/
extern volatile HoldingRegGroup holdingRegGroup_1;
extern volatile HoldingRegGroup holdingRegGroup_2;

#endif	//_USART_H
