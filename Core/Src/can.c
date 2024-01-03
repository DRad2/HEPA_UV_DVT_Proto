#include "can.h"
#include <stdio.h>
#include <string.h>
#include "enums.h"

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_RxHeaderTypeDef RxHeader;
extern FDCAN_TxHeaderTypeDef TxHeader;
extern uint8_t debug_str[32];
extern uint8_t RxData[8];
extern uint8_t TxData[8];
extern uint8_t EEPROM_CAN_Msg;
uint8_t counter;

uint8_t NL[] = "-----------------------------------\r\n";


extern UART_HandleTypeDef hlpuart1;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

//char msg[100];

/* Can bus test */
void test_can_bus()
{
/* Receive data */
if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0))
{
	//HAL_UART_Transmit(&hlpuart1, "Received:", 9, HAL_MAX_DELAY);
	HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxData);
	HAL_Delay(500);
	//sprintf((char*) debug_str, "%02x %02x %02x %02x %02x %02x %02x %02x\r\n", RxData[0], RxData[1], RxData[2], RxData[3], RxData[4], RxData[5], RxData[6], RxData[7]);

	/* Prepare received data to be sent back */
	counter = 0;
	for (int i = 0; i<8; i++)
	{
		/* AUX 1 increase by 1 */
		if (RxData[i] == 0)
		{
			counter++;
		}
		TxData[i]= RxData[i]+1;
		/* AUX 2 increase by 2 */
		//TxData[i]= RxData[i] + 2;
	}
		//HAL_UART_Transmit(&hlpuart1, (uint8_t*)debug_str, strlen((const char*)(debug_str)), HAL_MAX_DELAY);
		//HAL_UART_Transmit(&hlpuart1, "Sent:    ", 9, HAL_MAX_DELAY);
		if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) != HAL_OK)
		{
			// Transmission request Error
			  Error_Handler();
		}
		HAL_Delay(500);
		sprintf((char*) debug_str, "%02x %02x %02x %02x %02x %02x %02x %02x\r\n", TxData[0],
					  TxData[1], TxData[2], TxData[3], TxData[4], TxData[5], TxData[6], TxData[7]);
		//HAL_UART_Transmit(&hlpuart1, (uint8_t*)debug_str, strlen((const char*)debug_str), HAL_MAX_DELAY);
		//HAL_UART_Transmit(&hlpuart1, NL, sizeof(NL), HAL_MAX_DELAY);

		if (counter == 8)
		{
			EEPROM_CAN_Msg = EEPROM_TEST;
		}
		else EEPROM_CAN_Msg = NO_EEPROM_TEST;
		counter = 0;
	}
}

void send_msg(uint8_t* msg, int len)
{
	for (int i = 0; i<len; i++)
		{
			TxData[i] = msg[i];
		}
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) != HAL_OK)
			{
				// Transmission request Error
				  Error_Handler();
			}
	HAL_Delay(500);
}

void can_listen()
{
	if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0))
	{
		HAL_UART_Transmit(&hlpuart1, "Received:", 9, HAL_MAX_DELAY);
		HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxData);
		HAL_Delay(500);
		sprintf((char*) debug_str, "%02x %02x %02x %02x %02x %02x %02x %02x\r\n", RxData[0], RxData[1], RxData[2], RxData[3], RxData[4], RxData[5], RxData[6], RxData[7]);
		HAL_UART_Transmit(&hlpuart1, (uint8_t*)debug_str, strlen((const char*)debug_str), HAL_MAX_DELAY);
		HAL_UART_Transmit(&hlpuart1, NL, sizeof(NL), HAL_MAX_DELAY);
	}
}
