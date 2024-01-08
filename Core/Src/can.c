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
extern uint32_t EEPROM_CAN_Msg;
uint8_t counter;

extern UART_HandleTypeDef hlpuart1;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

/* Can bus test */
void test_can_bus()
{
/* Receive data */
if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0))
{
	HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxData);
	HAL_Delay(500);

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
		if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) != HAL_OK)
		{
			// Transmission request Error
			  Error_Handler();
		}
		HAL_Delay(500);

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
		HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxData);
		HAL_Delay(500);
	}
}
