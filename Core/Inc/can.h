#ifndef INC_CAN_H_
#define INC_CAN_H_

#include <stdint.h>
#include "stm32g4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

extern void Error_Handler(void);
void test_can_bus();
void can_listen();
void send_msg(uint8_t* msg, int len);


#ifdef __cplusplus
}
#endif

#endif // INC_CAN_H_
