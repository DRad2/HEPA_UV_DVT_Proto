#ifndef INC_EEPROM_H_
#define INC_EEPROM_H_
#include <stdint.h>
#include "stm32g4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

void EEPROM_Write (uint16_t page, uint16_t offset, uint8_t *data, uint16_t size);
void EEPROM_Read (uint16_t page, uint16_t offset, uint8_t *data, uint16_t size);
void EEPROM_PageErase (uint16_t page);
uint16_t bytestowrite (uint16_t size, uint16_t offset);
extern void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif // INC_EEPROM_H_
