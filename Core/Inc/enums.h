#ifndef ENUMS_H
#define ENUMS_H

typedef enum
{
    UV_PUSHBUTTON_INTERRUPT = 0,
    DOOR_SW_INTERRUPT = 1,
	POS_SW_INTERRUPT = 2,
	HEPA_FAN_INTERRUPT = 3
} interrupt_types;
extern uint16_t interrupt_type;

typedef enum
{
	UV_READY = 4,
	UV_NOT_READY = 5
} UV_IT_flags;
extern volatile uint16_t UV_IT_flag;

typedef enum
{
	EEPROM_TEST = 6,
	NO_EEPROM_TEST = 7
} EEPROM_CAN_Msgs;

extern uint32_t EEPROM_CAN_Msg;

#endif // ENUMS_H
