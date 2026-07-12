#ifndef _ENUMS_H
#define _ENUMS_H

#include <cstdint>


enum UROS_states {
  UROS_INIT,
  UROS_FOUND,
  UROS_OK,
  UROS_ERROR 
};


enum LED_States {
  LED_STATE_OFF,
  LED_STATE_AUTO,
  LED_STATE_TELEOP,
  LED_STATE_ARRIVED
};

enum EMC2305_Reg : uint8_t
{
    // FAN_1_TACH_TARGET_HIGH_BYTE = 0x3D,
    // FAN_1_TACH_TARGET_LOW_BYTE = 0x3C,
    // FAN_1_TACH_READING_HIGH_BYTE = 0x3E,
    // FAN_1_TACH_READING_LOW_BYTE = 0x3F,
    // FAN_1_CONFIG = 0x32,
    // FAN_1_TACH_VALID_COUNT = 0x39
};





extern UROS_states state_UROS;



#endif