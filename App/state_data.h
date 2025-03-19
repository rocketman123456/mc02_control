#ifndef __STATE_DATA_H
#define __STATE_DATA_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "main.h"

    typedef struct
    {
        uint16_t id;
        uint16_t mst_id;
        // MOTOR_STATE_T para;
        // MOTOR_CTRL_T ctrl;
        // ESC_INFO_T tmp;
    } GlobalMotorState;

    typedef struct
    {
        uint16_t id;
        uint16_t mst_id;
        // MOTOR_STATE_T para;
        // MOTOR_CTRL_T ctrl;
        // ESC_INFO_T tmp;
    } GlobalMotorCmd;

    extern GlobalMotorState motor_states[6];
    extern GlobalMotorCmd   motor_cmds[6];

#ifdef __cplusplus
}
#endif

#endif