#ifndef BALANCE_CONTROL_H_
#define BALANCE_CONTROL_H_

#include <stdint.h>
#include "PidController.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    BALANCE_CONTROL_STATE_ACTIVE = 0,
    BALANCE_CONTROL_STATE_FALL_DETECTED
} BalanceControl_State;

typedef struct
{
    float target_angle_deg;
    float fall_limit_deg;
    PidController_Config pid;
} BalanceControl_Config;

typedef struct
{
    uint8_t motor_enable;
    float motor_command;
    float balance_error_deg;
    BalanceControl_State state;
} BalanceControl_Output;

typedef struct
{
    PidController pid;
    BalanceControl_State state;
} BalanceControl;

extern const BalanceControl_Config BalanceControl_DefaultConfig;

void BalanceControl_Init(BalanceControl *control, const BalanceControl_Config *config);
void BalanceControl_Reset(BalanceControl *control);
BalanceControl_Output BalanceControl_Update(BalanceControl *control,
                                            const BalanceControl_Config *config,
                                            float control_angle_deg,
                                            float dt_s);

#ifdef __cplusplus
}
#endif

#endif /* BALANCE_CONTROL_H_ */
