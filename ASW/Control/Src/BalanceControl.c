#include "BalanceControl.h"

const BalanceControl_Config BalanceControl_DefaultConfig = {
    -2.5f,
    40.0f,
    {
        10.0f,
        0.0f,
        1.0f,
        0.0f,
        0.02f,
        0.001f,
        100.0f,
        -100.0f,
        5000.0f
    }
};

void BalanceControl_Init(BalanceControl *control, const BalanceControl_Config *config)
{
    const BalanceControl_Config *active_config = config;

    if (control == 0)
    {
        return;
    }

    if (active_config == 0)
    {
        active_config = &BalanceControl_DefaultConfig;
    }

    PidController_Init(&control->pid, &active_config->pid);
    control->state = BALANCE_CONTROL_STATE_ACTIVE;
}

void BalanceControl_Reset(BalanceControl *control)
{
    if (control == 0)
    {
        return;
    }

    PidController_Reset(&control->pid);
    control->state = BALANCE_CONTROL_STATE_ACTIVE;
}

BalanceControl_Output BalanceControl_Update(BalanceControl *control,
                                            const BalanceControl_Config *config,
                                            float control_angle_deg,
                                            float dt_s)
{
    BalanceControl_Output output = {
        0U,
        0.0f,
        0.0f,
        BALANCE_CONTROL_STATE_FALL_DETECTED
    };
    const BalanceControl_Config *active_config = config;

    if (control == 0)
    {
        return output;
    }

    if (active_config == 0)
    {
        active_config = &BalanceControl_DefaultConfig;
    }

    output.balance_error_deg = control_angle_deg - active_config->target_angle_deg;

    if ((output.balance_error_deg > active_config->fall_limit_deg) ||
        (output.balance_error_deg < -active_config->fall_limit_deg))
    {
        PidController_Reset(&control->pid);
        control->state = BALANCE_CONTROL_STATE_FALL_DETECTED;
        output.state = control->state;
        return output;
    }

    control->state = BALANCE_CONTROL_STATE_ACTIVE;
    output.motor_enable = 1U;
    output.motor_command = PidController_Step(&control->pid, output.balance_error_deg, dt_s);
    output.state = control->state;

    return output;
}
