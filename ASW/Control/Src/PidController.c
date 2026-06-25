#include "PidController.h"

const PidController_Config PidController_DefaultConfig = {
    10.0f,
    0.0f,
    1.0f,
    0.0f,
    0.02f,
    0.001f,
    100.0f,
    -100.0f,
    5000.0f
};

void PidController_Init(PidController *controller, const PidController_Config *config)
{
    if (controller == 0)
    {
        return;
    }

    if (config != 0)
    {
        controller->config = *config;
    }
    else
    {
        controller->config = PidController_DefaultConfig;
    }

    PidController_Reset(controller);
}

void PidController_Reset(PidController *controller)
{
    if (controller == 0)
    {
        return;
    }

    controller->integral = 0.0f;
    controller->err_prev = 0.0f;
    controller->deriv_prev = 0.0f;
    controller->command_sat_prev = 0.0f;
    controller->command_prev = 0.0f;
}

float PidController_Step(PidController *controller, float measurement, float dt)
{
    float err;
    float command;
    float command_sat;
    float deriv_filt;
    PidController_Config *config;

    if (controller == 0)
    {
        return 0.0f;
    }

    config = &controller->config;

    if (dt <= 0.0f)
    {
        dt = config->T;
    }

    if (dt <= 0.0f)
    {
        dt = PidController_DefaultConfig.T;
    }

    config->T = dt;
    err = -measurement;

    controller->integral += (config->Ki * err * dt) +
                            (config->Kaw * (controller->command_sat_prev - controller->command_prev) * dt);

    deriv_filt = (err - controller->err_prev + (config->T_C * controller->deriv_prev)) /
                 (dt + config->T_C);
    controller->err_prev = err;
    controller->deriv_prev = deriv_filt;

    command = (config->Kp * err) + controller->integral + (config->Kd * deriv_filt);
    controller->command_prev = command;

    if (command > config->max)
    {
        command_sat = config->max;
    }
    else if (command < config->min)
    {
        command_sat = config->min;
    }
    else
    {
        command_sat = command;
    }

    if (config->max_rate > 0.0f)
    {
        const float max_delta = config->max_rate * dt;
        if (command_sat > (controller->command_sat_prev + max_delta))
        {
            command_sat = controller->command_sat_prev + max_delta;
        }
        else if (command_sat < (controller->command_sat_prev - max_delta))
        {
            command_sat = controller->command_sat_prev - max_delta;
        }
    }

    controller->command_sat_prev = command_sat;

    return command_sat;
}
