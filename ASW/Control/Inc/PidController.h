#ifndef PID_CONTROLLER_H_
#define PID_CONTROLLER_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float Kaw;
    float T_C;
    float T;
    float max;
    float min;
    float max_rate;
} PidController_Config;

typedef struct
{
    PidController_Config config;
    float integral;
    float err_prev;
    float deriv_prev;
    float command_sat_prev;
    float command_prev;
} PidController;

typedef PidController PID;

extern const PidController_Config PidController_DefaultConfig;

void PidController_Init(PidController *controller, const PidController_Config *config);
void PidController_Reset(PidController *controller);
float PidController_Step(PidController *controller, float measurement, float dt);

#ifdef __cplusplus
}
#endif

#endif /* PID_CONTROLLER_H_ */
