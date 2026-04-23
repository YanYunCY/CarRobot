#include "pid.h"

#define PID_FILTER_A    0.85f     // 低通滤波系数（0~1，越大越稳但更慢）

#define CAMERA_PID_KP          0.75f
#define CAMERA_PID_KI          0.01f
#define CAMERA_PID_KD          0.2f
#define CAMERA_PID_MAX_OUTPUT  30.0f     // 最大输出（差速上限）
#define CAMERA_PID_MAX_I       30.0f     // 积分限幅（防止积分爆炸）

#define TARGET_PID_KP          3.0f
#define TARGET_PID_KI          0.0f
#define TARGET_PID_KD          1.2f
#define TARGET_PID_MAX_OUTPUT  30.0f
#define TARGET_PID_MAX_I       30.0f

typedef struct
{
    float kp;
    float ki;
    float kd;
    float max_output; 
    float max_i;

    float error;
    float last_error;
    float integral;
    float output;
    float filtered_error;
} PIDContext;

static PIDContext camera_pid = {
    .kp = CAMERA_PID_KP,
    .ki = CAMERA_PID_KI,
    .kd = CAMERA_PID_KD,
    .max_output = CAMERA_PID_MAX_OUTPUT,
    .max_i = CAMERA_PID_MAX_I,
    .error = 0.0f,
    .last_error = 0.0f,
    .integral = 0.0f,
    .output = 0.0f,
    .filtered_error = 0.0f,
};

static PIDContext target_pid = {
    .kp = TARGET_PID_KP,
    .ki = TARGET_PID_KI,
    .kd = TARGET_PID_KD,
    .max_output = TARGET_PID_MAX_OUTPUT,
    .max_i = TARGET_PID_MAX_I,
    .error = 0.0f,
    .last_error = 0.0f,
    .integral = 0.0f,
    .output = 0.0f,
    .filtered_error = 0.0f,
};

static float clamp_float(float value, float min, float max)
{
    if(value < min) return min;
    if(value > max) return max;
    return value;
}

static void PID_ResetContext(PIDContext *ctx)
{
    ctx->error = 0.0f;
    ctx->last_error = 0.0f;
    ctx->integral = 0.0f;
    ctx->output = 0.0f;
    ctx->filtered_error = 0.0f;
}

static int PID_ComputeContext(PIDContext *ctx, int error)
{
    float e = (float)error;

    // /* 一阶低通滤波 */
    // ctx->filtered_error = PID_FILTER_A * ctx->filtered_error
    //                     + (1.0f - PID_FILTER_A) * e;
    // ctx->error = ctx->filtered_error;

    ctx->error = e;

    ctx->integral += ctx->error;
    ctx->integral = clamp_float(ctx->integral, -ctx->max_i, ctx->max_i);

    float derivative = ctx->error - ctx->last_error;

    ctx->output = ctx->kp * ctx->error
                + ctx->ki * ctx->integral
                + ctx->kd * derivative;

    ctx->output = clamp_float(ctx->output, -ctx->max_output, ctx->max_output);
    ctx->last_error = ctx->error;

    return (int)ctx->output;
}

void PID_Init(void)
{
    PID_Camera_Reset();
    PID_Target_Reset();
}

void PID_Reset(void)
{
    PID_Camera_Reset();
    PID_Target_Reset();
}

void PID_Camera_Reset(void)
{
    PID_ResetContext(&camera_pid);
}

int PID_Camera_Compute(int error)
{
    return PID_ComputeContext(&camera_pid, error);
}

void PID_Target_Reset(void)
{
    PID_ResetContext(&target_pid);
}

int PID_Target_Compute(int error)
{
    return PID_ComputeContext(&target_pid, error);
}
