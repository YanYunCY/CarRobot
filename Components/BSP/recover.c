#include "recover.h"
#include "motor.h"
#include "sensor.h"
#include "stm32f4xx_hal.h"
#include "math.h"

#define RECOVER_HOLD_MS 800
#define RECOVER_TURN_MS_HEAD 0
#define RECOVER_TURN_MS_TAIL 530
#define RECOVER_TURN_MS_LEFT 280
#define RECOVER_TURN_MS_RIGHT 280
#define RECOVER_TURN_MS_FRONT_RIGHT 160
#define RECOVER_TURN_MS_FRONT_LEFT 160
#define RECOVER_TURN_MS_REAR_LEFT 410
#define RECOVER_TURN_MS_REAR_RIGHT 410

typedef enum
{
    RECOVER_IDLE = 0,
    RECOVER_DETECTED,
    RECOVER_EXECUTING,
    RECOVER_DONE
} RecoverState;

typedef enum
{
    RECOVER_DIR_NONE = 0,
    RECOVER_DIR_HEAD,
    RECOVER_DIR_TAIL,
    RECOVER_DIR_LEFT,
    RECOVER_DIR_RIGHT,
    RECOVER_DIR_FRONT_RIGHT,
    RECOVER_DIR_FRONT_LEFT,
    RECOVER_DIR_REAR_LEFT,
    RECOVER_DIR_REAR_RIGHT
} RecoverDirection;

typedef enum
{
    RECOVER_PHASE_INIT = 0,
    RECOVER_PHASE_TURN,
    RECOVER_PHASE_HOLD,
    RECOVER_PHASE_FINISH
} RecoverPhase;

static struct
{
    RecoverState state;
    RecoverDirection dir;
    RecoverPhase phase;
    uint32_t phase_start;
    uint8_t done;
} recover_ctx;

static RecoverDirection detect_off_stage_runtime(void)
{
    float d[8];

    for(int i = 0; i < 8; i++)
    {
        d[i] = get_distance(i);
    }

    // if(d[5] < 70.0f && d[1] < 60.0f && d[0] < 70.0f && d[2] < 60.0f)
    //     return RECOVER_DIR_HEAD;   // 头向墙

    // if(d[6] < 70.0f && d[2] < 60.0f && d[7] < 70.0f && d[1] < 60.0f)
    //     return RECOVER_DIR_TAIL;   // 尾向墙

    // if(d[5] < 70.0f && d[3] < 60.0f && d[7] < 70.0f && d[4] < 60.0f)
    //     return RECOVER_DIR_LEFT;   // 左向墙

    // if(d[0] < 70.0f && d[4] < 60.0f && d[6] < 70.0f && d[3] < 60.0f)
    //     return RECOVER_DIR_RIGHT;  // 右向墙

    // if(d[1] < 70.0f && d[0] < 70.0f)
    //     return RECOVER_DIR_FRONT_RIGHT; // 右前向墙

    // if(d[1] < 70.0f && d[5] < 70.0f)
    //     return RECOVER_DIR_FRONT_LEFT;  // 左前向墙

    // if(d[2] < 70.0f && d[7] < 70.0f)
    //     return RECOVER_DIR_REAR_LEFT;   // 左后向墙

    // if(d[2] < 70.0f && d[6] < 70.0f)
    //     return RECOVER_DIR_REAR_RIGHT;  // 右后向墙
    if(d[1] < 40.0f && d[2] < 40.0f)
    {
        return RECOVER_DIR_TAIL;   // 头向墙
    }
    else if(d[3] < 40.0f && d[4] < 40.0f)
    {
        return RECOVER_DIR_LEFT;   // 左向墙
    }
    else if(d[5] < 40.0f)
    {
        return RECOVER_DIR_FRONT_LEFT;  // 左前向墙
    }
    else if(d[0] < 40.0f)
    {
        return RECOVER_DIR_FRONT_RIGHT; // 右前向墙
    }
     else if(d[7] < 40.0f)
    {
        return RECOVER_DIR_REAR_LEFT;   // 左后向墙
    }
    else if(d[6] < 40.0f)
    {
        return RECOVER_DIR_REAR_RIGHT;  // 右后向墙
    }

    return RECOVER_DIR_NONE;
}

static void recover_next_phase(RecoverPhase next)
{
    recover_ctx.phase = next;
    recover_ctx.phase_start = HAL_GetTick();
}

static uint32_t recover_get_turn_ms(void)
{
    switch(recover_ctx.dir)
    {
        case RECOVER_DIR_HEAD:
            return RECOVER_TURN_MS_HEAD;
        case RECOVER_DIR_TAIL:
            return RECOVER_TURN_MS_TAIL;
        case RECOVER_DIR_LEFT:
            return RECOVER_TURN_MS_LEFT;
        case RECOVER_DIR_RIGHT:
            return RECOVER_TURN_MS_RIGHT;
        case RECOVER_DIR_FRONT_RIGHT:
            return RECOVER_TURN_MS_FRONT_RIGHT;
        case RECOVER_DIR_FRONT_LEFT:
            return RECOVER_TURN_MS_FRONT_LEFT;
        case RECOVER_DIR_REAR_LEFT:
            return RECOVER_TURN_MS_REAR_LEFT;
        case RECOVER_DIR_REAR_RIGHT:
            return RECOVER_TURN_MS_REAR_RIGHT;
        default:
            return RECOVER_TURN_MS_HEAD;
    }
}

static void recover_execute_phase(void)
{
    uint32_t elapsed = HAL_GetTick() - recover_ctx.phase_start;

    switch(recover_ctx.phase)
    {
        case RECOVER_PHASE_INIT:
        {
            if(recover_ctx.dir == RECOVER_DIR_HEAD ||
               recover_ctx.dir == RECOVER_DIR_TAIL ||
               recover_ctx.dir == RECOVER_DIR_FRONT_LEFT ||
               recover_ctx.dir == RECOVER_DIR_FRONT_RIGHT)
            {
                recover_next_phase(RECOVER_PHASE_TURN);
            }
            else
            {
                recover_next_phase(RECOVER_PHASE_TURN);
            }
            break;
        }

        case RECOVER_PHASE_TURN:
        {
            switch(recover_ctx.dir)
            {
                case RECOVER_DIR_HEAD:
                    run(stop, 0);
                    break;
                case RECOVER_DIR_FRONT_LEFT:
                case RECOVER_DIR_REAR_LEFT:
                    run(left, 20);
                    break;
                case RECOVER_DIR_TAIL:
                case RECOVER_DIR_FRONT_RIGHT:
                case RECOVER_DIR_REAR_RIGHT:
                    run(right, 20);
                    break;
                case RECOVER_DIR_LEFT:
                    run(left, 20);
                    break;
                case RECOVER_DIR_RIGHT:
                    run(right, 20);
                    break;
                default:
                    run(stop, 0);
                    break;
            }
            if(elapsed >= recover_get_turn_ms())
            {
                recover_next_phase(RECOVER_PHASE_HOLD);
            }
            break;
        }

        case RECOVER_PHASE_HOLD:
        {
            float d1 = get_distance(1);
            float d2 = get_distance(2);
            float diff = d1 - d2;

            // 先判断是否结束（最重要！）
            if(elapsed >= RECOVER_HOLD_MS)
            {
                recover_next_phase(RECOVER_PHASE_FINISH);
                break;
            }

            // 再做调整
            if(fabsf(diff) >= 10.0f)
            {
                if(diff > 0.0f)
                    run(go, 10);
                else
                    run(back, 10);
            }
            else
            {
                run(back, 10);
            }

            break;
        }

        case RECOVER_PHASE_FINISH:
        {
            recover_ctx.state = RECOVER_DONE;
            recover_ctx.done = 1;
            break;
        }

        default:
            break;
    }
}

void recover_init(void)
{
    recover_ctx.state = RECOVER_IDLE;
    recover_ctx.dir = RECOVER_DIR_NONE;
    recover_ctx.phase = RECOVER_PHASE_INIT;
    recover_ctx.phase_start = 0;
    recover_ctx.done = 0;
}

void recover_start(void)
{
    RecoverDirection dir = detect_off_stage_runtime();
    if(dir == RECOVER_DIR_NONE)
        return;

    recover_ctx.state = RECOVER_DETECTED;
    recover_ctx.dir = dir;
    recover_ctx.phase = RECOVER_PHASE_INIT;
    recover_ctx.phase_start = HAL_GetTick();
    recover_ctx.done = 0;
}

void recover_update(void)
{
    if(recover_ctx.state == RECOVER_IDLE || recover_ctx.state == RECOVER_DONE)
        return;

    if(recover_ctx.state == RECOVER_DETECTED)
    {
        recover_ctx.state = RECOVER_EXECUTING;
        recover_ctx.phase = RECOVER_PHASE_INIT;
        recover_ctx.phase_start = HAL_GetTick();
    }

    if(recover_ctx.state == RECOVER_EXECUTING)
    {
        recover_execute_phase();
    }
}

uint8_t recover_is_active(void)
{
    return recover_ctx.state == RECOVER_EXECUTING || recover_ctx.state == RECOVER_DETECTED;
}

uint8_t recover_is_done(void)
{
    return recover_ctx.done;
}


uint8_t recover_detect_off_stage(void)
{
    return (uint8_t)detect_off_stage_runtime();
}

void recover_blocking(void)
{
    RecoverDirection dir = detect_off_stage_runtime();
    if(dir == RECOVER_DIR_NONE) return;

    switch(dir)
    {
        case RECOVER_DIR_HEAD:
            run(stop, 0);
            HAL_Delay(RECOVER_TURN_MS_HEAD);
            break;

        case RECOVER_DIR_TAIL:
            run(right, 20);
            HAL_Delay(RECOVER_TURN_MS_TAIL);
            break;

        case RECOVER_DIR_LEFT:
            run(left, 20);
            HAL_Delay(RECOVER_TURN_MS_LEFT);
            break;

        case RECOVER_DIR_FRONT_LEFT:
            run(left, 20);
            HAL_Delay(RECOVER_TURN_MS_FRONT_LEFT);
            break;

        case RECOVER_DIR_REAR_LEFT:
            run(left, 20);
            HAL_Delay(RECOVER_TURN_MS_REAR_LEFT);
            break;

        case RECOVER_DIR_RIGHT:
            run(right, 20);
            HAL_Delay(RECOVER_TURN_MS_RIGHT);
            break;

        case RECOVER_DIR_FRONT_RIGHT:
            run(right, 20);
            HAL_Delay(RECOVER_TURN_MS_FRONT_RIGHT);
            break;
            
        case RECOVER_DIR_REAR_RIGHT:
            run(right, 20);
            HAL_Delay(RECOVER_TURN_MS_REAR_RIGHT);
            break;

        default:
            break;
    }

    // 统一后退一点（脱离边缘）1
    run(back, 45);
    HAL_Delay(RECOVER_HOLD_MS);

    run(stop, 0);
}