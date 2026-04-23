#include "edge.h"
#include "motor.h"
#include "stm32f4xx_hal.h"

#define EDGE_STOP_MS   50
#define EDGE_TURN_MS   250

typedef enum
{
    STATE_RUN = 0,
    //STATE_STOP,
    STATE_BACK,
    STATE_TURN
} EdgeState;

static EdgeState car_state = STATE_RUN;

static volatile uint8_t left_trigger = 0;
static volatile uint8_t right_trigger = 0;
static volatile uint8_t trigger_updated = 0;

static uint32_t last_tick_left = 0;
static uint32_t last_tick_right = 0;
static uint32_t state_time = 0;
static uint8_t turn_dir = 0; // 1左 2右

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint32_t tick = HAL_GetTick();

    if (GPIO_Pin == GPIO_PIN_12)   // 右侧
    {
        if (tick - last_tick_right < 10)
            return;
        last_tick_right = tick;
        right_trigger = 1;
    }
    else if (GPIO_Pin == GPIO_PIN_13) // 左侧
    {
        if (tick - last_tick_left < 10)
            return;
        last_tick_left = tick;
        left_trigger = 1;
    }
    else
    {
        return;
    }

    trigger_updated = 1;
}

uint8_t edge_is_busy(void)
{
    return car_state != STATE_RUN;
}

static void update_turn_dir(void)
{
    if(left_trigger && right_trigger)
    {
        // 双边同时触发时固定一个方向，避免随机冲出
        turn_dir = 2; // 统一向右转
    }
    else if(left_trigger)
    {
        turn_dir = 1; // 左检测 → 右转
    }
    else if(right_trigger)
    {
        turn_dir = 2; // 右检测 → 左转
    }
}

static void process_edge_trigger(void)
{
    if(trigger_updated)
    {
        if(car_state == STATE_RUN)
        {
            update_turn_dir();
            car_state = STATE_BACK;
            state_time = HAL_GetTick();
        }
        else
        {
            update_turn_dir();
            car_state = STATE_BACK;
            state_time = HAL_GetTick();
        }
        trigger_updated = 0;
    }
}

void edge_update(void)
{
    process_edge_trigger();

    switch(car_state)
    {
        case STATE_RUN:
        {
            run(go, 13);
        }
        break;

        case STATE_BACK:
        {
            run(back, 15);

            if(left_trigger && HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_13) == GPIO_PIN_SET)
            {
                left_trigger = 0;
            }
            if(right_trigger && HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_12) == GPIO_PIN_SET)
            {
                right_trigger = 0;
            }

            if(!left_trigger && !right_trigger)
            {
                car_state = STATE_TURN;
                state_time = HAL_GetTick();
                trigger_updated = 0;
            }
        }
        break;

        case STATE_TURN:
        {
            if(turn_dir == 1)
                run(left, 20);
            else
                run(right, 20);

            if(HAL_GetTick() - state_time > EDGE_TURN_MS)
            {
                car_state = STATE_RUN;
            }
        }
        break;
    }
}

void edge_update_no_run(void)
{
    process_edge_trigger();

    if(car_state == STATE_BACK)
    {
        run(back, 15);

        if(left_trigger && HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_13) == GPIO_PIN_SET)
        {
            left_trigger = 0;
        }
        if(right_trigger && HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_12) == GPIO_PIN_SET)
        {
            right_trigger = 0;
        }

        if(!left_trigger && !right_trigger)
        {
            car_state = STATE_TURN;
            state_time = HAL_GetTick();
            trigger_updated = 0;
        }
    }
    else if(car_state == STATE_TURN)
    {
        if(turn_dir == 1)
            run(left, 20);
        else
            run(right, 20);

        if(HAL_GetTick() - state_time > EDGE_TURN_MS)
        {
            car_state = STATE_RUN;
        }
    }
}
