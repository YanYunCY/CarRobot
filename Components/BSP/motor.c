#include "motor.h"
#include "tim.h"

void run(int st1,int st2)
{
    if(st2 >= 100) st2 = 100;
    if(st2 <= -100) st2 = -100;

    int delta = st2 * speed / 100 / 2;

    if(st1 == stop)
    {
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 150);
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 150);
    }
    else if(st1 == go)
    {
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 150 + delta);
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 150 + delta);
    }
    else if(st1 == back)
    {
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 150 - delta);
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 150 - delta);
    }
    else if(st1 == right)
    {
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 150 + delta);
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 150 - delta);
    }
    else if(st1 == left)
    {
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 150 - delta);
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 150 + delta);
    }
    else
    {
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 150);
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 150);
    }
}

void run_diff(int st1,int st2)
{
    if(st1 >= 100) st1 = 100;
    if(st1 <= -100) st1 = -100;
	if(st2 >= 100) st2 = 100;
    if(st2 <= -100) st2 = -100;

    int left_speed = st1 * speed / 100 / 2;
	int right_speed = st2 * speed / 100 / 2;

    /* 限幅 */
    if(left_speed > 50) left_speed = 50;
    if(left_speed < -50) left_speed = -50;

    if(right_speed > 50) right_speed = 50;
    if(right_speed < -50) right_speed = -50;

    /* 输出PWM */
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1,150 + left_speed);
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2,150 + right_speed);
}