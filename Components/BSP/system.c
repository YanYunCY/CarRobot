#include "system.h"
#include "motor.h"
#include "sensor.h"
#include "edge.h"
#include "math.h"
#include "camera.h"
#include "pid.h"
#include "stdlib.h"
#include "stm32f4xx_hal.h"

#define CAMERA_IMAGE_WIDTH           160
#define CAMERA_IMAGE_CENTER_X        (CAMERA_IMAGE_WIDTH / 2)
#define CAMERA_PID_MAX_OUTPUT        20
#define CAMERA_PID_MIN_OUTPUT        10
#define CAMERA_PID_DEADZONE          8
#define CAMERA_PID_MAX_FORWARD       20
#define CAMERA_PID_MIN_FORWARD       10
#define CAMERA_QR_PUSH_WIDTH         40
#define CAMERA_QR_PUSH_EXIT_WIDTH    30
#define CAMERA_QR_PUSH_SPEED         15

#define TARGET_ATTACK_SPEED_FAR      15
#define TARGET_ATTACK_SPEED_NEAR     15
#define TARGET_ATTACK_MAX_SPEED      18

typedef enum
{
    SYS_OFF_STAGE = 0, //台下
    SYS_CLIMB,  //上台
    SYS_PATROL,  //巡台
    SYS_TRACK_QR,  //April码
    SYS_ATTACK //攻击
    //SYS_RECOVER //掉台恢复
} SystemState;

typedef enum
{
    CLIMB_PHASE_INIT = 0,
    CLIMB_PHASE_UPSTAGE,
    CLIMB_PHASE_DONE
} ClimbPhase;

typedef enum
{
    AVOID_IDLE = 0,
    AVOID_TURN,
    AVOID_COOLDOWN
} AvoidState;

static AvoidState avoid_state = AVOID_IDLE;
static uint32_t avoid_state_time = 0;

static SystemState state = SYS_PATROL;
static ClimbPhase climb_phase = CLIMB_PHASE_INIT;
static uint32_t climb_time = 0;

// 传感器信息
typedef struct
{
    uint8_t on_stage;
    uint8_t qr_detected;
    int     qr_x;
    int     qr_id;
    uint8_t target_detected;
    int     target_dir;
} SensorInfo;

static SensorInfo sensor;

/*========工具函数========*/

// 判断是否在台上
static uint8_t detect_on_stage(void)
{
    static uint8_t cnt = 0;

    float d4 = get_distance(1);
    float d5 = get_distance(2);

    if(d4 < 35.0f && d5 < 35.0f)
    {
        if(cnt < 3) cnt++;   // 累计3次
    }
    else
    {
        cnt = 0;
    }

    // 连续满足认为在台上
    if(cnt >= 3)
        return 0;
    else
        return 1;
}

// static uint8_t detect_off_stage_runtime(void)
// {
//     float d[8];

//     for(int i = 0; i < 8; i++)
//     {
//         d[i] = get_distance(i);
//     }

//     /* ===== 四组判断 ===== */

//     // 组1: 5,1,0,2
//     if(d[5] < 60.0f && d[1] < 60.0f && d[0] < 60.0f && d[2] < 60.0f)
//         return 1;

//     // 组2: 6,2,7,1
//     if(d[6] < 60.0f && d[2] < 60.0f && d[7] < 60.0f && d[1] < 60.0f)
//         return 1;

//     // 组3: 5,3,7,4
//     if(d[5] < 60.0f && d[3] < 60.0f && d[7] < 60.0f && d[4] < 60.0f)
//         return 1;

//     // 组4: 0,4,6,3
//     if(d[0] < 60.0f && d[4] < 60.0f && d[6] < 60.0f && d[3] < 60.0f)
//         return 1;

//     //组5：1，0，2，3
//     if(d[1] < 60.0f && d[0] < 60.0f && d[2] < 60.0f && d[3] < 60.0f)
//         return 1;

//     //组6：1，5，2，6
//     if(d[1] < 60.0f && d[5] < 60.0f && d[2] < 60.0f && d[6] < 60.0f)
//         return 1;   

//     return 0;  // 没触发 → 台上
// }

// 找最近方向
// static int get_nearest_dir(void)
// {

//     return 0;
// }

// 目标检测
//PID版
#define TARGET_THRESHOLD  40.0f
#define MIN_DISTANCE      8.0f

static float target_error = 0.0f;   // 连续误差（给PID用）

static void detect_target(void)
{
    float dist[8];
    float weight[8] = {
    -0.707f,  // 0 右前 (-45°)
     0.0f,    // 1 前   (0°)
     0.0f,    // 2 后   (180°) ← 不参与PID
     1.0f,    // 3 左   (90°)
    -1.0f,    // 4 右   (-90°)
     0.707f,  // 5 左前 (45°)
    -0.707f,  // 6 右后 (-135°)
     0.707f   // 7 左后 (135°)
}; // 前后权重

     float sum = 0.0f;      // 加权和
    float sum_w = 0.0f;    // 总强度

    sensor.target_detected = 0;

    for(uint8_t i = 0; i < 8; i++)
    {
        dist[i] = get_distance(i);

        /* 防止贴墙爆炸 */
        if(dist[i] < MIN_DISTANCE)
            dist[i] = MIN_DISTANCE;
    }

    for(uint8_t i = 0; i < 8; i++)
    {
        if(dist[i] < TARGET_THRESHOLD)
        {
            sensor.target_detected = 1;

            /* 距离 → 强度 */
            float strength = TARGET_THRESHOLD - dist[i];

            sum += weight[i] * strength;
            sum_w += strength;
        }
    }

    if(!sensor.target_detected || sum_w < 1e-3f)
    {
        target_error = 0;
        return;
    }

    target_error = sum / sum_w;

    if(target_error > 1.5f)  target_error = 1.5f;
    if(target_error < -1.5f) target_error = -1.5f;
}

static int clamp_int(int value, int min, int max)
{
    if(value < min) return min;
    if(value > max) return max;
    return value;
}

// 二维码检测
static void detect_qr(void)
{
    CameraInfo camera_info;
    Camera_GetInfo(&camera_info);

    if(camera_info.valid)
    {
        sensor.qr_detected = 1;
        sensor.qr_x = camera_info.x;
        sensor.qr_id = camera_info.id;
    }
    else
    {
        sensor.qr_detected = 0;
        sensor.qr_id = -1;
    }
}

/*========功能实现========*/
// 上台
static void climb_stage(void)
{
    switch(climb_phase)
    {
        case CLIMB_PHASE_INIT:
            run(stop, 0);
            climb_phase = CLIMB_PHASE_UPSTAGE;
            climb_time = HAL_GetTick();
            break;

        case CLIMB_PHASE_UPSTAGE:
            run(back, 45);
            if(HAL_GetTick() - climb_time > 800)
            {
                climb_phase = CLIMB_PHASE_DONE;
            }
            break;

        case CLIMB_PHASE_DONE:
            run(stop, 0);
            break;
    }
}

// //掉台处理
// static uint8_t recover_done = 0;
// static uint32_t recover_done_time = 0;
// static const uint32_t RECOVER_HOLD_MS = 200;

// static void recover_behavior(void)
// {
//     float adc_left  = get_distance(5);
//     float adc_right = get_distance(0);
//     float adc_front = get_distance(1);

//     float diff = adc_left - adc_right;

//     /* ===== 判断是否恢复完成 ===== */
//     if(adc_left < 60 && adc_right < 60 && fabs(diff) < 3.0f && adc_front < 40.0f)
//     {
//         if(!recover_done)
//         {
//             recover_done = 1;
//             recover_done_time = HAL_GetTick();
//             printf("RECOVER DONE\r\n");
//         }

//         run(stop, 0);
//         return;
//     }

//     /* ===== 转向调整 ===== */
//     float turn = diff * 5;

//     if(turn > 5) turn = 5;
//     if(turn < -5) turn = -5;

//     run_diff(turn, -turn);

//     printf("RECOVERING | diff=%.2f front=%.2f\r\n", diff, adc_front);
// }

//巡台
static void patrol_behavior(void)
{
    // 边缘优先
    edge_update();
    if(edge_is_busy())
        return;

    // 默认前进
//    run(go, 15);

    // 同时检测
    detect_qr();
    detect_target();

    // 状态跳转
    if(sensor.qr_detected)
        state = SYS_TRACK_QR;
    else if(sensor.target_detected)
        state = SYS_ATTACK;
}

// 二维码追踪
static uint8_t qr_push_mode = 0;
static int selected_qr_id = 0; 
static uint32_t avoid_lock_time = 0;
#define AVOID_LOCK_DURATION 500   // 500ms内禁止攻击

static int avoid_id = -1;         // 被锁定的二维码ID

static void track_qr(void)
{
    CameraInfo camera_info;
    Camera_GetInfo(&camera_info);

    uint8_t avoid_mode = 0;

    /* ===== 无目标 ===== */
    if(!camera_info.valid)
    {
        sensor.qr_detected = 0;
        qr_push_mode = 0;
        PID_Camera_Reset();
        state = SYS_PATROL;
        return;
    }

    /* ===== 过滤非法ID ===== */
    if(camera_info.id < 0 || camera_info.id > 2)
    {
        sensor.qr_detected = 0;
        qr_push_mode = 0;
        return;
    }

    sensor.qr_detected = 1;
    sensor.qr_x = camera_info.x;
    sensor.qr_id = camera_info.id;

    //不是 selected_qr_id → 永远避让，绝不 push
    if(camera_info.id != selected_qr_id && camera_info.id != 0)
    {
        avoid_mode = 1;
        qr_push_mode = 0;
    }
    else
    {
        avoid_mode = 0;

        /* ===== push模式 ===== */
        if(camera_info.w >= CAMERA_QR_PUSH_WIDTH)
        {
            qr_push_mode = 1;
        }
        else if(camera_info.w <= CAMERA_QR_PUSH_EXIT_WIDTH)
        {
            qr_push_mode = 0;
        }
    }

    /* ===== 避让模式 ===== */
    if(avoid_mode)
    {
        avoid_lock_time = HAL_GetTick();
        avoid_id = camera_info.id; 

        int tag_center_x = camera_info.x + camera_info.w / 2;
        int error = tag_center_x - CAMERA_IMAGE_CENTER_X;

        int pid_output = PID_Camera_Compute(error);

        int forward_speed = -12;   // 后退避让

        /* 软死区 */
        int abs_error = error < 0 ? -error : error;
        if(abs_error < CAMERA_PID_DEADZONE)
        {
            float ratio = (float)abs_error / CAMERA_PID_DEADZONE;
            pid_output = (int)(pid_output * ratio * ratio);
        }

        int left_speed  = forward_speed - pid_output;
        int right_speed = forward_speed + pid_output;

        left_speed  = clamp_int(left_speed,  -25, 25);
        right_speed = clamp_int(right_speed, -25, 25);

        run_diff(left_speed, right_speed);
        return;
    }

    /* ===== PUSH模式 ===== */
    if(qr_push_mode)
    {
        run_diff(CAMERA_QR_PUSH_SPEED, CAMERA_QR_PUSH_SPEED);
        return;
    }

    /* ===== 正常追踪 ===== */
    int tag_center_x = camera_info.x + camera_info.w / 2;
    int error = tag_center_x - CAMERA_IMAGE_CENTER_X;

    int pid_output = PID_Camera_Compute(error);

    int forward_speed;

    if(camera_info.w < 20)
    {
        forward_speed = 12;
    }
    else if(camera_info.w < CAMERA_QR_PUSH_WIDTH)
    {
        forward_speed = 12 +
            (camera_info.w - 20) *
            (CAMERA_PID_MAX_FORWARD - 12) /
            (CAMERA_QR_PUSH_WIDTH - 20);

        forward_speed = clamp_int(forward_speed, 12, CAMERA_PID_MAX_FORWARD);
    }
    else
    {
        forward_speed = CAMERA_PID_MAX_FORWARD;
    }

    /* 软死区 */
    int abs_error = error < 0 ? -error : error;
    if(abs_error < CAMERA_PID_DEADZONE)
    {
        float ratio = (float)abs_error / CAMERA_PID_DEADZONE;
        pid_output = (int)(pid_output * ratio * ratio);
    }

    int left_speed  = forward_speed + pid_output;
    int right_speed = forward_speed - pid_output;

    left_speed  = clamp_int(left_speed,  -25, 25);
    right_speed = clamp_int(right_speed, -25, 25);

    run_diff(left_speed, right_speed);
}

//攻击
//PID版
static uint8_t escape_mode = 0;
static uint32_t escape_start_time = 0;
#define ESCAPE_DURATION 2000   // 2秒

static void attack_target(void)
{
    static uint32_t last_seen_time = 0;

    uint32_t time_now = HAL_GetTick();

    /* ===== 黑名单保护（只对指定ID生效）===== */
   if(time_now - avoid_lock_time < AVOID_LOCK_DURATION)
    {
        if(sensor.qr_detected && 
        sensor.qr_id != selected_qr_id && 
        sensor.qr_id != 0)
        {
            //run(back, 15);
            run_diff(-20, 20);
            HAL_Delay(500);
            state = SYS_PATROL;
            return;
        }
    }

    //避让状态机
    edge_update_no_run();
    if(edge_is_busy())
    {
        return;
    }

    if(avoid_state == AVOID_IDLE)
    {
        if(time_now - avoid_lock_time < AVOID_LOCK_DURATION && 
            sensor.qr_detected && 
            sensor.qr_id != selected_qr_id &&
            sensor.qr_id != 0)
        {
            avoid_state = AVOID_TURN;
            avoid_state_time = time_now;
        }
    }

    switch(avoid_state)
    {
        case AVOID_TURN:
        {
            run_diff(20, -20);   // 原地转

            if(time_now - avoid_state_time > 800)
            {
                avoid_state = AVOID_COOLDOWN;
                avoid_state_time = time_now;
            }

            return;
        }

        case AVOID_COOLDOWN:
        {
            run(go, 15);

            if(time_now - avoid_state_time > 2000)
            {
                avoid_state = AVOID_IDLE;

                escape_mode = 1;
                escape_start_time = time_now;
            }

            state = SYS_PATROL;
            return;
        }

        default:
            break;
    }

    
    detect_target();
    detect_qr();

    //优先二维码
    if(sensor.qr_detected)
    {
        state = SYS_TRACK_QR;
        return;
    }

    // 丢失目标处理
    if(!sensor.target_detected)
    {
        PID_Target_Reset();
        target_error = 0;

        if(time_now - last_seen_time > 100)
        {
            state = SYS_PATROL;
            return;
        }
    }
    else
    {
        last_seen_time = time_now;
    }

    /* ===== 边缘保护 ===== */
    edge_update_no_run();
    if(edge_is_busy())
    {
        return;
    }

    //后方检测
    float back_dist = get_distance(2);   // 2号：正后

    /* 左后 + 右后（辅助判断） */
    float back_left  = get_distance(7);
    float back_right = get_distance(6);

    /* 强度（越近越大） */
    float back_strength = 0;

    if(back_dist < 40.0f)
        back_strength += (40.0f - back_dist);

    if(back_left < 40.0f)
        back_strength += 0.5f * (40.0f - back_left);

    if(back_right < 40.0f)
        back_strength += 0.5f * (40.0f - back_right);

    /* ===== 如果后方很强 → 掉头 ===== */
    if(back_strength > 20.0f)
    {
        /* 判断偏左还是偏右 */
        if(back_left < back_right)
        {
            /* 左后更近 → 向左转 */
            run_diff(-20, 20);
        }
        else
        {
            /* 右后更近 → 向右转 */
            run_diff(20, -20);
        }

        printf("BACK TURN | strength=%.2f\r\n", back_strength);
        return;
    }

    /* ===== 计算误差 ===== */
    float abs_err = fabs(target_error);

    /* ===== 大误差原地转 ===== */
    if(abs_err > 0.8f)
    {
        if(target_error > 0)
            run_diff(-20, 20);   // 右前切入
        else
            run_diff(20, -20);   // 左前切入

        return;
    }

    /* 正常前方PID追踪 */
    float d1 = get_distance(1);   // 前
    float d2 = get_distance(2);   // 后（这里主要用于中心判断）
    float center = (d1 < d2) ? d1 : d2;

    //贴脸直推模式
    float adc_left  = get_distance(5);  // 左前
    float adc_right = get_distance(0);  // 右前

    if(center < 20.0f)
    {
        float diff = fabs(adc_left - adc_right);

        if(diff < 3.0f)
        {
            run_diff(15, 15);

            printf("PUSH MODE | center=%.2f diff=%.2f\r\n", center, diff);
            return;
        }
    }

    int base_speed;

    if(center < 40.0f)
    {
        base_speed = TARGET_ATTACK_SPEED_NEAR;   // 靠近 → 低速冲击
    }
    else
    {
        base_speed = TARGET_ATTACK_SPEED_FAR;   // 远 → 追踪
    }

    //渐进前进
    float scale;

    if(abs_err > 0.5f)
        scale = 0.5f;
    else
        scale = 1.0f;

    int base_scaled = (int)(base_speed * scale);

    float turn_f = PID_Target_Compute((int)(target_error * 6));
    int turn = (int)turn_f;

    int left_speed  = base_scaled - turn;
    int right_speed = base_scaled + turn;

    /* 限幅 */
    if(left_speed > 25) left_speed = 25;
    if(left_speed < -25) left_speed = -25;

    if(right_speed > 25) right_speed = 25;
    if(right_speed < -25) right_speed = -25;

    printf("Target Error: %.2f | Turn: %d | L: %d | R: %d\r\n",
           target_error, turn, left_speed, right_speed);

    run_diff(left_speed, right_speed);
}


/*========状态更新========*/

void system_update(void)
{
    static uint8_t init_done = 0;
    static uint8_t init_selected = 0; 
    static uint32_t init_time = 0;

    /* ===== 强制脱离模式（最高优先级）===== */
    if(escape_mode)
    {
        uint32_t now = HAL_GetTick();

        PID_Target_Reset();
        PID_Camera_Reset();

        sensor.target_detected = 0;
        sensor.qr_detected = 0;

        run(go, 15);

        edge_update_no_run();
        if(edge_is_busy())
        {
            return;
        }

        if(now - escape_start_time > ESCAPE_DURATION)
        {
            escape_mode = 0;
        }

        return;
    }

    /* ===== 开机初始化选择目标ID ===== */
    if(!init_done)
    {
        float d3 = get_distance(3);
        float d4 = get_distance(4);

        if(!init_selected)
        {
            if(d3 < 15.0f)
            {
                selected_qr_id = 2;
                init_selected = 1;
                init_time = HAL_GetTick();

                printf("INIT: select ID = 2\r\n");
            }
            else if(d4 < 15.0f)
            {
                selected_qr_id = 1;
                init_selected = 1;
                init_time = HAL_GetTick();

                printf("INIT: select ID = 1\r\n");
            }
            else
            {
                run(stop, 0);
                return;
            }
        }
        else
        {
            if(HAL_GetTick() - init_time < 500) 
            {
                run(stop, 0);
                return;
            }

            init_done = 1;
            printf("INIT DONE\r\n");
        }
    }

    sensor.on_stage = detect_on_stage();

    // if(recover_done)
    // {
    //     if(HAL_GetTick() - recover_done_time >= RECOVER_HOLD_MS)
    //     {
    //         recover_done = 0;
    //     }
    // }

    // if(state != SYS_RECOVER && !recover_done && detect_off_stage_runtime())
    // {
    //     state = SYS_RECOVER;
    // }

    switch(state)
    {
        case SYS_OFF_STAGE:
        {
            run(back,5);

            if(sensor.on_stage == 0)               
                state = SYS_CLIMB;
            //state = SYS_PATROL;
        }
        break;

        case SYS_CLIMB:
        {
            climb_stage();

            if (climb_phase == CLIMB_PHASE_DONE)
            {
                state = SYS_PATROL;
                climb_phase = CLIMB_PHASE_INIT;
            }
            //先不做上台，直接巡台
            //state = SYS_PATROL;
        }
        break;
 
        case SYS_PATROL:
        {
            patrol_behavior();

            if(sensor.on_stage == 0)
                state = SYS_OFF_STAGE;
        }
        break;

        case SYS_TRACK_QR:
        {
            edge_update_no_run();
            if(edge_is_busy()) 
				return;

            track_qr();

            if(sensor.on_stage == 0) 
                state = SYS_OFF_STAGE;
        }
        break;

        case SYS_ATTACK:
        {
            edge_update_no_run();
            if(edge_is_busy()) 
				return;

            attack_target();

            if(sensor.on_stage == 0)
                state = SYS_OFF_STAGE;
        }
        break;

        // case SYS_RECOVER:
        // {
        //     recover_behavior();

        //     if(recover_done)
        //     {
        //         state = SYS_OFF_STAGE;
        //         //state = SYS_CLIMB;   // 恢复后上台
        //     }
        // }
        // break;
    }
}
































































//规则控制版
// #define LOCK_CONFIRM_CNT   2       // 连续确认次数

// static void detect_target(void)
// {
//     static float last_dist[8] = {0};   // 滤波
//     static int last_dir = -1;          // 上一帧方向
//     static uint8_t lock_count = 0;     // 连续确认计数

//     float min_dist = 999.0f;
//     int new_dir = -1;

//     sensor.target_detected = 0;

//     /* ===== 1. 读取 + 滤波 + 找最近 ===== */
//     for(uint8_t ch = 0; ch < 8; ch++)
//     {
//         float dist = get_distance(ch);

//         /* 一阶滤波（抗抖） */
//         dist = 0.7f * last_dist[ch] + 0.3f * dist;
//         last_dist[ch] = dist;

//         if(dist < 40.0f)
//         {
//             if(dist < min_dist)
//             {
//                 min_dist = dist;
//                 new_dir = ch;
//             }
//         }
//     }

//     /* ===== 2. 连续帧确认 ===== */
//     if(new_dir != -1)
//     {
//         if(last_dir != -1 && abs(new_dir - last_dir) <= 1)
//         {
//             lock_count++;
//         }
//         else
//         {
//             lock_count = 0;
//         }

//         last_dir = new_dir;

//         if(lock_count >= LOCK_CONFIRM_CNT)
//         {
//             sensor.target_detected = 1;
//         }
//     }
//     else
//     {
//         lock_count = 0;
//         last_dir = -1;
//     }

//     /* ===== 3. 方向锁定（防跳变） ===== */
//     if(sensor.target_detected)
//     {
//         if(sensor.target_dir == -1)
//         {
//             /* 第一次锁定 */
//             sensor.target_dir = new_dir;
//         }
//         else
//         {
//             /* 只允许小范围变化（防左右跳） */
//             if(abs(new_dir - sensor.target_dir) <= 2)
//             {
//                 sensor.target_dir = new_dir;
//             }
//             /* 否则保持原方向 */
//         }
//     }
//     else
//     {
//         sensor.target_dir = -1;
//     }
// }

// //精止检测版
// static void detect_target(void)
// {
//     sensor.target_detected = 0;
//     sensor.target_dir = -1;

//     float min_dist = 999.0f;

//     for(uint8_t ch = 0; ch < 8; ch++)
//     {
//         float dist = get_distance(ch);

//         if(dist < 40.0f)
//         {
//             sensor.target_detected = 1;

//             if(dist < min_dist)
//             {
//                 min_dist = dist;
//                 sensor.target_dir = (int)ch;
//             }
//         }
//     }
// }



//规则控制板
// static void attack_target(void)
// {
//     static uint32_t last_seen_time = 0;
//     uint32_t now = HAL_GetTick();

//     /* ===== 目标丢失处理 ===== */
//     if(!sensor.target_detected)
//     {
//         /* 短时间丢失 → 继续追（防止抖动丢目标） */
//         if(now - last_seen_time < 200)
//         {
//             // 继续使用旧的 target_dir
//             // 但不更新 last_seen_time，保持追踪状态
//         }
//         else
//         {
//             /* 完全丢失 → 回巡逻 */
//             sensor.target_dir = -1;
//             state = SYS_PATROL;
//             return;
//         }
//     }
//     else
//     {
//         last_seen_time = now;
//     }

//     /* ===== 边缘优先 ===== */
//     edge_update();
//     if(edge_is_busy())
//     {
//         return;
//     }

//     /* ===== 中心检测（是否已对准） ===== */
//     float center_dist = get_distance(3);
//     float center_dist2 = get_distance(7);
//     float center = (center_dist < center_dist2) ? center_dist : center_dist2;

//     /* ===== 差速控制 ===== */
//     int base_speed;
//     int turn;

//     if(sensor.target_dir != -1)
//     {
//         /* 方向误差（3是正前） */
//         int error = sensor.target_dir - 3;

//         /* 根据是否对准决定速度 */
//         if(center < 40.0f)
//         {
//             base_speed = 30;   // 已对准 → 加速撞
//         }
//         else
//         {
//             base_speed = 20;   // 未对准 → 稳定追踪
//         }

//         /* 转向力度（可调） */
//         turn = error * 6;

//         /* 差速输出 */
//         int left_speed  = base_speed + turn;
//         int right_speed = base_speed - turn;
//         left_speed  = clamp_int(left_speed,  -30, 30);
//         right_speed = clamp_int(right_speed, -30, 30);

//         run_diff(left_speed, right_speed);
//     }
//     else
//     {
//         /* 理论不会进这里，保险处理 */
//         run(go, 20);
//     }
// }

//静止检测版
// static void attack_target(void)
// {
//     if(!sensor.target_detected)
//     {
//         state = SYS_PATROL;
//         return;
//     }

//     float center_dist = get_distance(1);
//     if(sensor.target_dir == 3 || sensor.target_dir == 5 || sensor.target_dir == 7)
//     {
//         // 左侧目标，左转直到 1 号传感器检测到敌人
//         if(center_dist < 40.0f)
//         {
//             run(go, 20);
//         }
//         else
//         {
//             run(left, 15);
//         }
//     }
//     else if(sensor.target_dir == 0 || sensor.target_dir == 4 || sensor.target_dir == 6)
//     {
//         // 右侧目标，右转直到 1 号传感器检测到敌人
//         if(center_dist < 40.0f)
//         {
//             run(go, 20);
//         }
//         else
//         {
//             run(right, 15);
//         }
//     }
//     else if(sensor.target_dir == 1)
//     {
//         // 中间或接近中间目标，直接前冲
//         run(go, 20);
//     }
//     else if(sensor.target_dir == 2)
//     {
//         // 后方目标，掉头
//         if(center_dist < 40.0f)
//         {
//             run(go, 20);
//         }
//         else
//         {
//             run(right, 15);
//         }
//     }
// }