#include "camera.h"
#include "stm32_hal_legacy.h"
#include "stm32f427xx.h"
#include <string.h>
#include <stdlib.h>

#define RX_BUF_SIZE 256
uint8_t rx_buf[RX_BUF_SIZE];

CameraInfo camera_info = {0};
static uint8_t camera_frame_buf[RX_BUF_SIZE];
static uint16_t camera_frame_len = 0;

uint16_t old_pos = 0;

static void camera_process_frame(uint8_t* frame, uint16_t len)
{
    if(frame == NULL || len < 3)
        return;

    if(frame[0] != '$' || frame[len - 1] != '#')
        return;

    char tmp[RX_BUF_SIZE];
    uint16_t payload_len = len - 2;
    if(payload_len >= sizeof(tmp))
        return;

    memcpy(tmp, frame + 1, payload_len);
    tmp[payload_len] = '\0';

    int values[10] = {0};
    int count = 0;
    char* token = strtok(tmp, ",");
    while(token != NULL && count < (int)(sizeof(values) / sizeof(values[0])))
    {
        values[count++] = (int)strtol(token, NULL, 10);
        token = strtok(NULL, ",");
    }

    if(count >= 6)
    {
        camera_info.x = values[2];
        camera_info.w = values[4];
        camera_info.id = values[6];
        camera_info.valid = 1;
        camera_info.last_update = HAL_GetTick();
    }
    else
    {
        camera_info.valid = 0;
    }
}

static void camera_parse_buffer(uint8_t* buf, uint16_t len)
{
    for(uint16_t i = 0; i < len; i++)
    {
        uint8_t ch = buf[i];

        if(camera_frame_len == 0)
        {
            if(ch == '$')
            {
                camera_frame_buf[camera_frame_len++] = ch;
            }
        }
        else
        {
            if(camera_frame_len < sizeof(camera_frame_buf))
            {
                camera_frame_buf[camera_frame_len++] = ch;
            }
            else
            {
                camera_frame_len = 0;
                continue;
            }

            if(ch == '#')
            {
                camera_process_frame(camera_frame_buf, camera_frame_len);
                camera_frame_len = 0;
            }
            else if(ch == '$')
            {
                camera_frame_len = 1;
                camera_frame_buf[0] = '$';
            }
        }
    }
}

void UART3_Init(void)
{
    HAL_UART_Receive_DMA(&huart3, rx_buf, RX_BUF_SIZE);
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
}

void Camera_UART3_IRQHandler(void)
{
     if(__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE))
    {
        const char debug_msg[] = "USART3 IDLE\r\n";
        HAL_UART_Transmit(&huart1, (uint8_t*)debug_msg, sizeof(debug_msg) - 1, 100);

        __HAL_UART_CLEAR_IDLEFLAG(&huart3);

        uint16_t new_pos = RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart3.hdmarx);

        if(new_pos != old_pos)
        {
            if(new_pos > old_pos)
            {
                // 正常情况
                UART3_RX_Callback(&rx_buf[old_pos], new_pos - old_pos);
            }
            else
            {
                // 环形缓冲区回绕
                UART3_RX_Callback(&rx_buf[old_pos], RX_BUF_SIZE - old_pos);
                if(new_pos > 0)
                {
                    UART3_RX_Callback(&rx_buf[0], new_pos);
                }
            }

            old_pos = new_pos;
        }
    }

    HAL_UART_IRQHandler(&huart3);
}

void UART3_RX_Callback(uint8_t* buf, uint16_t len)
{
    printf("len = %d\r\n", len);
    HAL_UART_Transmit(&huart1, buf, len, 100);
    HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 2, 100);

    camera_parse_buffer(buf, len);

    for(int i = 0; i < len; i++)
    {
        printf("%02X ", buf[i]);
    }
    printf("\r\n");
}

void Camera_GetInfo(CameraInfo* info)
{
    if(info != NULL)
    {
        CameraInfo tmp = camera_info;
        if(tmp.valid && HAL_GetTick() - tmp.last_update > 100)
        {
            tmp.valid = 0;
        }
        *info = tmp;
    }
}
