#ifndef _CAMERA_H_
#define _CAMERA_H_

#include "usart.h"
#include "stdio.h"
#include <stdint.h>

typedef struct
{
    int x;
    int w;
    int id;
    uint8_t valid;
    uint32_t last_update;
} CameraInfo;

extern CameraInfo camera_info;

void UART3_Init(void);
void Camera_UART3_IRQHandler(void);
void UART3_RX_Callback(uint8_t* buf, uint16_t len);
void Camera_GetInfo(CameraInfo* info);

#endif