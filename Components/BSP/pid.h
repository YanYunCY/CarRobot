#ifndef _PID_H_
#define _PID_H_

#include <stdint.h>

void PID_Init(void);
void PID_Reset(void);

void PID_Camera_Reset(void);
int  PID_Camera_Compute(int error);

void PID_Target_Reset(void);
int  PID_Target_Compute(int error);

#endif