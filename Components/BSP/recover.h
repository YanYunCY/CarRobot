#ifndef __RECOVER_H
#define __RECOVER_H

#include "stdint.h"

void recover_init(void);
void recover_start(void);
void recover_update(void);
uint8_t recover_is_active(void);
uint8_t recover_is_done(void);
uint8_t recover_detect_off_stage(void);
void recover_blocking(void);

#endif
