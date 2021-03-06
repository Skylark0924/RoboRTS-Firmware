/**
  ******************************************************************************
  * File Name          : PowerLimitation.h
  * Description        : 底盘功率限制算法头文件
  ******************************************************************************
  *
  * Copyright (c) 2018 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#ifndef __POWERLIMITATIONTASK_H
#define __POWERLIMITATIONTASK_H

#include "sys.h"

extern float SpeedAttenuation;
extern void PowerLimitation(void);
extern void CurBased_PowerLimitation(void);
extern void CapBased_PowerLimitation(void);
extern void SpeedLimitation(void);
extern void getRealSpeed(void);
extern uint8_t flag;
extern void Limit_Chassis_Power(void);
extern struct wheel_rotate_speed Limit_Chassis_Power_new(int16_t fb_ref, int16_t lr_ref, float GapAngle);
#define POWER_LIMITATION_PID_DEFAULT \
{\
	0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ,\
	3.0, 4.0, 0.0, \
	0.0, 0.0, 0.0, \
	10800, 10800, 10800,\
	0.0,\
	15500, 0,\
	{0,0}, \
	&fw_PID_Calc, &fw_PID_Reset, \
}
#endif /*__POWERLIMITATIONTASK_H */
