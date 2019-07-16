/**
  ******************************************************************************
  * File Name      	: PowerLimitationTask.c
  * Description    	: 底盘功率限制算法实现
  * Author			:
  ******************************************************************************
  *
  * Copyright (c) 2018 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#include "powerLimitation_task.h"
#include "JudgeTask.h"
#include "chassis.h"
#include "Cap2ControlTask.h"
#include "pid.h"

float SpeedAttenuation = 1.0f;
float LimitFactor = 1.0f;
uint8_t flag = 1;

pid_t PowerLimitationPID = POWER_LIMITATION_PID_DEFAULT;


//jxy
//new chassis power limitation
#define CHASSIS_PID fw_PID_INIT(2.0,0.5,0.5, 15000.0, 15000.0, 15000.0, 16384, 0)

#define CONST_ENERGY_BUFFER_THRESHOLD 30 // 
#define CONST_READ_BUFFER_PERIOD 0.05 //reading referee buffer is very unstable, from 50hz to 20hz
#define CONST_VOLTAGE 24.0
#define CONST_MAX_ITERATION_TIMES 12
#define COEF_SPEED_INCREMENT 1.1
#define COEF_SPEED_DECREMENT 0.95
#define COEF_CURRENT 20.0/16384.0
uint16_t power_upperbound = 80;
uint8_t broke = 0; //for debug
uint8_t iteration_time = 0;
void Calc_Power_UpperBound(void)
{
    //ToDo read buffer fail safe
    uint16_t present_energy_buffer = PowerHeat.chassis_power_buffer;
    if(present_energy_buffer > CONST_ENERGY_BUFFER_THRESHOLD)
    {
        power_upperbound = (present_energy_buffer - CONST_ENERGY_BUFFER_THRESHOLD) / CONST_READ_BUFFER_PERIOD;
    } 
    else
    {
        power_upperbound = 80;
    }
}

extern pid_t CHASSIS_FOLLOW_PID;

struct wheel_rotate_speed Limit_Chassis_Power_new(int16_t fb_ref, int16_t lr_ref, float GapAngle)
{
		iteration_time = 0;
		broke = 0;//for debug
		struct wheel_rotate_speed wheel_speed_tmp;
	int16_t rotate_ref;
	
	gimbal_t pgimbal = NULL;
	pgimbal = gimbal_find("gimbal");
	chassis_t pchassis = NULL;
  pchassis = chassis_find("chassis");
	
	//wheel_speed_tmp = calc_wheel_rpm(fb_ref, lr_ref, GapAngle);
	rotate_ref = GapAngle;
	float gap_angle_radian = pgimbal->ecd_angle.yaw / RADIAN_COEF;
	int16_t vw = PID_PROCESS_Single(&(CHASSIS_FOLLOW_PID),0, rotate_ref);
	static float rotate_ratio_f = ((WHEELBASE+WHEELTRACK)/2-GIMBAL_Y_OFFSET)/RADIAN_COEF;
	static float rotate_ratio_b = ((WHEELBASE+WHEELTRACK)/2+GIMBAL_Y_OFFSET)/RADIAN_COEF; 
	static float wheel_rpm_ratio = 60.0f*CHASSIS_DECELE_RATIO/(PI*WHEEL_DIAMETER);//rpm//
	float max = 0;
	
	VAL_LIMIT(lr_ref,-MAX_CHASSIS_VX_SPEED,MAX_CHASSIS_VX_SPEED);//mm/s
	VAL_LIMIT(fb_ref,-MAX_CHASSIS_VY_SPEED,MAX_CHASSIS_VY_SPEED);//mm/s
	VAL_LIMIT(vw,-MAX_CHASSIS_VW_SPEED,MAX_CHASSIS_VW_SPEED);//degree/s
	float cos_sub_sin = cos(gap_angle_radian)-sin(gap_angle_radian);
	float cos_add_sin = cos(gap_angle_radian)+sin(gap_angle_radian);
	wheel_speed_tmp.wheel_rpm[0] = (-lr_ref*cos_sub_sin-fb_ref*cos_add_sin+vw*rotate_ratio_f)*wheel_rpm_ratio;
	wheel_speed_tmp.wheel_rpm[1] = (-lr_ref*cos_add_sin+fb_ref*cos_sub_sin+vw*rotate_ratio_f)*wheel_rpm_ratio;
	wheel_speed_tmp.wheel_rpm[2] = (lr_ref*cos_sub_sin+fb_ref*cos_add_sin+vw*rotate_ratio_b)*wheel_rpm_ratio;
	wheel_speed_tmp.wheel_rpm[3] = (lr_ref*cos_add_sin-fb_ref*cos_sub_sin+vw*rotate_ratio_b)*wheel_rpm_ratio;

	//find max item
	for(uint8_t i=0;i<4;i++)
	{
		if(abs(wheel_speed_tmp.wheel_rpm[i]) > max)
			max = abs(wheel_speed_tmp.wheel_rpm[i]);
	}
	//equal propotion
	if(max>MAX_RPM)
	{
		float rate=MAX_RPM/max;
		for(uint8_t i=0;i<4;i++) wheel_speed_tmp.wheel_rpm[i]*=rate;
	}
	
	int16_t CMFLIntensity = PID_PROCESS_Single(&(pchassis->motor_pid[0]), wheel_speed_tmp.wheel_rpm[1], pchassis->motor[0].data.speed_rpm);
	int16_t CMFRIntensity = PID_PROCESS_Single(&(pchassis->motor_pid[1]), wheel_speed_tmp.wheel_rpm[0], pchassis->motor[0].data.speed_rpm);
	int16_t CMBLIntensity = PID_PROCESS_Single(&(pchassis->motor_pid[2]), wheel_speed_tmp.wheel_rpm[2], pchassis->motor[0].data.speed_rpm);
	int16_t CMBRIntensity = PID_PROCESS_Single(&(pchassis->motor_pid[3]), wheel_speed_tmp.wheel_rpm[3], pchassis->motor[0].data.speed_rpm);

	uint16_t sum_intensity = __fabs(CMFLIntensity) + __fabs(CMFRIntensity) + __fabs(CMBLIntensity) + __fabs(CMBRIntensity);	

		while(sum_intensity * COEF_CURRENT * CONST_VOLTAGE >= power_upperbound)
		{
			iteration_time++;
			if(iteration_time <= CONST_MAX_ITERATION_TIMES)
			{
/*
				fb_ref*=COEF_SPEED_DECREMENT;
				lr_ref*=COEF_SPEED_DECREMENT;
				// wheel_speed_tmp = calc_wheel_rpm(fb_ref, lr_ref, GapAngle);
				MINMAX(lr_ref,-MAX_CHASSIS_VX_SPEED,MAX_CHASSIS_VX_SPEED);//mm/s
				MINMAX(fb_ref,-MAX_CHASSIS_VY_SPEED,MAX_CHASSIS_VY_SPEED);//mm/s
				MINMAX(vw,-MAX_CHASSIS_VW_SPEED,MAX_CHASSIS_VW_SPEED);//degree/s
				wheel_speed_tmp.wheel_rpm[0] = (-lr_ref*(cos(gap_angle_radian)-sin(gap_angle_radian))-fb_ref*(cos(gap_angle_radian)+sin(gap_angle_radian))+vw*rotate_ratio_f)*wheel_rpm_ratio; //??????
				wheel_speed_tmp.wheel_rpm[1] = (-lr_ref*(cos(gap_angle_radian)+sin(gap_angle_radian))+fb_ref*(cos(gap_angle_radian)-sin(gap_angle_radian))+vw*rotate_ratio_f)*wheel_rpm_ratio;
				wheel_speed_tmp.wheel_rpm[2] = (lr_ref*(cos(gap_angle_radian)-sin(gap_angle_radian))+fb_ref*(cos(gap_angle_radian)+sin(gap_angle_radian))+vw*rotate_ratio_b)*wheel_rpm_ratio;
				wheel_speed_tmp.wheel_rpm[3] = (lr_ref*(cos(gap_angle_radian)+sin(gap_angle_radian))-fb_ref*(cos(gap_angle_radian)-sin(gap_angle_radian))+vw*rotate_ratio_b)*wheel_rpm_ratio;
*/
				for(uint8_t i=0; i<4; i++)
				{
					wheel_speed_tmp.wheel_rpm[i] *= COEF_SPEED_DECREMENT;
				}
				
				CMFLIntensity = PID_PROCESS_Single(&(pchassis->motor_pid[0]), wheel_speed_tmp.wheel_rpm[0], pchassis->motor[0].data.speed_rpm);
				CMFRIntensity = PID_PROCESS_Single(&(pchassis->motor_pid[1]), wheel_speed_tmp.wheel_rpm[1], pchassis->motor[0].data.speed_rpm);
				CMBLIntensity = PID_PROCESS_Single(&(pchassis->motor_pid[2]), wheel_speed_tmp.wheel_rpm[2], pchassis->motor[0].data.speed_rpm);
				CMBRIntensity = PID_PROCESS_Single(&(pchassis->motor_pid[3]), wheel_speed_tmp.wheel_rpm[3], pchassis->motor[0].data.speed_rpm);
				sum_intensity = __fabs(CMFLIntensity) + __fabs(CMFRIntensity) + __fabs(CMBLIntensity) + __fabs(CMBRIntensity);
				Calc_Power_UpperBound();
			}
			else
			{
				//PowerLimitation();
				broke = 1;
				break;
			}
		}
		if(iteration_time > CONST_MAX_ITERATION_TIMES)
		{
			wheel_speed_tmp.wheel_rpm[0] = 9999;
		}
		return wheel_speed_tmp;
}

int while_test = 0;
void Limit_Chassis_Power(void)
{
	
		chassis_t pchassis = NULL;
    pchassis = chassis_find("chassis");
	  struct chassis_info info;
		chassis_get_info(pchassis, &info);
	
//    uint8_t iteration_time = 0;
		iteration_time = 0;
		broke = 0;
    uint16_t sum_intensity = 0;
		int16_t CMFLIntensity = pchassis->motor[0].current;
		int16_t CMFRIntensity = pchassis->motor[1].current;
		int16_t CMBLIntensity = pchassis->motor[2].current;
		int16_t CMBRIntensity = pchassis->motor[3].current;
    int16_t CMFLTargetSpeed = info.wheel_rpm[0];
    int16_t CMFRTargetSpeed = info.wheel_rpm[1];
    int16_t CMBLTargetSpeed = info.wheel_rpm[2];
    int16_t CMBRTargetSpeed = info.wheel_rpm[3];
    int16_t CMFLRealSpeed = pchassis->motor[0].data.speed_rpm;
    int16_t CMFRRealSpeed = pchassis->motor[0].data.speed_rpm;
    int16_t CMBLRealSpeed = pchassis->motor[0].data.speed_rpm;
    int16_t CMBRRealSpeed = pchassis->motor[0].data.speed_rpm;
    sum_intensity = __fabs(CMFLIntensity) + __fabs(CMFRIntensity) + __fabs(CMBLIntensity) + __fabs(CMBRIntensity);
    Calc_Power_UpperBound();
    while(sum_intensity * COEF_CURRENT * CONST_VOLTAGE >= power_upperbound)
    {		
			while_test = 1;
			iteration_time++;
			if(iteration_time <= CONST_MAX_ITERATION_TIMES)
			{
				//ToDo distinguish accelrate / decelerate
				CMFLTargetSpeed *= COEF_SPEED_DECREMENT;
				CMFRTargetSpeed *= COEF_SPEED_DECREMENT;
				CMBLTargetSpeed *= COEF_SPEED_DECREMENT;
				CMBRTargetSpeed *= COEF_SPEED_DECREMENT;
				CMFLRealSpeed = pchassis->motor[0].data.speed_rpm;
				CMFRRealSpeed = pchassis->motor[0].data.speed_rpm;
				CMBLRealSpeed = pchassis->motor[0].data.speed_rpm;
				CMBRRealSpeed = pchassis->motor[0].data.speed_rpm;
				CMFLIntensity = PID_PROCESS_Single(&(pchassis->motor_pid[1]), CMFLTargetSpeed, CMFLRealSpeed);
				CMFRIntensity = PID_PROCESS_Single(&(pchassis->motor_pid[2]), CMFRTargetSpeed, CMFRRealSpeed);
				CMBLIntensity = PID_PROCESS_Single(&(pchassis->motor_pid[3]), CMBLTargetSpeed, CMBLRealSpeed);
				CMBRIntensity = PID_PROCESS_Single(&(pchassis->motor_pid[4]), CMBRTargetSpeed, CMBRRealSpeed);
				sum_intensity = __fabs(CMFLIntensity) + __fabs(CMFRIntensity) + __fabs(CMBLIntensity) + __fabs(CMBRIntensity);
			}
			else
			{
				PowerLimitation();
				broke = 1;
				break;
			}
			Calc_Power_UpperBound();
    }
		if(iteration_time <= CONST_MAX_ITERATION_TIMES)
		{
			info.wheel_rpm[0] = CMFLTargetSpeed;
			info.wheel_rpm[0] = CMFRTargetSpeed;
			info.wheel_rpm[0] = CMBLTargetSpeed;
			info.wheel_rpm[0] = CMBRTargetSpeed;
//			CMFL.Intensity = CMFLIntensity;
//			CMFR.Intensity = CMFRIntensity;
//			CMBL.Intensity = CMBLIntensity;
//			CMBR.Intensity = CMBRIntensity;
		}
		while_test=0;
		
}
//END new chassis power limitation 

//底盘功率限制
void PowerLimitation(void)
{
	chassis_t pchassis = NULL;
  pchassis = chassis_find("chassis");
	
	uint16_t sum = 0;
	int16_t CM_current_max;
	int16_t CMFLIntensity = pchassis->motor[0].current;
	int16_t CMFRIntensity = pchassis->motor[0].current;
	int16_t CMBLIntensity = pchassis->motor[0].current;
	int16_t CMBRIntensity = pchassis->motor[0].current;
	
	sum = __fabs(CMFLIntensity) + __fabs(CMFRIntensity) + __fabs(CMBLIntensity) + __fabs(CMBRIntensity);
	//static int16_t FLILast,FRILast,BLILast,BRILast;
	//离线模式
	if (JUDGE_State == OFFLINE)
	{
		CM_current_max = 4000;
		if(sum > CM_current_max)
		{
			CMFLIntensity = (CMFLIntensity/(sum+1.0f))*CM_current_max;
			CMFRIntensity = (CMFRIntensity/(sum+1.0f))*CM_current_max;
			CMBLIntensity = (CMBLIntensity/(sum+1.0f))*CM_current_max;
			CMBRIntensity = (CMBRIntensity/(sum+1.0f))*CM_current_max;
		}
		//CM_current_max = CM_current_MAX;
	}
	
	//仿桂电策略
//	else if(PowerHeatData.chassisPowerBuffer-((PowerHeatData.chassisPower-80)>0?(PowerHeatData.chassisPower-80):0)*0.5f < 10.0f)
//	{
//		//CM_current_max = 2730;
//		float realPowerBuffer = PowerHeatData.chassisPowerBuffer;
	else if(PowerHeat.chassis_power_buffer-((PowerHeat.chassis_power-80)>0?(PowerHeat.chassis_power-80):0)*1.0f < 20.0f)
	{//ToDo float(PowerHeat.chassis_power_buffer)
		//CM_current_max = 2730;
		float realPowerBuffer = PowerHeat.chassis_power_buffer;
		//float realPower = PowerHeatData.chassisPower;
		//PowerLimitationPID.feedback = realPower;
		//PowerLimitationPID.target = 70;
		//PowerLimitationPID.Calc(&PowerLimitationPID);
		//CM_current_max = PowerLimitationPID.output;
		//LimitFactor += PowerLimitationPID.output/sum;
		//if(CM_current_max > 0.5) CM_current_max = 0;
		//if(CM_current_max < -sum) CM_current_max = -sum;
		//LimitFactor = 50/(((realPower-80)>0?(realPower-80):0)+1) * pow((realPowerBuffer),2);
		
		if(realPowerBuffer < 0) realPowerBuffer = 0;
		LimitFactor = 2500 + 192*pow((realPowerBuffer),1);
//		LimitFactor = 3200+320*realPowerBuffer;
		
		if(LimitFactor > sum) LimitFactor = sum;
		CMFLIntensity *= LimitFactor/sum;
		CMFRIntensity *= LimitFactor/sum;
		CMBLIntensity *= LimitFactor/sum;
		CMBRIntensity *= LimitFactor/sum;
	}
	//else if (Control_SuperCap.release_power==0 || PowerHeatData.chassisPower>30)
	else if(Cap_Get_Cap_State()!=CAP_STATE_RELEASE||Cap_Get_Cap_Voltage()<11)
	{
		//PowerLimitationPID.Reset(&PowerLimitationPID);
		//LimitFactor = 1.0f;
		CM_current_max = 11000;
		if(sum > CM_current_max){
			CMFLIntensity = (CMFLIntensity/(sum+0.0f))*CM_current_max;
			CMFRIntensity = (CMFRIntensity/(sum+0.0f))*CM_current_max;
			CMBLIntensity = (CMBLIntensity/(sum+0.0f))*CM_current_max;
			CMBRIntensity = (CMBRIntensity/(sum+0.0f))*CM_current_max;
		}
	}
//	else if(sum>11000)
//	{
//	  FLILast=(CMFLIntensity>0?1:-1)*abs(FLILast);
//	  FRILast=(CMFRIntensity>0?1:-1)*abs(FRILast);
//	  BLILast=(CMBLIntensity>0?1:-1)*abs(BLILast);
//	  BRILast=(CMBRIntensity>0?1:-1)*abs(BRILast);
//		if(abs(CMFLIntensity-FLILast)>2000)
//		{
//			CMFLIntensity=FLILast+(CMFLIntensity-FLILast)*0.01;
//			CMFRIntensity=FRILast+(CMFRIntensity-FRILast)*0.01;
//			CMBLIntensity=BLILast+(CMBLIntensity-BLILast)*0.01;
//			CMBRIntensity=BRILast+(CMBRIntensity-BRILast)*0.01;
//		}
//		else if(abs(CMFLIntensity-FLILast)>1000)
//		{
//			CMFLIntensity=FLILast+(CMFLIntensity-FLILast)*0.02;
//			CMFRIntensity=FRILast+(CMFRIntensity-FRILast)*0.02;
//			CMBLIntensity=BLILast+(CMBLIntensity-BLILast)*0.02;
//			CMBRIntensity=BRILast+(CMBRIntensity-BRILast)*0.02;
//		}
//	}
//	FLILast=CMFLIntensity;
//	FRILast=CMFRIntensity;
//	BLILast=CMBLIntensity;
//	BRILast=CMBRIntensity;
	
	pchassis->motor[0].current = CMFLIntensity;
	pchassis->motor[0].current = CMFRIntensity;
	pchassis->motor[0].current = CMBLIntensity;
	pchassis->motor[0].current = CMBRIntensity;
}

//用于常态的基于自检测功率的功率限制
void CurBased_PowerLimitation(void)
{
	chassis_t pchassis = NULL;
  pchassis = chassis_find("chassis");
	
	int32_t sum = 0;
	int32_t CM_current_max;
	int32_t CMFLIntensity = pchassis->motor[0].current;
	int32_t CMFRIntensity = pchassis->motor[0].current;
	int32_t CMBLIntensity = pchassis->motor[0].current;
	int32_t CMBRIntensity = pchassis->motor[0].current;
	//离线模式
	if (JUDGE_State == OFFLINE)
	{
		CM_current_max = 4000;
		sum = __fabs(CMFLIntensity) + __fabs(CMFRIntensity) + __fabs(CMBLIntensity) + __fabs(CMBRIntensity);
		if(sum > CM_current_max)
		{
			CMFLIntensity = (CMFLIntensity/(sum+1.0f))*CM_current_max;
			CMFRIntensity = (CMFRIntensity/(sum+1.0f))*CM_current_max;
			CMBLIntensity = (CMBLIntensity/(sum+1.0f))*CM_current_max;
			CMBRIntensity = (CMBRIntensity/(sum+1.0f))*CM_current_max;
		}
		//CM_current_max = CM_current_MAX;
	}
	
	//仿桂电策略
	else if((PowerHeat.chassis_power_buffer-((Cap_Get_Power_CURR()*Cap_Get_Power_Voltage()-80)>0?(Cap_Get_Power_CURR()*Cap_Get_Power_Voltage()-80):0)*1.0f < 40.0f))
	{
		//CM_current_max = 2730;
		sum = __fabs(CMFLIntensity) + __fabs(CMFRIntensity) + __fabs(CMBLIntensity) + __fabs(CMBRIntensity);
		float realPowerBuffer = PowerHeat.chassis_power_buffer;
		float realPower = PowerHeat.chassis_power;
		PowerLimitationPID.feedback = realPower;
		PowerLimitationPID.target = 75;
		PowerLimitationPID.Calc(&PowerLimitationPID);
		CM_current_max = PowerLimitationPID.output;
		//LimitFactor += PowerLimitationPID.output/sum;
		//if(CM_current_max > 0.5) CM_current_max = 0;
		//if(CM_current_max < -sum) CM_current_max = -sum;
		if(realPowerBuffer < 0) realPowerBuffer = 0;
		//LimitFactor = 50/(((realPower-80)>0?(realPower-80):0)+1) * pow((realPowerBuffer),2);
		LimitFactor = 2500 + 192*pow((realPowerBuffer),1);
		if(LimitFactor > sum) LimitFactor = sum;
		CMFLIntensity *= LimitFactor/sum;
		CMFRIntensity *= LimitFactor/sum;
		CMBLIntensity *= LimitFactor/sum;
		CMBRIntensity *= LimitFactor/sum;
	}
	pchassis->motor[0].current = CMFLIntensity;
	pchassis->motor[0].current = CMFRIntensity;
	pchassis->motor[0].current = CMBLIntensity;
	pchassis->motor[0].current = CMBRIntensity;
	rlease_flag = 0;
}

//用于放电模式下的基于自检测功率的功率限制
void CapBased_PowerLimitation(void)
{
	chassis_t pchassis = NULL;
  pchassis = chassis_find("chassis");
	
	int32_t sum = 0;
	int32_t CM_current_max;
	int32_t CMFLIntensity = pchassis->motor[0].current;
	int32_t CMFRIntensity = pchassis->motor[0].current;
	int32_t CMBLIntensity = pchassis->motor[0].current;
	int32_t CMBRIntensity = pchassis->motor[0].current;
	
	//离线模式
	if (JUDGE_State == OFFLINE)
	{
		CM_current_max = 4000;
		sum = __fabs(CMFLIntensity) + __fabs(CMFRIntensity) + __fabs(CMBLIntensity) + __fabs(CMBRIntensity);
		if(sum > CM_current_max)
		{
			CMFLIntensity = (CMFLIntensity/(sum+1.0f))*CM_current_max;
			CMFRIntensity = (CMFRIntensity/(sum+1.0f))*CM_current_max;
			CMBLIntensity = (CMBLIntensity/(sum+1.0f))*CM_current_max;
			CMBRIntensity = (CMBRIntensity/(sum+1.0f))*CM_current_max;
		}
		//CM_current_max = CM_current_MAX;
	}
	else if( (PowerHeat.chassis_power_buffer-((Cap_Get_Power_CURR()*Cap_Get_Power_Voltage()-70)>0?(Cap_Get_Power_CURR()*Cap_Get_Power_Voltage()-70):0)*1.0f < 50.0f))
	{
		//CM_current_max = 2730;
		sum = __fabs(CMFLIntensity) + __fabs(CMFRIntensity) + __fabs(CMBLIntensity) + __fabs(CMBRIntensity);
		float realPowerBuffer = PowerHeat.chassis_power_buffer;
		float realPower = PowerHeat.chassis_power;
		PowerLimitationPID = realPower;
		PowerLimitationPID.target = 75;
		PowerLimitationPID.Calc(&PowerLimitationPID);
		CM_current_max = PowerLimitationPID.output;
		//LimitFactor += PowerLimitationPID.output/sum;
		//if(CM_current_max > 0.5) CM_current_max = 0;
		//if(CM_current_max < -sum) CM_current_max = -sum;
		if(realPowerBuffer < 0) realPowerBuffer = 0;
		//LimitFactor = 50/(((realPower-80)>0?(realPower-80):0)+1) * pow((realPowerBuffer),2);
		LimitFactor = 2500 + 192*pow((realPowerBuffer),1);
		if(LimitFactor > sum) LimitFactor = sum;
		CMFLIntensity *= LimitFactor/sum;
		CMFRIntensity *= LimitFactor/sum;
		CMBLIntensity *= LimitFactor/sum;
		CMBRIntensity *= LimitFactor/sum;
	}
	else if(Cap_Get_Cap_Voltage() < 12 )
	{
		//CM_current_max = 2730;
		sum = __fabs(CMFLIntensity) + __fabs(CMFRIntensity) + __fabs(CMBLIntensity) + __fabs(CMBRIntensity);
		float realPowerBuffer = PowerHeat.chassis_power_buffer;
		float realPower = PowerHeat.chassis_power;
		PowerLimitationPID.feedback = realPower;
		PowerLimitationPID.target = 75;
		PowerLimitationPID.Calc(&PowerLimitationPID);
		CM_current_max = PowerLimitationPID.output;
		//LimitFactor += PowerLimitationPID.output/sum;
		//if(CM_current_max > 0.5) CM_current_max = 0;
		//if(CM_current_max < -sum) CM_current_max = -sum;
		if(realPowerBuffer < 0) realPowerBuffer = 0;
		//LimitFactor = 50/(((realPower-80)>0?(realPower-80):0)+1) * pow((realPowerBuffer),2);
		LimitFactor = 2500 + 192*pow((realPowerBuffer),1);
		if(LimitFactor > sum) LimitFactor = sum;
		CMFLIntensity *= LimitFactor/sum;
		CMFRIntensity *= LimitFactor/sum;
		CMBLIntensity *= LimitFactor/sum;
		CMBRIntensity *= LimitFactor/sum;
	}
	
	pchassis->motor[0].current = CMFLIntensity;
	pchassis->motor[0].current = CMFRIntensity;
	pchassis->motor[0].current = CMBLIntensity;
	pchassis->motor[0].current = CMBRIntensity;
	rlease_flag = 0;
}


