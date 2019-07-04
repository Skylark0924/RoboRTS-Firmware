/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#include "dbus.h"
#include "chassis_task.h"
#include "timer_task.h"
#include "infantry_cmd.h"
#include "param.h"

static float vx, vy, vw;

float follow_relative_angle;
struct pid pid_follow = {0}; //angle control

void chassis_task(void const *argument)
{
  uint32_t period = osKernelSysTick();
  chassis_t pchassis = NULL;
	gimbal_t ch_gimbal = NULL;
  rc_device_t prc_dev = NULL;
  rc_info_t prc_info = NULL;
  pchassis = chassis_find("chassis");
	ch_gimbal=gimbal_find("gimbal");
  prc_dev = rc_device_find("uart_rc");
	
	cali_sys_t *pparam = NULL;
  pparam = get_cali_param();

  if (pparam->gim_cali_data.calied_done == CALIED_FLAG)
  {
    gimbal_set_offset(ch_gimbal, pparam->gim_cali_data.yaw_offset, pparam->gim_cali_data.pitch_offset);
  }

  if (prc_dev != NULL)
  {
    prc_info = rc_device_get_info(prc_dev);
  }
  else
  {
  }

  soft_timer_register(chassis_push_info, (void *)pchassis, 10);

  pid_struct_init(&pid_follow, MAX_CHASSIS_VW_SPEED, 50, 8.0f, 0.0f, 2.0f);

  while (1)
  {
    if (rc_device_get_state(prc_dev, RC_S2_DOWN) != RM_OK)
    {
      if (rc_device_get_state(prc_dev, RC_S2_UP) == RM_OK)
      {
        vx = (float)prc_info->ch2 / 660 * MAX_CHASSIS_VX_SPEED;
        vy = -(float)prc_info->ch1 / 660 * MAX_CHASSIS_VY_SPEED;
				vw = 400;
//        wz = -pid_calculate(&pid_follow, follow_relative_angle, 0);
				chassis_set_offset(pchassis, 0, 0);
//        chassis_set_offset(pchassis, ROTATE_X_OFFSET, ROTATE_Y_OFFSET);
        chassis_set_speed(pchassis, vx, vy, vw);
      }

      if (rc_device_get_state(prc_dev, RC_S2_MID) == RM_OK)
      {
        vx = (float)prc_info->ch2 / 660 * MAX_CHASSIS_VX_SPEED;
        vy = -(float)prc_info->ch1 / 660 * MAX_CHASSIS_VY_SPEED;
        vw = -(float)prc_info->ch3 / 660 * MAX_CHASSIS_VW_SPEED;
        chassis_set_offset(pchassis, 0, 0);
        chassis_set_speed(pchassis, vx, vy, vw);
      }

      if (rc_device_get_state(prc_dev, RC_S2_MID2DOWN) == RM_OK)
      {
        chassis_set_speed(pchassis, 0, 0, 0);
      }

      if (rc_device_get_state(prc_dev, RC_S2_MID2UP) == RM_OK)
      {
        chassis_set_speed(pchassis, 0, 0, 0);
      }

      chassis_set_acc(pchassis, 0, 0, 0);
    }
		
    chassis_execute(pchassis, ch_gimbal);
    osDelayUntil(&period, 2);
  }
}

int32_t chassis_set_relative_angle(float angle)
{
  follow_relative_angle = angle;
  return 0;
}
