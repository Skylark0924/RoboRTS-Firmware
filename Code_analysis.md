### chassis.h

1. `struct chassis_acc`：底盘加速度

2. `struct chassis`：
   
   >   struct object parent;
   >   struct mecanum mecanum;
   >   struct chassis_acc acc;
   >   struct motor_device motor[4];
   >   struct pid motor_pid[4];
   >   struct pid_feedback motor_feedback[4];
   >   struct controller ctrl[4];
   
3. `struct chassis_info`：底盘速度、位置、角度轮速等信息



### chassis.c

1. `int32_t chassis_pid_register(struct chassis *chassis, const char *name, enum device_can can)`：底盘pid参数、电机信息、麦轮信息等初始化

2. `int32_t chassis_execute(struct chassis *chassis)`：底盘运行

3. `int32_t chassis_gyro_updata(struct chassis *chassis, float yaw_angle, float yaw_rate)`：从陀螺仪更新角度和速度数据并存入底盘结构体的麦轮陀螺仪信息中

4. `int32_t chassis_set_speed(struct chassis *chassis, float vx, float vy, float vw)`：设置chassis->mecanum.speed(vx, vy, vw)

5. `int32_t chassis_set_acc(struct chassis *chassis, float ax, float ay, float wz)`：设置底盘加速度，chassis->acc

6. `int32_t chassis_set_vw(struct chassis *chassis, float vw)`：单独设置底盘麦轮自转速度（未使用）

7. `int32_t chassis_set_vx_vy(struct chassis *chassis, float vx, float vy)`：仅设置底盘x, y方向速度（未使用）

8. `int32_t chassis_set_offset(struct chassis *chassis, float offset_x, float offset_y)`：设置x, y轴底盘中心

9. `int32_t chassis_get_info(struct chassis *chassis, struct chassis_info *info)`：获取底盘参数存入`struct chassis_info`

10. `chassis_t chassis_find(const char *name)`：调用`object_find`寻找底盘，判断其是否连接

11. `int32_t chassis_enable(struct chassis *chassis)`：底盘使能开启

    `int32_t chassis_disable(struct chassis *chassis)`：底盘使能关闭

12. `static int32_t motor_pid_input_convert(struct controller *ctrl, void *input)`：将电机的速度反馈作为pid的反馈值，即此处使用速度pid



### shoot.h

1. `struct shoot_param`：射击参数

2. `struct shoot_target`：射击次数、两个摩擦轮速度、电机速度

3. `struct shoot`：
   >   struct object parent;
   >   struct shoot_param param;
   >   enum shoot_state state;
   >   uint8_t cmd;
   >   uint8_t trigger_key;
   >   uint16_t fric_spd[2];
   >   uint32_t shoot_num;
   >   uint32_t block_time;
   >   struct shoot_target target;
   >   struct motor_device motor;
   >   struct pid motor_pid;
   >   struct pid_feedback motor_feedback;
   >   struct controller ctrl;

### shoot.c

1. `int32_t shoot_pid_register(struct shoot *shoot, const char *name, enum device_can can)`：射击pid参数、电机信息、摩擦轮转速、卡弹转速等参数初始化

2. `int32_t shoot_set_fric_speed(struct shoot *shoot, uint16_t fric_spd1, uint16_t fric_spd2)`：设置摩擦轮速度

3. `int32_t shoot_get_fric_speed(struct shoot *shoot, uint16_t *fric_spd1, uint16_t *fric_spd2)`：获取摩擦轮速度

4. `int32_t shoot_set_cmd(struct shoot *shoot, uint8_t cmd, uint32_t shoot_num)`：设置射击模式\(SHOOT_ONCE_CMD、SHOOT_CONTINUOUS_CMD、SHOOT_STOP_CMD)

5. `int32_t shoot_execute(struct shoot *shoot)`：射击运行

   - 

6. `int32_t shoot_state_update(struct shoot *shoot)`：射击状态更新，检查是否触发射击

7. `int32_t shoot_set_turn_speed(struct shoot *shoot, uint16_t speed)`：设置射击频率

8. `shoot_t shoot_find(const char *name)`：调用`object_find`寻找射击部分，判断其是否连接

9. `int32_t shoot_enable(struct shoot *shoot)`：射击使能开启

   `int32_t shoot_disable(struct shoot *shoot)`：射击使能关闭

10. `static int32_t shoot_cmd_ctrl(struct shoot *shoot)`：射击各模式下的工作状态（初始化、单发、连发、停止、

