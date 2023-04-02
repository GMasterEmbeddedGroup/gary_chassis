# gary_chassis
gary_chassis包括了遥控器控制（chassis_teleop），底盘控制器解算（麦轮mecanum_chassis_solver，全向轮omni_chassis_solver）及发送设定值到PID和功率控制（chassis_powercontrol）等功能.并且在底盘控制器解算中集成了里程计发布。


## chassis_teleop
chassis_teleop节点主要负责将遥控器接收的数据转化为模式切换或速度（该部分使用了一节滤波函数）并且包括了底盘跟随云台以及小陀螺的坐标系转换解算。
### usage
使用生命周期节点的相关指令开启或关闭该节点。也可以在gary_bringup里更改参数

`chassis_teleop`可配置参数如下
* twist_pub_topic:用于发布x，y，z速度的话题，默认为"/cmd_vel"
* remote_control_topic：用于接收遥控器数据的话题，默认为"/remote_control"
* joint_topic：用于接收yaw电机数据的话题，默认为"/dynamic_joint_states"
* x_max_speed: 横向移动最大速度，默认为2
* y_max_speed:  纵向移动最大速度，默认为2
* rotate_max_speed: 旋转最大速度，默认为1
* x_max_accel: 滤波参数, 默认为1
* y_max_accel: 滤波参数, 默认为1
* use_break: 滤波清空状态机, 默认为true
* update_rate: 节点信息更新频率, 默认为200
* yaw_encoder_bias: 云台朝向底盘正前方时yaw电机编码器换算后的值，目前为1.960427

## mecanum_chassis_solver or omni_chassis_solver
* mecanum_chassis_solver or omni_chassis_solver是两种轮子的解算器。根据chassis_teleop种发布的twist消息，将vx，vy，az速度通过utils中的解算公式解算为四个轮子的转速设定值并发布。
* 同时，该节点也包括了获取四个轮子的速度并进行里程计的计算，通过时间计算粗略的机器人相对于起始位置坐标系的位置。

### usage
使用生命周期节点的相关指令开启或关闭该节点。也可以在gary_bringup里更改参数

`(底盘类型)_chassis_solver`可配置参数如下
* cmd_topic:用于接收twist消息的话题，默认为"~/cmd_vel"
* joint_topic：用于接收yaw电机数据的话题，默认为"/dynamic_joint_states"
* diagnostic_topic：用于生命周期节点是否正常运行的检测，默认为"/diagnostics_agg"
* odom_topic：用于发布里程计信息的话题，默认为“/odom”
* output_(轮子)_topic: 发布四个轮子pid设定值的话题
* motor_（轮子）_hw_id:  四个轮子是否掉线标志
* a,b,r: 麦轮/全向轮解算公式参数, 电机间距与轮子半径

## utils
### (底盘类型)_kinematics
底盘解算公式包含用于逆解算轮子转速用于pid设定值以及正解算用于里程计信息计算。

### first_order_filter
用于遥控器输入控制，使得加速更见平缓。