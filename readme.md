控制关节角的机器人末端轨迹追踪任务

轨迹生成找王子荣
关节角度指令执行找张武松

编译全部包

colcon build

编译单个包

colcon build --packages-select umi_on_air_arm #编译求逆解算法包
colcon build --packages-select umi_arm_msg #编译求逆解消息包


1,单元测试，验证算法
cd arm_ws/ #进入工作空间

ros2 run umi_on_air_arm main_demo

鼠标点击左键设置目标位置（空心圆圈），不要超出可达范围（黄色圆圈），即可看到连杆位置更新，正常情况下末端应与空星圆圈重合

2,启动节点

ros2 run umi_on_air_arm arm_ik_node

发送模拟数据到话题，验证流程

ros2 topic pub -1 /arm_ik_in umi_arm_msg/msg/IKInput "{q0_init: 1.57, q1_init: 0.0, q2_init: 0.0, x_des: 0.695, y_des: 0.0, z_des: 0.20}"

预期节点打印输出：
[INFO] [1787751358.314103769] [arm_ik_topic_node]: ik receve init joint angle[1.570, 0.000, 0.000], targe position[0.695, 0.000, 0.200]
reach goal position
[INFO] [1787751358.314920459] [arm_ik_topic_node]: ik ret_code=0 residual=0.00066 | joint_angle:[1.017, -1.162, -0.468]

注意：通过梯度下降的思路找逆解，因此目标位置不应偏离初始位置太远，不然容易找逆解失败

代码仓库：

正逆运动学算法：git@github.com:JoeYao-bit/umi_on_air_arm.git

消息类型：git@github.com:JoeYao-bit/umi_arm_msg.git


启动节点
ros2 run umi_on_air_arm arm_ik_node


另开终端输入测逆运动学，单次

ros2 topic pub /arm_ik_in umi_arm_msg/msg/IKInput "{q0_init: 0.1, q1_init: 0.2, q2_init: 0.15, x_des: 0.35, y_des: 0.0, z_des: 0.25, quaternion: {x: 0.0, y: 0.0998334, z: 0.0, w: 0.995004}}" --once

另开终端输入测正运动学，单次

ros2 topic pub /arm_fk_in umi_arm_msg/msg/FKInput "{q0:0.1, q1:0.2, q2:0.15, q3:0.0}" --once

2026.8.29 和张武松正逆运动学测试通过

2026.9.2

发布单次关节角消息,测试用

ros2 topic pub --once /joint_angles umi_arm_msg/msg/JointAngles "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, q0: 0.0, q1: 1.0, q2: 2.0, q3: 3.0}"

测试通过

发布单次关节角控制完成

ros2 topic pub /angle_control_feedback umi_arm_msg/msg/AngleControlFeedBack "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, error_code: 0}" --once

测试通过

发布单次末端轨迹与接收

ros2 topic pub /end_track umi_arm_msg/msg/EndTrack "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'world'}, poses: [{position: {x:0.2, y:0.3, z:0.5}, orientation: {x:0.0, y:0.0, z:0.0, w:1.0}}, {position: {x:0.4, y:0.1, z:0.5}, orientation: {x:0.0, y:0.0, z:0.0, w:1.0}}]}" --once

测试通过

上一个轨迹是为验证流程的虚假轨迹，机械臂不一定能执行。
换成

ros2 topic pub /end_track umi_arm_msg/msg/EndTrack "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'world'}, poses: [
{position: {x: 0.52, y: 0.0, z: 0.3825}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}},
{position: {x: 0.595, y: 0.0, z: 0.3575}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}},
{position: {x: 0.6575, y: 0.0, z: 0.325}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}},
{position: {x: 0.595, y: 0.0, z: 0.3575}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}},
{position: {x: 0.52, y: 0.0, z: 0.3825}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}
]}" --once

机械臂能执行

工作流程：

1,启动本节点

2,接收到目标轨迹，立即下发第一个位姿对应的关节角。收到机械臂反馈完成后再继续前往下一个末端位姿。
目标位姿后，立即下发第一个位姿对应的关节角。

3,若机械臂反馈异常，即错误码不为0,则终止轨迹跟踪任务，目标轨迹以及当前任务清零。

4,如果追踪轨迹时求逆解失败，则终止任务并移除目标轨迹和当前进度。

通信协议
全部信息收发通过消息-话题机制实现。
相关代码位于umi_on_air_arm/src/IK_node.cpp
消息文件定义位于umi_arm_msg/msg文件夹
|话题| 消息类型| 含义|
|----|----|----|
|/end_track | EndTrack|目标机械臂末端轨迹，由多个位姿组成，位姿之间应尽可能接近，使得关节角平稳变化 |
|/angle_control_feedback|AngleControlFeedBack|反馈机械臂是否执行到下发关节角，如果失败则包括失败原因|
|/joint_angles|JointAngles|机械臂传给轨迹跟踪节点的当前关节角，逆运动学解会尽量靠近当前关节角|
|/arm_fk_in|FKInput|机械臂正运动学输入，包含四个关节角|
|/arm_fk_out|FKOutput|机械臂正运动学输出，输出一个末端位姿|
|/arm_ik_in|IKInput|机械臂逆运动学输入，包含一个末端位姿|
|/arm_ik_out|IKOutput|机械臂逆运动学输出，并控制机械臂到逆解对应的关节角|

示教轨迹：
往前伸再缩回去

x,z,pitch,roll

0.52, 0.3825，0,0

0.595, 0.3575，0,0

0.6575, 0.325，0, 0

0.595, 0.3575，0,0

0.52, 0.3825，0,0

@张武松提供的机械臂控制接口
### 使用说明
#### 1、启动方式
```
python3 /home/agx01/zws_ws/DynamixelSDK-4.0.5/python/tests/motor_test/fast_sync_read_ros.py
```
启动后记录【启动位置】，运行到【零位】，此时正逆运动学相关逻辑生效，按ESC或CTRLl+C，恢复到【启动位置】
#### 2、关节软限位
关节限位，单位rad
 [
    [-0.8477, 1.3004],   # q0
    [-1.7683, 1.1469],   # q1
    [-1.9218, 1.6072],   # q2
    [-1.4615, 1.6072]    # q3
]
#### 3、修改关节速度上限
```
#速度默认为10，可修改范围：0 ~ 32767，0代表最大
#定义：value*0.229 [rev/min]

ros2 topic pub --once /motor_set_profile_velocity std_msgs/msg/Int32 '{data: 30}'
```
