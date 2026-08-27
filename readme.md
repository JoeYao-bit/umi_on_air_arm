控制关节角的机器人末端轨迹追踪任务

轨迹生成找王子荣
关节角度指令执行找张武松

臂所在方向为正前方

张武松所使用的系统为ubuntu 22.04，c++代码


结果可视化

基于git分享代码

readme介绍基本原理

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

部署需要更新子模块安装依赖项

agx01

source工作空间

source /home/agx01/umi_arm_yz_ws/install/setup.bash


安装g2o

sudo apt install libeigen3-dev cmake libsparsehash-dev libsuitesparse-dev libqglviewer-dev-qt5 qtbase5-dev libqt5opengl5-dev git


