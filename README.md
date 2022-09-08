# robot_arm_control_pkg  
Use ROS noetic to control the Robot-Arm which is controlled by the microcontroller(STM32).  
And I use Rviz to visualize the motion.  

---

## Ros topic

### In STM32
- geometry_msgs::Point / RobotArmPoint
- sensor_msgs::JointState / RobotArmJointState
- std_msgs::Bool / StowageState

### In Rpi
- sensor_msgs::JointState / RobotArmControl_Vel
- sensor_msgs::JointState / joint_states

## 主要概念

為了簡化跟 STM32 的溝通方式，原則上是透過 RobotArmJointState 來進行控制。
把 正向運動學 / 逆向運動學 的計算移到 RPI，因為是用 CPP 寫的，我覺得不會有效力不足的問題，但這需要實驗確認。

速度控制主要是利用追點來完成，控制的單位皆為標準單位。
至於會不會切西瓜的問題要等實測才能確定，目前用 rviz 看起來是完美 ～

## command
開啟 rviz 模擬 ：
roslaunch robot_arm_control Show_Robot_Arm.launch
