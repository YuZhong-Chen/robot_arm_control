# robot_arm_control_pkg  
Use ROS noetic to control the Robot-Arm which is controlled by the microcontroller(STM32).  
And I use Rviz to visualize the motion.  

---

## ROS topic

### In STM32
- geometry_msgs::Point / RobotArmPoint
- sensor_msgs::JointState / RobotArmJointState
- std_msgs::Bool / StowageState

### In RPI
- sensor_msgs::JointState / RobotArmControl_Vel
- sensor_msgs::JointState / joint_states

---

## 主要概念

為了簡化跟 STM32 的溝通方式，原則上是透過 RobotArmJointState 來進行控制。   
目前把 正向運動學 / 逆向運動學 的計算移到 RPI，因為是用 CPP 寫的，我相信不會有算力不足的問題，但這需要實驗確認（ 比較需要擔心速度控制時通訊的延遲 ）  

速度控制主要是利用追點來完成，控制的單位皆為標準單位。   
至於會不會切西瓜或是其他問題要等實測才能確定，目前用 rviz 看起來是完美 ～

---

## Run the code.
記得下載到 workspace 後要編譯。

- 控制位置 node 
  - rosrun robot_arm_control Arm_Position_Pub
    - show / s : 顯示現在資訊
    - quit / q : 結束 node 
    - (write / w) (ang / a) (關節編號) (角度) 
    - (write / w) (vel / v) (關節編號) (速度)
    - (write / w) (both / b) (關節編號) (角度) (速度)
    - (write / w) (pos / p) (x y z 座標)
    - (pub / p) (state / s) 
    - (pub / p) (point / p)
- 速度更新 node
  - rosrun robot_arm_control Arm_Velocity_Update
- 速度控制 node
  - rosrun robot_arm_control Arm_Velocity_Pub 
- Rviz 模擬 ：
  - roslaunch robot_arm_control Show_Robot_Arm.launch
  
 
