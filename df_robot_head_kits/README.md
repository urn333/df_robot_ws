# df_robot_head_kits ROS2 Package

## 概述

这是一个用于控制仿生眼机器人头部的ROS2包，支持控制两个眼球和颈部的运动。

## 功能特性

- 订阅左眼、右眼和颈部的目标角度数据
- 发布左眼、右眼和颈部的当前角度数据
- 根据订阅的目标角度自动调整眼球和颈部的位置
- 支持本地控制模式（直连USB）
- 使用C++14开发，编译为ARM64版本

## 依赖

- ROS2 Foxy
- BionicEyes SDK
- rclcpp
- std_msgs

## 编译

```bash
cd /home/xiyz/dev_ws/BionicEyes
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --packages-select df_robot_head_kits
```

## 运行

```bash
cd /home/xiyz/dev_ws/BionicEyes
source install/setup.bash
ros2 launch df_robot_head_kits bionic_ctrl.launch.py
```

## 话题

### 订阅话题

- `/left_eye_target_angles` (std_msgs/Float32MultiArray): 左眼目标角度 [pitch, roll, yaw]
- `/right_eye_target_angles` (std_msgs/Float32MultiArray): 右眼目标角度 [pitch, roll, yaw]
- `/neck_target_angles` (std_msgs/Float32MultiArray): 颈部目标角度 [pan, tilt, roll]

### 发布话题

- `/left_eye_current_angles` (std_msgs/Float32MultiArray): 左眼当前角度 [pitch, roll, yaw]
- `/right_eye_current_angles` (std_msgs/Float32MultiArray): 右眼当前角度 [pitch, roll, yaw]
- `/neck_current_angles` (std_msgs/Float32MultiArray): 颈部当前角度 [pan, tilt, roll]

## 测试

```bash
# 测试左眼控制
ros2 topic pub /left_eye_target_angles std_msgs/Float32MultiArray "{data: [10.0, 0.0, 0.0]}"

# 测试右眼控制
ros2 topic pub /right_eye_target_angles std_msgs/Float32MultiArray "{data: [10.0, 0.0, 0.0]}"

# 测试颈部控制
ros2 topic pub /neck_target_angles std_msgs/Float32MultiArray "{data: [10.0, 5.0, 0.0]}"

# 查看当前角度
ros2 topic echo /left_eye_current_angles
ros2 topic echo /right_eye_current_angles
ros2 topic echo /neck_current_angles
```

## 注意事项

1. 确保设备已正确连接并通过USB访问
2. 确保BionicEyes SDK库文件在正确的位置
3. 颈部控制需要设备支持颈部功能
4. 角度值需要在设备的运动范围内