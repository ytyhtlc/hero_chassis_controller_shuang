# hero_chassis_controller

## Overview

麦克纳姆轮底盘控制器，基于 `ros_control` 的 `EffortJointInterface`。接收 `/cmd_vel`，用逆运动学得到四轮期望转速，四个 `control_toolbox::Pid` 出力矩；用正运动学积分里程计，发布 `/odom` 并广播 `odom -> base_link`。可在底盘坐标系速度模式和全局坐标系速度模式之间切换。

**Keywords:** RoboMaster, ROS, ros_control, mecanum

### License

The source code is released under a [BSD 3-Clause license](LICENSE).

**Author: shuang  
Affiliation: DynamicX  
Maintainer: shuang, 2129274677@qq.com**

Tested under [ROS] Noetic on Ubuntu 20.04.

## Installation

### Dependencies

- [ROS](http://wiki.ros.org) Noetic
- [rm_description](https://github.com/gdut-dynamic-x/rm_description) (assignment 分支)
- controller_interface, hardware_interface, pluginlib, control_toolbox
- geometry_msgs, nav_msgs, std_msgs, tf2, tf2_ros, tf2_geometry_msgs

### Building

    cd ~/catkin_ws/src
    git clone git@github.com:ytyhtlc/hero_chassis_controller_shuang.git
    cd ..
    rosdep install --from-paths src --ignore-src -y
    catkin build
    source devel/setup.bash

## Usage

    roslaunch hero_chassis_controller run_simulation_and_controller.launch


调 PID：`rqt_reconfigure`、`rqt_plot` 看 `~/fl/des`、`~/fl/act` 等。

## Config files

* **config/controllers.yaml**  四轮 PID、`use_global_vel`（false=底盘模式，true=全局模式）、`global_frame`。

## Launch files

* **run_simulation_and_controller.launch:** 加载英雄底盘仿真，并 spawn `hero_chassis_controller` 与 `joint_state_controller`。

## Controller

### Subscribed Topics

* **`/cmd_vel`** (`geometry_msgs/Twist`)  
  底盘模式：视为 `base_link` 速度。全局模式：视为 `global_frame` 速度，经 TF 转到车体后再做 IK。

### Published Topics

* **`/odom`** (`nav_msgs/Odometry`)  正运动学积分得到的里程计。
* **`~/fl/des` `~/fl/act` ...** (`std_msgs/Float64`)  四轮期望/实际角速度，供 rqt_plot。

### TF

* **`odom` -> `base_link`**

### Parameters

* **`use_global_vel`** (bool, default: false)
* **`global_frame`** (string, default: "odom")
* **`pid_fl` / `pid_fr` / `pid_bl` / `pid_br`**  各轮 PID。

[ROS]: http://www.ros.org