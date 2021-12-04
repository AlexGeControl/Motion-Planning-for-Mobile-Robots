# Motion Planning for Mobile Robots -- 移动机器人运动规划: Trajectory Generation, Collision Avoidance

深蓝学院移动机器人运动规划第6节Trajectory Generation, Collision Avoidance作业框架.

---

## Overview

本作业旨在引导您:

* 基于ROS C++, 实现基于Bezier Curve + Flight Corridor的Quadrotor自主避障.

---

## Up & Running

启动Docker后, 打开浏览器, 前往localhost:40080, 进入Web Workspace. **若需要提高清晰度, 可以更改URL中的quality参数**. 启动Terminator, 将两个Shell的工作目录切换如下:

<img src="doc/images/terminator.png" alt="Terminator" width="100%">

在**左侧**的Shell中, 输入如下命令, **编译Search Based Path Finder**

```bash
# build
catkin_make
```

然后**启动解决方案**. 可配置的参数如下表所示:

|      Param      |                        Meaning                        |           Input Domain          |
|:---------------:|:-----------------------------------------------------:|:-------------------------------:|
|     max_vel     |                   max. ego velocity                   |          0.0 <= double          |
|     max_acc     |                 max. ego acceleration                 |          0.0 <= double          |
|     t_order     |  L2 norm of t_order derivative as objective function  | 0 <= integer <= 2 * c_order + 1 |
|     c_order     | the generated trajectory should be c_order continuous |      min_c_order <= integer     |
|   min_c_order   |                      min. c_order                     |           0 <= integer          |
| time_allocation |               time allocation heuristic               |         segment / global        |
| solution_method |                  minimum snap solver                  |        analytic / numeric       |

```bash
# set up session:
source devel/setup.bash
# launch:
roslaunch waypoint_trajectory_generator test.launch 
```

随后在**右侧**的Shell中, 输入如下命令, **发布随机的Coarse Path, 触发Trajectory Optimization**
```bash
# set up session:
source devel/setup.bash

# publish coarse path - set waypoint_type to manual
./src/waypoint_trajectory_generator/scripts/publish_waypoints.sh

# or just publish stop waypoint for fast debugging - set waypoint_type to circle or eight:
./src/waypoint_trajectory_generator/scripts/publish_stop_waypoint.sh
```
---

## Q1. 算法流程与运行结果

### Minimum Snap

**Minimum Snap**的运行结果如下, **Eight Waypoints Set** using **Numeric Solver**:

<img src="doc/images/demo-RViz-eight-numeric.png" alt="Minimum Snap Solver, Eight Points, Numeric" width="100%">

**Minimum Snap**的运行结果如下, **Circle Waypoints Set** using **Numeric Solver**:

<img src="doc/images/demo-RViz-circle-numeric.png" alt="Minimum Snap Solver, Circle, Numeric" width="100%">

**Minimum Snap**的运行结果如下, **Eight Waypoints Set** using **Analytic Solver**:

<img src="doc/images/demo-RViz-eight-analytic.png" alt="Minimum Snap Solver, Eight Points, Analytic" width="100%">

**Minimum Snap**的运行结果如下, **Circle Waypoints Set** using **Analytic Solver**:

<img src="doc/images/demo-RViz-circle-analytic.png" alt="Minimum Snap Solver, Circle, Analytic" width="100%">

算法流程如下:

* [Step 1: Time Allocation](https://github.com/AlexGeControl/Motion-Planning-for-Mobile-Robots/blob/1eb40c7056e4768e630039aee60c0d492b20f13c/workspace/assignments/05-trajectory-generation--minimum-snap/ROS/src/waypoint_trajectory_generator/src/trajectory_generator_node.cpp#L430)
    * [Step 1.1: Time Allocation, Segment Trapezoidal](https://github.com/AlexGeControl/Motion-Planning-for-Mobile-Robots/blob/1eb40c7056e4768e630039aee60c0d492b20f13c/workspace/assignments/05-trajectory-generation--minimum-snap/ROS/src/waypoint_trajectory_generator/src/trajectory_generator_node.cpp#L380)
    * [Step 1.2: Time Allocation, Global Trapezoidal](https://github.com/AlexGeControl/Motion-Planning-for-Mobile-Robots/blob/1eb40c7056e4768e630039aee60c0d492b20f13c/workspace/assignments/05-trajectory-generation--minimum-snap/ROS/src/waypoint_trajectory_generator/src/trajectory_generator_node.cpp#L291)
* [Step 2: Minimum Snap Solver Interface](https://github.com/AlexGeControl/Motion-Planning-for-Mobile-Robots/blob/1eb40c7056e4768e630039aee60c0d492b20f13c/workspace/assignments/05-trajectory-generation--minimum-snap/ROS/src/waypoint_trajectory_generator/src/trajectory_generator_waypoint.cpp#L66)
    * [Step 2.1: Numeric Solver with OSQP C++](https://github.com/AlexGeControl/Motion-Planning-for-Mobile-Robots/blob/1eb40c7056e4768e630039aee60c0d492b20f13c/workspace/assignments/05-trajectory-generation--minimum-snap/ROS/src/waypoint_trajectory_generator/src/trajectory_generator_waypoint.cpp#L135)
    * [Step 2.1: Analytic Solver with C++ Eigen](https://github.com/AlexGeControl/Motion-Planning-for-Mobile-Robots/blob/1eb40c7056e4768e630039aee60c0d492b20f13c/workspace/assignments/05-trajectory-generation--minimum-snap/ROS/src/waypoint_trajectory_generator/src/trajectory_generator_waypoint.cpp#L402)

---
