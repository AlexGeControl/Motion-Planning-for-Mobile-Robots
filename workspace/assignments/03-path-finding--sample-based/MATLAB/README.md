# Motion Planning for Mobile Robots -- 移动机器人运动规划: Path Finding, Sample Based

深蓝学院移动机器人运动规划第3节Path Finding, Sample Based作业框架.

---

## Overview

本作业旨在引导您:

* 基于MATLAB Native, 实现RRT Path Finding

---

## Up & Running


---

## Q1. 算法流程与运行结果

### RRT Star

**RRT Star**的运行结果如下:

* **Green**为RRT Exploration的结果
* **Blue**为查找到的Optimal Path

<img src="doc/images/output.jpg" alt="Sample Based Path Finding Demo, MATLAB" width="100%">

算法流程如下:

* [Step 1: Random Sample](https://github.com/AlexGeControl/Motion-Planning-for-Mobile-Robots/blob/206a2ba1076c6c7b2765fafb4f13801730941d74/workspace/assignments/03-path-finding--sample-based/MATLAB/RRT.m#L43)
* [Step 2: Find Closest](https://github.com/AlexGeControl/Motion-Planning-for-Mobile-Robots/blob/206a2ba1076c6c7b2765fafb4f13801730941d74/workspace/assignments/03-path-finding--sample-based/MATLAB/RRT.m#L49)
* [Step 3: Get Candidate New Node](https://github.com/AlexGeControl/Motion-Planning-for-Mobile-Robots/blob/206a2ba1076c6c7b2765fafb4f13801730941d74/workspace/assignments/03-path-finding--sample-based/MATLAB/RRT.m#L66)
* [Step 4: Insert into Tree](https://github.com/AlexGeControl/Motion-Planning-for-Mobile-Robots/blob/206a2ba1076c6c7b2765fafb4f13801730941d74/workspace/assignments/03-path-finding--sample-based/MATLAB/RRT.m#L79)
* [Step 5: Visualize](https://github.com/AlexGeControl/Motion-Planning-for-Mobile-Robots/blob/206a2ba1076c6c7b2765fafb4f13801730941d74/workspace/assignments/03-path-finding--sample-based/MATLAB/RRT.m#L87)
* [Step 6: Goal Check](https://github.com/AlexGeControl/Motion-Planning-for-Mobile-Robots/blob/206a2ba1076c6c7b2765fafb4f13801730941d74/workspace/assignments/03-path-finding--sample-based/MATLAB/RRT.m#L91)

---
