#!/bin/bash
rostopic pub --once /goal geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'map'
pose:
  position:
    x: 5.0
    y: 5.0
    z: 1.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0" 

rostopic pub --once /goal geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'map'
pose:
  position:
    x: -5.0
    y:  5.0
    z:  2.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0" 

rostopic pub --once /goal geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'map'
pose:
  position:
    x: -5.0
    y: -5.0
    z:  3.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0" 

rostopic pub --once /goal geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'map'
pose:
  position:
    x:  5.0
    y: -5.0
    z:  4.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0" 

rostopic pub --once /goal geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'map'
pose:
  position:
    x: 7.5
    y: 7.5
    z: 5.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0" 

rostopic pub --once /goal geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'map'
pose:
  position:
    x: -7.5
    y:  7.5
    z: -1.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0" 