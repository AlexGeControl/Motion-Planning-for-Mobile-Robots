#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <quadrotor_msgs/PolynomialTrajectory.h>
#include <random>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// Useful customized headers
#include "path_finder.hpp"
#include "backward.hpp"
#include "trajectory_optimizer.hpp"

using namespace std;
using namespace Eigen;

TrajectoryOptimizer *_traj_optimizer = new TrajectoryOptimizer();
PathFinder *_path_finder = new PathFinder();

// Set the obstacle map
std::string _map_frame_name;
double _resolution, _inv_resolution, _path_resolution;
double _x_size, _y_size, _z_size;
Vector3d _map_lower, _map_upper;
int _max_x_id, _max_y_id, _max_z_id;

// Param from launch file
double _vis_traj_width;
double _Vel, _Acc;
int _t_order, _c_order, _min_order;

// ros related
ros::Subscriber _map_sub, _pts_sub, _odom_sub;
ros::Publisher _traj_vis_pub, _traj_pub, _path_vis_pub;

// for planning
Vector3d odom_pos, odom_vel, start_pos, target_pos, start_vel;
int _poly_num1D;
MatrixXd _polyCoeff;
VectorXd _polyTime;
double time_duration;
ros::Time time_traj_start;
bool has_odom = false;
bool has_target = false;

// for replanning
enum STATE {
  INIT,
  WAIT_TARGET,
  GEN_NEW_TRAJ,
  EXEC_TRAJ,
  REPLAN_TRAJ
} exec_state = STATE::INIT;
double no_replan_thresh, replan_thresh;
ros::Timer _exec_timer;
void STMCB(const ros::TimerEvent &e);

// declare
void changeState(STATE new_state, string pos_call);
void printState();

void VisualizeWaypoints(const Eigen::MatrixXd &nodes);
void VisualizeTrajectory(const Eigen::MatrixXd &polyCoeff, const Eigen::VectorXd &time);
void PublishTrajectory(const Eigen::MatrixXd &polyCoeff, const Eigen::VectorXd &time);

void OptimizeTrajectory(const Eigen::MatrixXd &waypoints);
void OdometryCB(const nav_msgs::Odometry::ConstPtr &odom);
void rcvWaypointsCallback(const nav_msgs::Path &wp);
void PointCloudCB(const sensor_msgs::PointCloud2 &pointcloud_map);

bool trajGeneration();

Vector3d getPos(double t_cur);
Vector3d getVel(double t_cur);

// change the state to the new state
void changeState(STATE new_state, string pos_call) {
  string state_str[5] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "EXEC_TRAJ",
                         "REPLAN_TRAJ"};
  int pre_s = int(exec_state);
  exec_state = new_state;
  cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " +
              state_str[int(new_state)]
       << endl;
}

void printState() {
  string state_str[5] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "EXEC_TRAJ",
                         "REPLAN_TRAJ"};
  cout << "[Clock]: state: " + state_str[int(exec_state)] << endl;
}

void OdometryCB(const nav_msgs::Odometry::ConstPtr &msg) {
  // update ego state:
  // position:
  odom_pos(0) = msg->pose.pose.position.x;
  odom_pos(1) = msg->pose.pose.position.y;
  odom_pos(2) = msg->pose.pose.position.z;

  // velocity:
  odom_vel(0) = msg->twist.twist.linear.x;
  odom_vel(1) = msg->twist.twist.linear.y;
  odom_vel(2) = msg->twist.twist.linear.z;

  // done:
  has_odom = true;
}

void GoalCB(const nav_msgs::Path &msg) {
  // update navigation goal:

  // set navigation goal:
  const auto& goal_position = msg.poses[0].pose.position;

  if (goal_position.z >= 0.0) {
      start_pos = odom_pos;
      start_vel = odom_vel;

      target_pos << goal_position.x, 
                    goal_position.y,
                    goal_position.z;
      
      has_target = true;

      ROS_INFO("[QuadPlanner] Navigation target received.");
  
      if (exec_state == WAIT_TARGET) {
        changeState(GEN_NEW_TRAJ, "STATE");
      }
      else if (exec_state == EXEC_TRAJ) {
        changeState(REPLAN_TRAJ, "STATE");
      }
  }
}

void PointCloudCB(const sensor_msgs::PointCloud2 &msg) {
  // update perceived obstacles:
  pcl::PointCloud<pcl::PointXYZ> point_cloud;

  pcl::fromROSMsg(msg, point_cloud);

  if (point_cloud.points.size() > 0u) {
    for (size_t idx = 0; idx < point_cloud.points.size(); ++idx) {
      const auto &point = point_cloud.points[idx];

      // update obstacles in grid map:
      _path_finder->setObs(point.x, point.y, point.z);
    }
  }
}

void STMCB(const ros::TimerEvent &e) {
  static int num = 0;
  
  num++;
  if (num == 100) {
    printState();
    if (!has_odom)
      cout << "no odom." << endl;
    if (!has_target)
      cout << "wait for goal." << endl;
    num = 0;
  }

  switch (exec_state) {
    case INIT: {
      if (!has_odom)
        return;
      if (!has_target)
        return;
      changeState(WAIT_TARGET, "STATE");
      break;
    }

    case WAIT_TARGET: {
      if (!has_target)
        return;
      else
        changeState(GEN_NEW_TRAJ, "STATE");
      break;
    }

    case GEN_NEW_TRAJ: {
      bool success = trajGeneration();
      if (success)
        changeState(EXEC_TRAJ, "STATE");
      else
        changeState(GEN_NEW_TRAJ, "STATE");
      break;
    }

    case EXEC_TRAJ: {
      ros::Time time_now = ros::Time::now();
      double t_cur = (time_now - time_traj_start).toSec();
      double t_replan = ros::Duration(1, 0).toSec();
      t_cur = min(time_duration, t_cur);

      if (t_cur > time_duration - 1e-2) {
        has_target = false;
        changeState(WAIT_TARGET, "STATE");
        return;
      } else if ((target_pos - odom_pos).norm() < no_replan_thresh) {
        return;
      } else if ((start_pos - odom_pos).norm() < replan_thresh) {
        return;
      } else if (t_cur < t_replan) {
        return;
      } else {
        changeState(REPLAN_TRAJ, "STATE");
      }
      break;
    }
    case REPLAN_TRAJ: {
      ros::Time time_now = ros::Time::now();
      double t_cur = (time_now - time_traj_start).toSec();
      double t_delta = ros::Duration(0, 50).toSec();
      t_cur = t_delta + t_cur;
      start_pos = getPos(t_cur);
      start_vel = getVel(t_cur);
      bool success = trajGeneration();
      if (success)
        changeState(EXEC_TRAJ, "STATE");
      else
        changeState(GEN_NEW_TRAJ, "STATE");
      break;
    }
  }
}

// trajectory generation: front-end + back-end
// front-end : A* search method
// back-end  : Minimum snap trajectory generation
bool trajGeneration() {
  // STEP 1: find path with A*
  _path_finder->FindPath(start_pos, target_pos);
  auto grid_path = _path_finder->GetPath();
  // reset map for next call
  _path_finder->resetUsedGrids();

  // STEP 2: simplify path with
  grid_path = _path_finder->SimplifyPath(grid_path, _path_resolution);
  Eigen::MatrixXd path(int(grid_path.size()), 3);
  for (int k = 0; k < int(grid_path.size()); k++) {
    path.row(k) = grid_path[k];
    ROS_WARN(
      "\t[RDP]: (%.2f, %.2f, %.2f) @ %d", 
      path(k, 0), path(k, 1), path(k, 2),
      k 
    );
  }

  // STEP 3: optimize trajectory with minimum-snap piecewise monomial trajectory
  OptimizeTrajectory(path);
  time_duration = _polyTime.sum();

  // STEP 4: publish the trajectory
  PublishTrajectory(_polyCoeff, _polyTime);
  // record the trajectory start time
  time_traj_start = ros::Time::now();

  // done:
  if (_polyCoeff.rows() > 0)
    return true;
  else
    return false;
}

void OptimizeTrajectory(const Eigen::MatrixXd &waypoints) {
  // if( !has_odom ) return;
  MatrixXd vel = MatrixXd::Zero(2, 3);
  MatrixXd acc = MatrixXd::Zero(2, 3);

  vel.row(0) = start_vel;

  // STEP 3.1: allocate time to each trajectory segment
  _polyTime = _traj_optimizer->AllocateTimes(waypoints, _Vel, _Acc);

  // STEP 3.2: generate a minimum-jerk piecewise monomial trajectory
  _polyCoeff = _traj_optimizer->GenerateTrajectory(
    _t_order, _c_order, 
    waypoints, vel, acc, _polyTime,
    TrajectoryOptimizer::Solver::Numeric
  );

  // STEP 3.3: do collision detection:
  const auto numUnsafeSegment = _path_finder->safeCheck(_polyCoeff, _polyTime);

  const Eigen::MatrixXd repath = waypoints;
  int count = 0;
  while (numUnsafeSegment != -1) {
    /**
     *
     * STEP 3.4:  reoptimize
     * the method comes from: "Polynomial Trajectory
     * Planning for Aggressive Quadrotor Flight in Dense Indoor Environment"
     * part 3.5
     *
     * **/
  }
  // visulize path and trajectory
  VisualizeWaypoints(repath);
  VisualizeTrajectory(_polyCoeff, _polyTime);
}

void PublishTrajectory(
  const Eigen::MatrixXd &polyCoeff, 
  const Eigen::VectorXd &time
) {
  if (polyCoeff.size() == 0 || time.size() == 0) {
    ROS_WARN("[trajectory_generator_waypoint] empty trajectory, nothing to "
             "publish.");
    return;
  }

  unsigned int poly_number;

  static int count =
      1; // The first trajectory_id must be greater than 0. zxzxzxzx

  quadrotor_msgs::PolynomialTrajectory traj_msg;

  traj_msg.header.seq = count;
  traj_msg.header.stamp = ros::Time::now();
  traj_msg.header.frame_id = _map_frame_name;
  traj_msg.trajectory_id = count;
  traj_msg.action = quadrotor_msgs::PolynomialTrajectory::ACTION_ADD;

  traj_msg.num_order = 2 * _t_order - 1; // the order of polynomial
  traj_msg.num_segment = time.size();

  Vector3d initialVel, finalVel;
  initialVel = _traj_optimizer->GetVel(
    _polyCoeff, 
    _poly_num1D, 0, 
    0.0
  );
  finalVel = _traj_optimizer->GetVel(
    _polyCoeff, 
    _poly_num1D, traj_msg.num_segment - 1,
    time(traj_msg.num_segment - 1)
  );
  traj_msg.start_yaw = atan2(initialVel(1), initialVel(0));
  traj_msg.final_yaw = atan2(finalVel(1), finalVel(0));

  poly_number = traj_msg.num_order + 1;
  Eigen::VectorXd timeExponentials = Eigen::VectorXd::Ones(poly_number);
  for (unsigned int i = 0; i < traj_msg.num_segment; i++) {
    for (int n = 1; n < poly_number; ++n) {
      timeExponentials(n) = time(i)*timeExponentials(n - 1);
    }

    for (unsigned int j = 0; j < poly_number; j++) {
      traj_msg.coef_x.push_back(polyCoeff(i, j)*timeExponentials(j));
      traj_msg.coef_y.push_back(polyCoeff(i, poly_number + j)*timeExponentials(j));
      traj_msg.coef_z.push_back(polyCoeff(i, 2 * poly_number + j)*timeExponentials(j));
    }
    traj_msg.time.push_back(time(i));
    traj_msg.order.push_back(traj_msg.num_order);
  }
  traj_msg.mag_coeff = 1;

  count++;
  ROS_WARN("[traj..gen...node] traj_msg publish");
  _traj_pub.publish(traj_msg);
}

void VisualizeWaypoints(const Eigen::MatrixXd &path) {
  visualization_msgs::Marker points, line_list;
  int id = 0;
  points.header.frame_id    = line_list.header.frame_id    = _map_frame_name;
  points.header.stamp       = line_list.header.stamp       = ros::Time::now();
  points.ns                 = line_list.ns                 = "trajectory_generator/trajectory";
  points.action             = line_list.action             = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = line_list.pose.orientation.w = 1.0;
  points.pose.orientation.x = line_list.pose.orientation.x = 0.0;
  points.pose.orientation.y = line_list.pose.orientation.y = 0.0;
  points.pose.orientation.z = line_list.pose.orientation.z = 0.0;

  points.id    = id;
  line_list.id = id;

  points.type    = visualization_msgs::Marker::SPHERE_LIST;
  line_list.type = visualization_msgs::Marker::LINE_STRIP;

  points.scale.x = 0.3;
  points.scale.y = 0.3;
  points.scale.z = 0.3;
  points.color.a = 1.0;
  points.color.r = 0.0;
  points.color.g = 0.0;
  points.color.b = 0.0;

  line_list.scale.x = 0.15;
  line_list.scale.y = 0.15;
  line_list.scale.z = 0.15;
  line_list.color.a = 1.0;

  
  line_list.color.r = 0.0;
  line_list.color.g = 1.0;
  line_list.color.b = 0.0;
  
  line_list.points.clear();

  for(int i = 0; i < path.rows(); i++){
    geometry_msgs::Point p;
    p.x = path(i, 0);
    p.y = path(i, 1); 
    p.z = path(i, 2); 

    points.points.push_back(p);

    if( i < (path.rows() - 1) )
    {
        geometry_msgs::Point p_line;
        p_line = p;
        line_list.points.push_back(p_line);
        p_line.x = path(i+1, 0);
        p_line.y = path(i+1, 1); 
        p_line.z = path(i+1, 2);
        line_list.points.push_back(p_line);
    }
  }

  _path_vis_pub.publish(points);
  _path_vis_pub.publish(line_list);
}

void VisualizeTrajectory(
  const Eigen::MatrixXd &polyCoeff, 
  const Eigen::VectorXd &time
) {
  visualization_msgs::Marker _traj_vis;

  _traj_vis.header.stamp       = ros::Time::now();
  _traj_vis.header.frame_id    = _map_frame_name;

  _traj_vis.ns = "trajectory_generator/waypoints";
  _traj_vis.id = 0;
  _traj_vis.type = visualization_msgs::Marker::SPHERE_LIST;
  _traj_vis.action = visualization_msgs::Marker::ADD;
  _traj_vis.scale.x = _vis_traj_width;
  _traj_vis.scale.y = _vis_traj_width;
  _traj_vis.scale.z = _vis_traj_width;
  _traj_vis.pose.orientation.x = 0.0;
  _traj_vis.pose.orientation.y = 0.0;
  _traj_vis.pose.orientation.z = 0.0;
  _traj_vis.pose.orientation.w = 1.0;

  _traj_vis.color.a = 1.0;
  _traj_vis.color.r = 1.0;
  _traj_vis.color.g = 0.0;
  _traj_vis.color.b = 0.0;

  double traj_len = 0.0;
  int count = 0;
  Vector3d cur, pre;
  cur.setZero();
  pre.setZero();

  _traj_vis.points.clear();
  Vector3d pos;
  geometry_msgs::Point pt;

  for(int i = 0; i < time.size(); i++ )
  {   
      for (double t = 0.0; t < time(i); t += 0.01, count += 1)
      {
        pos = _traj_optimizer->GetPos(polyCoeff, _poly_num1D, i, t);
        cur(0) = pt.x = pos(0);
        cur(1) = pt.y = pos(1);
        cur(2) = pt.z = pos(2);
        _traj_vis.points.push_back(pt);

        if (count) {
          traj_len += (pre - cur).norm();
        }
        pre = cur;
      }

      pos = _traj_optimizer->GetPos(polyCoeff, _poly_num1D, i, time(i));
      ROS_WARN(
        "\t[Seg. %d]: (%.2f, %.2f, %.2f) @ %.2f",
        i,
        pos(0), pos(1), pos(2),
        time(i)
      );
  }

  _traj_vis_pub.publish(_traj_vis);
}

Vector3d getPos(double t_cur) {
  double time = 0;
  Vector3d pos = Vector3d::Zero();
  for (int i = 0; i < _polyTime.size(); i++) {
    for (double t = 0.0; t < _polyTime(i); t += 0.01) {
      time = time + 0.01;
      if (time > t_cur) {
        pos = _traj_optimizer->GetPos(_polyCoeff, _poly_num1D, i, t);
        return pos;
      }
    }
  }
  return pos;
}

Vector3d getVel(double t_cur) {
  double time = 0;
  Vector3d Vel = Vector3d::Zero();
  for (int i = 0; i < _polyTime.size(); i++) {
    for (double t = 0.0; t < _polyTime(i); t += 0.01) {
      time = time + 0.01;
      if (time > t_cur) {
        Vel = _traj_optimizer->GetVel(_polyCoeff, _poly_num1D, i, t);
        return Vel;
      }
    }
  }
  return Vel;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "traj_node");
  ros::NodeHandle nh("~");

  nh.param("planning/vel", _Vel, 1.0);
  nh.param("planning/acc", _Acc, 1.0);

  nh.param("planning/min_order", _min_order, 3);
  nh.param("planning/t_order", _t_order, 4);
  nh.param("planning/c_order", _c_order, 3);

  nh.param("vis/vis_traj_width", _vis_traj_width, 0.15);
  nh.param("map_frame_name", _map_frame_name, std::string("world"));
  nh.param("map/resolution", _resolution, 0.2);
  nh.param("map/x_size", _x_size, 50.0);
  nh.param("map/y_size", _y_size, 50.0);
  nh.param("map/z_size", _z_size, 5.0);
  nh.param("path/resolution", _path_resolution, 0.05);
  nh.param("replanning/thresh_replan", replan_thresh, -1.0);
  nh.param("replanning/thresh_no_replan", no_replan_thresh, -1.0);

  // sanity check:
  // set objective function:
  _t_order = max(_min_order, _t_order);
  // set waypoint continuity constraint:
  _c_order = (_t_order - 1);
  _poly_num1D = TrajectoryOptimizer::GetNumCoeffs(_c_order);

  _exec_timer = nh.createTimer(ros::Duration(0.01), STMCB);

  _odom_sub = nh.subscribe("odom", 10, OdometryCB);
  _map_sub = nh.subscribe("local_pointcloud", 1, PointCloudCB);
  _pts_sub = nh.subscribe("waypoints", 1, GoalCB);

  _traj_pub = nh.advertise<quadrotor_msgs::PolynomialTrajectory>("trajectory", 50);
  _traj_vis_pub = nh.advertise<visualization_msgs::Marker>("vis_trajectory", 1);
  _path_vis_pub = nh.advertise<visualization_msgs::Marker>("vis_path", 1);

  // init obstacle map, the playground of quadrotor planner:
  _map_lower << -_x_size / 2.0, -_y_size / 2.0, 0.0;
  _map_upper << +_x_size / 2.0, +_y_size / 2.0, _z_size;
  _inv_resolution = 1.0 / _resolution;
  _max_x_id = (int)(_x_size * _inv_resolution);
  _max_y_id = (int)(_y_size * _inv_resolution);
  _max_z_id = (int)(_z_size * _inv_resolution);

  _path_finder = new PathFinder();
  _path_finder->initGridMap(
    _resolution, _map_lower, _map_upper,
      _max_x_id,  _max_y_id,  _max_z_id
  );

  ros::Rate rate(100);
  bool status = ros::ok();
  while (status) {
    ros::spinOnce();
    status = ros::ok();
    rate.sleep();
  }

  return EXIT_SUCCESS;
}