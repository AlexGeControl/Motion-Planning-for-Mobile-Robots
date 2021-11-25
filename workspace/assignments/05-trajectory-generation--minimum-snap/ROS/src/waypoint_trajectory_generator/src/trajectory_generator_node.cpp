#include <string>
#include <iostream>
#include <fstream>
#include <math.h>
#include <random>
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <quadrotor_msgs/PolynomialTrajectory.h>
#include <sensor_msgs/Joy.h>
#include <algorithm>

// Useful customized headers
#include "trajectory_generator_waypoint.h"

using namespace std;
using namespace Eigen;

enum TimeAllocation {
    SEGMENT_TRAPEZOIDAL,
    GLOBAL_TRAPEZOIDAL
};

// Param from launch file
double _vis_traj_width;
double _Vel, _Acc;
int    _obj_order, _dev_order, _min_order;

std::string _time_allocation_spec;
TimeAllocation _time_allocation;

std::string _solution_method_spec;
TrajectoryGeneratorWaypoint::Method _solution_method;

// ros related
ros::Subscriber _way_pts_sub;
ros::Publisher  _wp_traj_vis_pub, _wp_path_vis_pub;

// for planning
int _poly_num1D;
MatrixXd _polyCoeff;
VectorXd _polyTime;
Vector3d _startPos = Vector3d::Zero();
Vector3d _startVel = Vector3d::Zero();

// declare
void visWayPointTraj( MatrixXd polyCoeff, VectorXd time);
void visWayPointPath(MatrixXd path);
Vector3d getPosPoly( MatrixXd polyCoeff, int k, double t );

VectorXd AllocateTime( MatrixXd Path, TimeAllocation strategy=TimeAllocation::GLOBAL_TRAPEZOIDAL);

void trajGeneration(Eigen::MatrixXd path);
void rcvWaypointsCallBack(const nav_msgs::Path & wp);

//Get the path points 
void rcvWaypointsCallBack(const nav_msgs::Path & wp)
{   
    vector<Vector3d> wp_list;
    wp_list.clear();

    for (int k = 0; k < (int)wp.poses.size(); k++)
    {
        Vector3d pt( wp.poses[k].pose.position.x, wp.poses[k].pose.position.y, wp.poses[k].pose.position.z);
        wp_list.push_back(pt);

        if(wp.poses[k].pose.position.z < 0.0)
            break;
    }

    MatrixXd waypoints(wp_list.size() + 1, 3);
    waypoints.row(0) = _startPos;
    
    for(int k = 0; k < (int)wp_list.size(); k++)
        waypoints.row(k+1) = wp_list[k];

    // generate minimum snap trajectory:
    trajGeneration(waypoints);
}

void trajGeneration(Eigen::MatrixXd path)
{
    TrajectoryGeneratorWaypoint  trajectoryGeneratorWaypoint;
    
    MatrixXd vel = MatrixXd::Zero(2, 3); 
    MatrixXd acc = MatrixXd::Zero(2, 3);

    vel.row(0) = _startVel;

    // give an arbitraty time allocation, all set all durations as 1 in the commented function.
    _polyTime  = AllocateTime(path, _time_allocation);

    // generate a minimum-snap piecewise monomial polynomial-based trajectory
    _polyCoeff = trajectoryGeneratorWaypoint.PolyQPGeneration(
        _obj_order,
        _dev_order, 
        path, 
        vel, 
        acc, 
        _polyTime,
        _solution_method
    );

    visWayPointPath(path);

    //After you finish your homework, you can use the function visWayPointTraj below to visulize your trajectory
    visWayPointTraj(_polyCoeff, _polyTime);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "traj_node");
    ros::NodeHandle nh("~");

    nh.param("planning/max_vel",                          _Vel,                    1.00);
    nh.param("planning/max_acc",                          _Acc,                    0.50);
    nh.param("planning/t_order",                    _obj_order,                       4);
    nh.param("planning/c_order",                    _dev_order,                       2);
    nh.param("planning/min_c_order",                _min_order,                       2);
    nh.param("planning/time_allocation", _time_allocation_spec,   std::string("global"));
    nh.param("planning/solution_method", _solution_method_spec, std::string("analytic"));
    nh.param("planning/min_c_order",                _min_order,                       2);
    nh.param("vis/vis_traj_width",             _vis_traj_width,                    0.15);

    //_poly_numID is the maximum order of polynomial
    _dev_order = std::max(_min_order, _dev_order);
    _poly_num1D = TrajectoryGeneratorWaypoint::GetNumCoeffs(_dev_order);

    _time_allocation = ((_time_allocation_spec == "global") ? TimeAllocation::GLOBAL_TRAPEZOIDAL : TimeAllocation::SEGMENT_TRAPEZOIDAL);
    _solution_method = ((_solution_method_spec == "analytic") ? TrajectoryGeneratorWaypoint::Method::Analytic : TrajectoryGeneratorWaypoint::Method::Numeric);

    //state of start point
    _startPos(0)  = 0;
    _startPos(1)  = 0;
    _startPos(2)  = 0;    

    _startVel(0)  = 0;
    _startVel(1)  = 0;
    _startVel(2)  = 0;
    
    _way_pts_sub     = nh.subscribe( "waypoints", 1, rcvWaypointsCallBack );

    _wp_traj_vis_pub = nh.advertise<visualization_msgs::Marker>("vis_trajectory", 1);
    _wp_path_vis_pub = nh.advertise<visualization_msgs::Marker>("vis_waypoint_path", 1);

    ros::Rate rate(100);
    bool status = ros::ok();
    while(status) 
    {
        ros::spinOnce();  
        status = ros::ok();  
        rate.sleep();
    }
    return 0;
}

void visWayPointTraj( MatrixXd polyCoeff, VectorXd time)
{        
    visualization_msgs::Marker _traj_vis;

    _traj_vis.header.stamp       = ros::Time::now();
    _traj_vis.header.frame_id    = "/map";

    _traj_vis.ns = "traj_node/trajectory_waypoints";
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
          pos = getPosPoly(polyCoeff, i, t / time(i));
          cur(0) = pt.x = pos(0);
          cur(1) = pt.y = pos(1);
          cur(2) = pt.z = pos(2);
          _traj_vis.points.push_back(pt);

          if (count) traj_len += (pre - cur).norm();
          pre = cur;
        }
    }

    _wp_traj_vis_pub.publish(_traj_vis);
}

void visWayPointPath(MatrixXd path)
{
    visualization_msgs::Marker points, line_list;
    int id = 0;
    points.header.frame_id    = line_list.header.frame_id    = "/map";
    points.header.stamp       = line_list.header.stamp       = ros::Time::now();
    points.ns                 = line_list.ns                 = "wp_path";
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

    _wp_path_vis_pub.publish(points);
    _wp_path_vis_pub.publish(line_list);
}

Eigen::Vector3d getPosPoly(Eigen::MatrixXd polyCoeff, int k, double t )
{
    Vector3d ret;

    for ( int dim = 0; dim < 3; dim++ )
    {
        const Eigen::VectorXd &coeff = (polyCoeff.row(k)).segment( dim * _poly_num1D, _poly_num1D );
        ret(dim) = TrajectoryGeneratorWaypoint::EvaluatePoly(coeff, t);
    }

    return ret;
}

Eigen::VectorXd DoGlobalTrapezoidalTimeAllocation(Eigen::MatrixXd Path) {
    const int K = (Path.rows() - 1);
    
    Eigen::VectorXd time = Eigen::VectorXd::Zero(K);
    
    /*
        STEP 1: use "trapezoidal velocity" heuristic for time allocation
    */
    const double constAccDisplacement = (
        (_Vel * _Vel ) / (2.0 * _Acc)
    );

    // calculate segment / accumulated displacement:
    double totalDisplacement{0.0};
    Eigen::VectorXd segDisplacement = Eigen::VectorXd::Zero(K);
    for (int k = 1; k <= K; ++k) {
        // calculate segment displacement:
        segDisplacement(k - 1) = (Path.row(k) - Path.row(k - 1)).norm();
        // update accumulated displacement:
        totalDisplacement += segDisplacement(k - 1);
    }

    //
    // init time allocation:
    //
    double totalTime{0.0};
    double maxVel{_Vel};
    double accDisplacement{0.0}, decDisplacement{0.0};

    double currDisplacement{0.0}, nextDisplacement{0.0};
    double currTime{0.0}, nextTime{0.0};

    // CASE 1: trapezoidal is reduced to triangle
    if (totalDisplacement <= 2.0 * constAccDisplacement) {
        // set phase transition displacements:
        accDisplacement = decDisplacement = (totalDisplacement / 2.0);

        // set max. vel:
        maxVel = std::sqrt(2.0 * _Acc * decDisplacement);

        // set total duration:
        totalTime = 2.0 * (maxVel / _Acc);
    } 
    // CASE 2: stay with trapezoidal 
    else {
        // set phase transition displacements:
        accDisplacement = constAccDisplacement;
        decDisplacement = totalDisplacement - constAccDisplacement;

        // set max. vel:
        maxVel = _Vel;

        // set total duration:
        totalTime = 2.0 * (_Vel / _Acc) + (totalDisplacement - 2.0 * constAccDisplacement) / _Vel;
    }

    //
    // do time allocation:
    //
    for (int k = 0; k < K; ++k) {
        // generate proposal:
        nextDisplacement = currDisplacement + segDisplacement(k);

        // STAGE 1: acceleration
        if (nextDisplacement <= accDisplacement) {
            double endVel = std::sqrt(2.0 * _Acc * nextDisplacement);
            nextTime = endVel / _Acc;
        } 
        // STAGE 2: constant velocity
        else if (nextDisplacement <= decDisplacement) {
            nextTime = (maxVel / _Acc) + (nextDisplacement - accDisplacement) / maxVel;
        }
        // STAGE 3: deceleration
        else {
            double endVel = std::sqrt(2.0 * _Acc * (totalDisplacement - nextDisplacement));
            nextTime = totalTime - (endVel / _Acc);
        }

        // allocate time:
        time(k) = nextTime - currTime;

        // proceed to next segment;
        currDisplacement = nextDisplacement;
        currTime = nextTime;
    }

    return time;    
}

Eigen::VectorXd DoSegmentTrapezoidalTimeAllocation(Eigen::MatrixXd Path)
{ 
    const int K = (Path.rows() - 1);
    
    Eigen::VectorXd time = Eigen::VectorXd::Zero(K);
    
    /*
        STEP 1: use "trapezoidal velocity" heuristic for time allocation
    */
    const double constAccDisplacement = (
        (_Vel * _Vel ) / (2.0 * _Acc)
    );

    // calculate segment  displacement:
    Eigen::VectorXd segDisplacement = Eigen::VectorXd::Zero(K);
    for (int k = 1; k <= K; ++k) {
        // calculate segment displacement:
        segDisplacement(k - 1) = (Path.row(k) - Path.row(k - 1)).norm();
    }

    //
    // do time allocation:
    //
    double maxVel{0.0};
    double segTime{0.0};
    for (int k = 0; k < K; ++k) {
        // CASE 1: trapezoidal is reduced to triangle
        if (segDisplacement(k) <= 2.0 * constAccDisplacement) {
            // set max. vel:
            maxVel = std::sqrt(_Acc * segDisplacement(k));

            // set total duration:
            segTime = 2.0 * (maxVel / _Acc);
        } 
        // CASE 2: stay with trapezoidal 
        else {
            // set max. vel:
            maxVel = _Vel;

            // set total duration:
            segTime = 2.0 * (maxVel / _Acc) + (segDisplacement(k) - 2.0 * constAccDisplacement) / _Vel;
        }

        // allocate time:
        time(k) = segTime;
    }

    return time;
}

Eigen::VectorXd AllocateTime(Eigen::MatrixXd Path, TimeAllocation strategy) {
    const int K = (Path.rows() - 1);
    
    Eigen::VectorXd time = Eigen::VectorXd::Zero(K);

    switch (strategy) {
        case TimeAllocation::SEGMENT_TRAPEZOIDAL:
            return DoSegmentTrapezoidalTimeAllocation(Path);
            break;
        case TimeAllocation::GLOBAL_TRAPEZOIDAL:
            return DoGlobalTrapezoidalTimeAllocation(Path);
            break;
        default:
            break;
    }

    return time;
}