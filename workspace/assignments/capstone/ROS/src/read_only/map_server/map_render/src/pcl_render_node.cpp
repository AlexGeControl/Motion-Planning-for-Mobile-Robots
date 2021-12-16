#include <iostream>
#include <fstream>
#include <vector>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Dense>
#include <Eigen/Eigen>

using namespace std;
using namespace Eigen;

ros::Publisher pub_cloud, global_map_vis_pub, global_ground_vis_pub;

sensor_msgs::PointCloud2 local_map_pcl;
sensor_msgs::PointCloud2 local_depth_pcl;

ros::Subscriber odom_sub;
ros::Subscriber global_map_sub, global_ground_sub;

ros::Timer local_sensing_timer;

bool has_global_map(false);
bool has_global_ground(false);
bool has_local_map(false);
bool has_odom(false);

nav_msgs::Odometry _odom;

double sensing_horizon, sensing_angle, sensing_rate, estimation_rate;
double _x_size, _y_size, _z_size;
double _gl_xl, _gl_yl, _gl_zl;
double _resolution, _inv_resolution;
int _GLX_SIZE, _GLY_SIZE, _GLZ_SIZE;
std::string map_frame_name;

ros::Time last_odom_stamp = ros::TIME_MAX;

inline Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i &index)
{
    Eigen::Vector3d pt;
    pt(0) = ((double)index(0) + 0.5) * _resolution + _gl_xl;
    pt(1) = ((double)index(1) + 0.5) * _resolution + _gl_yl;
    pt(2) = ((double)index(2) + 0.5) * _resolution + _gl_zl;

    return pt;
};

inline Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d &pt)
{
    Eigen::Vector3i idx;
    idx(0) = std::min(std::max(int((pt(0) - _gl_xl) * _inv_resolution), 0), _GLX_SIZE - 1);
    idx(1) = std::min(std::max(int((pt(1) - _gl_yl) * _inv_resolution), 0), _GLY_SIZE - 1);
    idx(2) = std::min(std::max(int((pt(2) - _gl_zl) * _inv_resolution), 0), _GLZ_SIZE - 1);

    return idx;
};

void rcvOdometryCallbck(const nav_msgs::Odometry &odom)
{
    /*if(!has_global_map)
    return;*/
    has_odom = true;
    _odom = odom;
}

pcl::PointCloud<pcl::PointXYZ> _cloud_all_map, _local_map;
pcl::VoxelGrid<pcl::PointXYZ> _voxel_sampler;
sensor_msgs::PointCloud2 _local_map_pcd;

pcl::search::KdTree<pcl::PointXYZ> _kdtreeLocalMap;
vector<int> _pointIdxRadiusSearch;
vector<float> _pointRadiusSquaredDistance;

pcl::PointCloud<pcl::PointXYZ> cloud_input;
void rcvGlobalPointCloudCallBack(const sensor_msgs::PointCloud2 &pointcloud_map)
{
    if (has_global_map)
        return;

    //transform map to point cloud format
    pcl::fromROSMsg(pointcloud_map, cloud_input);
    has_global_map = has_global_ground && true;
    // for visualize only
    pcl::PointCloud<pcl::PointXYZ> cloud_vis;
    sensor_msgs::PointCloud2 map_vis;
    pcl::PointXYZ pt;
    
    if( (int)cloud_input.points.size() == 0 ) return;

    for (int idx = 0; idx < (int)cloud_input.points.size(); idx++)
    {    
        pt = cloud_input.points[idx];
        Vector3d cor_round = gridIndex2coord(coord2gridIndex(Vector3d(pt.x, pt.y, pt.z)));
        pt.x = cor_round(0);
        pt.y = cor_round(1);
        pt.z = cor_round(2);
        cloud_vis.points.push_back(pt);
    }

    cloud_vis.width    = cloud_vis.points.size();
    cloud_vis.height   = 1;
    cloud_vis.is_dense = true;

    pcl::toROSMsg(cloud_vis, map_vis);

    map_vis.header.frame_id = "/world";
    global_map_vis_pub.publish(map_vis);
}

void rcvGlobalGroundPointCloudCallBack(const sensor_msgs::PointCloud2 &pointcloud_map)
{
    if (has_global_ground)
        return;

    ROS_WARN("Global Pointcloud received..");
    //load global map
    pcl::PointCloud<pcl::PointXYZ> cloudIn;
    pcl::PointXYZ pt_in;
    //transform map to point cloud format
    pcl::fromROSMsg(pointcloud_map, cloudIn);
    for (int i = 0; i < int(cloudIn.points.size()); i++)
    {
        pt_in = cloudIn.points[i];
        cloud_input.push_back(pt_in);
    }
    printf("global map has points: %d.\n", (int)cloud_input.size());

    has_global_ground = true;

    _voxel_sampler.setLeafSize(0.1f, 0.1f, 0.1f);
    _voxel_sampler.setInputCloud(cloud_input.makeShared());
    _voxel_sampler.filter(_cloud_all_map);

    _kdtreeLocalMap.setInputCloud(_cloud_all_map.makeShared());

    has_global_map = has_global_ground && true;

    // for visualize only
    pcl::PointCloud<pcl::PointXYZ> cloud_vis;
    sensor_msgs::PointCloud2 map_vis;
    pcl::PointXYZ pt;
    
    if( (int)cloudIn.points.size() == 0 ) return;

    for (int idx = 0; idx < (int)cloudIn.points.size(); idx++)
    {    
        pt = cloudIn.points[idx];
        Vector3d cor_round = gridIndex2coord(coord2gridIndex(Vector3d(pt.x, pt.y, pt.z)));
        pt.x = cor_round(0);
        pt.y = cor_round(1);
        pt.z = cor_round(2);
        cloud_vis.points.push_back(pt);
    }

    cloud_vis.width    = cloud_vis.points.size();
    cloud_vis.height   = 1;
    cloud_vis.is_dense = true;

    pcl::toROSMsg(cloud_vis, map_vis);

    map_vis.header.frame_id = "/world";
    global_ground_vis_pub.publish(map_vis);
}

void renderSensedPoints(const ros::TimerEvent &event)
{
    if (!has_global_map || !has_odom)
        return;

    Eigen::Quaterniond q;
    q.x() = _odom.pose.pose.orientation.x;
    q.y() = _odom.pose.pose.orientation.y;
    q.z() = _odom.pose.pose.orientation.z;
    q.w() = _odom.pose.pose.orientation.w;

    Eigen::Matrix3d rot;
    rot = q;
    Eigen::Vector3d yaw_vec = rot.col(0);

    _local_map.points.clear();
    pcl::PointXYZ searchPoint(_odom.pose.pose.position.x, _odom.pose.pose.position.y, _odom.pose.pose.position.z);
    _pointIdxRadiusSearch.clear();
    _pointRadiusSquaredDistance.clear();

    pcl::PointXYZ pt;
    if (_kdtreeLocalMap.radiusSearch(searchPoint, sensing_horizon, _pointIdxRadiusSearch, _pointRadiusSquaredDistance) > 0)
    {
        for (size_t i = 0; i < _pointIdxRadiusSearch.size(); ++i)
        {
            pt = _cloud_all_map.points[_pointIdxRadiusSearch[i]];

            if ((fabs(pt.z - _odom.pose.pose.position.z) / (sensing_horizon)) > tan(M_PI / 12.0))
                continue;

            Vector3d pt_vec(pt.x - _odom.pose.pose.position.x, pt.y - _odom.pose.pose.position.y, pt.z - _odom.pose.pose.position.z);

            if (pt_vec.dot(yaw_vec) < 0)
                continue;
            _local_map.points.push_back(pt);
        }
    }
    else
    {
        return;
    }

    _local_map.width = _local_map.points.size();
    _local_map.height = 1;
    _local_map.is_dense = true;

    pcl::toROSMsg(_local_map, _local_map_pcd);
    _local_map_pcd.header.frame_id = map_frame_name;

    pub_cloud.publish(_local_map_pcd);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_render");
    ros::NodeHandle nh("~");

    nh.getParam("sensing_horizon", sensing_horizon);
    nh.getParam("sensing_rate", sensing_rate);
    nh.getParam("estimation_rate", estimation_rate);

    nh.getParam("map/x_size", _x_size);
    nh.getParam("map/y_size", _y_size);
    nh.getParam("map/z_size", _z_size);
    nh.getParam("map_frame_name", map_frame_name);

    nh.param("map/resolution", _resolution, 0.2);

    //subscribe point cloud
    global_map_sub      = nh.subscribe("global_map", 1, rcvGlobalPointCloudCallBack);
    global_ground_sub   = nh.subscribe("global_ground", 1, rcvGlobalGroundPointCloudCallBack);
    odom_sub            = nh.subscribe("odometry", 50, rcvOdometryCallbck);

    //publisher
    pub_cloud               = nh.advertise<sensor_msgs::PointCloud2>("local_pointcloud", 10);
    //pub_vis_cloud           = nh.advertise<sensor_msgs::PointCloud2>("local_map_vis",1);
    global_map_vis_pub      = nh.advertise<sensor_msgs::PointCloud2>("global_map_vis", 1);
    global_ground_vis_pub   = nh.advertise<sensor_msgs::PointCloud2>("global_ground_vis", 1);

    double sensing_duration = 1.0 / sensing_rate * 2.5;

    local_sensing_timer = nh.createTimer(ros::Duration(sensing_duration), renderSensedPoints);

    _inv_resolution = 1.0 / _resolution;

    _gl_xl = -_x_size / 2.0;
    _gl_yl = -_y_size / 2.0;
    _gl_zl = 0.0;

    _GLX_SIZE = (int)(_x_size * _inv_resolution);
    _GLY_SIZE = (int)(_y_size * _inv_resolution);
    _GLZ_SIZE = (int)(_z_size * _inv_resolution);

    ros::Rate rate(100);
    bool status = ros::ok();
    while (status)
    {
        ros::spinOnce();
        status = ros::ok();
        rate.sleep();
    }
}
