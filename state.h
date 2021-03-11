#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
// #include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Accel.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/CameraInfo"
#include "nav_msgs/Odometry.h"
#include "ros/time.h"
#include <sstream>
#include <vector>
#include <Eigen/Dense>
#define _USE_MATH_DEFINES
#include <math.h> 
// #include <pcl_ros/point_cloud.h>
#include <iostream>
#include <fstream>

using namespace laser_assembler;
using Eigen::MatrixXd;
using Eigen::MatrixXf;
using Eigen::VectorXd;

class State{
    private:
        // float x, y, z, qx, qy, qz, qw, vx, vy, vz, ax, ay, az, avx, avy, avz, g, time_sec; // roll, pitch, yaw;
        
        geometry_msgs::Point p;
        geometry_msgs::Quaternion q;
        geometry_msgs::Twist v;
        geometry_msgs::Accel a;

        geometry_msgs::Vector3 lin_vel = v.linear;
        geometry_msgs::Vector3 ang_vel = v.angular;
        geometry_msgs::Vector3 lin_acc = a.linear;
        geometry_msgs::Vector3 ang_vel = a.angular;
        
        ros::Time time_sec = ros::Time::now().toSec;
        ros::Time time;

    public:
    
        State();
        ~State();

        State(geometry_msgs::Point p, geometry_msgs::Quaternion q, geometry_msgs::Twist v, ros::Time time_now = ros::Time::now());

        State(MatrixXd stateVector);

        MatrixXd stateToMatrix();

};

State::~State(){

}