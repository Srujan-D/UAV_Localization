#include "update.h"

class Sensors{
    // private:
    public:

        // bool isGPS = false;
        // void mf_cb_with_gps(const sensor_msgs::Imu::ConstPtr& msg_cam_imu, const nav_msgs::Odometry::ConstPtr& msg_cam_odom, const sensor_msgs::LaserScan::ConstPtr& msg_lidar, const sensor_msgs::NavSatFix::ConstPtr& msg_gps);    
        // void mf_cb_without_gps(const sensor_msgs::Imu::ConstPtr& msg_cam_imu, const nav_msgs::Odometry::ConstPtr& msg_cam_odom, const sensor_msgs::LaserScan::ConstPtr& msg_lidar);    
        // void imu_cb(const sensor_msgs::Imu::ConstPtr& msg_imu);
        void cb_imu_lidar(const sensor_msgs::Imu::ConstPtr& msg_imu, const sensor_msgs::LaserScan::ConstPtr& msg_lidar);
        void cb_imu_gps(const sensor_msgs::Imu::ConstPtr& msg_imu, const sensor_msgs::NavSatFix::ConstPtr& msg_gps);
        // Declare EKF object.
    
    // public:
        ros::Subscriber imu_sub;
        ros::Subscriber lidar_sub;
        ros::Subscriber cam_sub;
        ros::Subscriber gps_sub;

        sensor_msgs::Imu cam_imu;
        nav_msgs::Odometry cam_odom;
        sensor_msgs::LaserScan lidar_scan;
        sensor_msgs::NavSatFix gps_data;

        sensor_msgs::Imu prev_cam_imu;
        nav_msgs::Odometry prev_cam_odom;
        sensor_msgs::LaserScan prev_lidar_scan;
        sensor_msgs::NavSatFix prev_gps_data;

        Update* update;

        Sensors(ros::NodeHandle nh);
        State* imu_measured_state;

};
