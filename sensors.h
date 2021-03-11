#include "update.h"

class Sensors{
    // private:
    public:
        void Sensors::mf_cb(const sensor_msgs::CameraInfo::ConstPtr& msg_cam, const sensor_msgs::LaserScan::ConstPtr& msg_lidar, const sensor_msgs::NavSatFix::ConstPtr& msg_gps);    
        void imu_cb(const sensor_msgs::Imu::ConstPtr& msg_imu);
        // Declare EKF object.
    
    // public:
        ros::Subscriber imu_sub;
        ros::Subscriber lidar_sub;
        ros::Subscriber cam_sub;
        ros::Subscriber gps_sub;
        Sensors(ros::NodeHandle nh);
        State* imu_measured_state;

};
