#include "sensors.h"


void Sensors::mf_cb(const sensor_msgs::CameraInfo::ConstPtr& msg_cam, const sensor_msgs::LaserScan::ConstPtr& msg_lidar, const sensor_msgs::NavSatFix::ConstPtr& msg_gps){
// TODO: Process Sensor data and apply transformation.
// ensor_read will be EKF class ka object.
    // sensor_read.scan = msg_lidar;
    // sensor_read.cam = msg_cam;
    // sensor_read.gps = msg_gps;
}

void Sensors::imu_cb(const sensor_msgs::Imu::ConstPtr& msg_imu){
    // TODO: Process Sensor data and apply transformation.
    Sensors::imu_measured_state->v.angular = msg_imu->angular_velocity;
    Sensors::imu_measured_state->q = msg_imu->orientation;
    Sensors::imu_measured_state->a.linear = msg_imu->linear_acceleration;
}

Sensors::Sensors(ros::NodeHandle nh){
    // message_filters::Subscriber<sensor_msgs::NavSatFix> gps_sub(n, "/mavros/gpsstatus/gps1/rtk", 1);
    imu_sub = nh.subscribe("/mavros/imu/data", 10, imu_cb);
    message_filters::Subscriber<sensor_msgs::NavSatFix> gps_sub(this, "/mavros/gpsstatus/gps1/rtk", 1);
    message_filters::Subscriber<sensor_msgs::LaserScan> lidar_sub(this, "/mavros/lidar", 1); // TODO:Find topic name for LIDAR
    message_filters::Subscriber<nav_msgs::Odometry> cam_sub(this, "/depth/camera_info", 1);
    typedef sync_policies::ApproximateTime<geometry_msgs::Odometry, sensor_msgs::LaserScan, sensor_msgs::NavSatFix> custom_sync;
    message_filters::Synchronizer<custom_sync> syncApproximate(custom_sync(10), cam_sub, lidar_sub, gps_sub);
    syncApproximate.registerCallback(boost::blind(&Sensors::mf_cb, this);

}