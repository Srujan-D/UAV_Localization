#include "sensors.h"

void Sensors::cb_imu_lidar(const sensor_msgs::Imu::ConstPtr& msg_imu, const sensor_msgs::LaserScan::ConstPtr& msg_lidar){

}
void Sensors::cb_imu_gps(const sensor_msgs::Imu::ConstPtr& msg_imu, const sensor_msgs::NavSatFix::ConstPtr& msg_gps){

}


// void Sensors::imu_cb(const sensor_msgs::Imu::ConstPtr& msg_imu){
//     // TODO: Process Sensor data and apply transformation.
//     Sensors::imu_measured_state->v.angular = msg_imu->angular_velocity;
//     Sensors::imu_measured_state->q = msg_imu->orientation;
//     Sensors::imu_measured_state->a.linear = msg_imu->linear_acceleration;
// }

Sensors::Sensors(ros::NodeHandle nh){
    // message_filters::Subscriber<sensor_msgs::NavSatFix> gps_sub(n, "/mavros/gpsstatus/gps1/rtk", 1);
    // imu_sub = nh.subscribe("/mavros/imu/data", 10, imu_cb);

    message_filters::Subscriber<sensor_msgs::NavSatFix> gps_sub(this, "/mavros/gpsstatus/gps1/rtk", 1);
    message_filters::Subscriber<sensor_msgs::LaserScan> lidar_sub(this, "/perception_peeps", 1); // TODO:Find topic name for LIDAR
    message_filters::Subscriber<nav_msgs::Odometry> cam_sub_odom(this, "/odom", 1);
    message_filters::Subscriber<nav_msgs::Imu> cam_sub_imu(this, "/imu/data", 1);

    typedef sync_policies::ApproximateTime<sensor_msgs::Imu, sensor_msgs::LaserScan> custom_sync_lidar;
    message_filters::Synchronizer<custom_sync_lidar> syncApproximate(custom_sync_with_gps(10), imu_sub, lidar_sub);
    syncApproximate.registerCallback(boost::blind(&Sensors::cb_imu_lidar, this);
    
    typedef sync_policies::ApproximateTime<sensor_msgs::Imu, sensor_msgs::NavSatFix> custom_sync_gps;
    message_filters::Synchronizer<custom_sync_gps> syncApproximate(custom_sync_with_gps(10), imu_sub, lidar_sub);
    syncApproximate.registerCallback(boost::blind(&Sensors::cb_imu_gps, this);

}