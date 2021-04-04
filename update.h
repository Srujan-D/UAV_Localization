#include "prediction.h"

class Update{
    // private:
    public:
        Prediction* predict;
        VectorXd *y_meas;
        // bool isGPS = false;
    // public:
        Update(/* args */);
        ~Update();
        MatrixXd H();
        MatrixXd H();
        MatrixXd K();
        VectorXd y(sensor_msgs::Imu msg_imu, sensor_msgs::Imu msg_cam_imu, nav_msgs::Odometry msg_cam_odom, sensor_msgs::LaserScan msg_lidar, sensor_msgs::NavSatFix msg_gps);
        VectorXd y_lidar(sensor_msgs::Imu msg_imu, sensor_msgs::Imu msg_cam_imu, nav_msgs::Odometry msg_cam_odom, sensor_msgs::LaserScan msg_lidar, sensor_msgs::NavSatFix msg_gps);
        VectorXd y_gps(sensor_msgs::Imu msg_imu, sensor_msgs::Imu msg_cam_imu, nav_msgs::Odometry msg_cam_odom, sensor_msgs::LaserScan msg_lidar, sensor_msgs::NavSatFix msg_gps);
        MatrixXd h();
        MatrixXd R_k();
        MatrixXd UpdateCovar();
        MatrixXd UpdateEst(sensor_msgs::Imu msg_imu, sensor_msgs::Imu msg_cam_imu, nav_msgs::Odometry msg_cam_odom, sensor_msgs::LaserScan msg_lidar, sensor_msgs::NavSatFix msg_gps);
        MatrixXd ComputeK();
};

Update::Update(/* args */)
{
}

Update::~Update()
{
}

