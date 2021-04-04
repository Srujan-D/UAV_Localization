#include "update.h"

VectorXd Update::y(sensor_msgs::Imu msg_imu, sensor_msgs::Imu msg_cam_imu, nav_msgs::Odometry msg_cam_odom, sensor_msgs::LaserScan msg_lidar, sensor_msgs::NavSatFix msg_gps)
{
    string ongoing = "lidar";
    if (msg_gps.header.time < msg_imu.header.time)
    {
        // TODO
        Update::y_meas << msg_gps.latitude, msg_gps.longitude, msg_gps.altitude, msg_cam_odom.twist.twist.linear.x, msg_cam_odom.twist.twist.linear.y, msg_cam_odom.twist.twist.linear.z, msg_cam_imu.linear_acceleration.x, msg_cam_imu.linear_acceleration.y, msg_cam_imu.linear_acceleration.z;
    }
    else
    {
        // TODO
        Update::y_meas << msg_cam_odom.pose.pose.position.x, msg_cam_odom.pose.pose.position.y, msg_cam_odom.pose.pose.position.z, msg_cam_odom.twist.twist.linear.x, msg_cam_odom.twist.twist.linear.y, msg_cam_odom.twist.twist.linear.z, msg_cam_imu.linear_acceleration.x, msg_cam_imu.linear_acceleration.y, msg_cam_imu.linear_acceleration.z;
        
    }
    return Update::y_meas;
}

VectorXd Update::y_gps(sensor_msgs::Imu msg_imu, sensor_msgs::Imu msg_cam_imu, nav_msgs::Odometry msg_cam_odom, sensor_msgs::LaserScan msg_lidar, sensor_msgs::NavSatFix msg_gps)
{

    if (msg_gps.header.time < msg_imu.header.time)
    {
        // TODO

        Update::y_meas << msg_gps.latitude, msg_gps.longitude, msg_gps.altitude, msg_cam_odom.twist.twist.linear.x, msg_cam_odom.twist.twist.linear.y, msg_cam_odom.twist.twist.linear.z, msg_cam_imu.linear_acceleration.x, msg_cam_imu.linear_acceleration.y, msg_cam_imu.linear_acceleration.z;
    }
    return Update::y_meas;
}

VectorXd Update::y_lidar(sensor_msgs::Imu msg_imu, sensor_msgs::Imu msg_cam_imu, nav_msgs::Odometry msg_cam_odom, sensor_msgs::LaserScan msg_lidar, sensor_msgs::NavSatFix msg_gps)
{
    if (msg_lidar.header.time < msg_lidar.header.time)
    {
        // TODO
        y_meas << msg_cam_odom.pose.pose.position.x, msg_cam_odom.pose.pose.position.y, msg_cam_odom.pose.pose.position.z, msg_cam_odom.twist.twist.linear.x, msg_cam_odom.twist.twist.linear.y, msg_cam_odom.twist.twist.linear.z, msg_cam_imu.linear_acceleration.x, msg_cam_imu.linear_acceleration.y, msg_cam_imu.linear_acceleration.z;
    }
    return y_meas;
}

MatrixXd Update::h()
{
    MatrixXd h_meas;
    if (isGPS)
    {
        // TODO
    }
    else
    {
        // TODO
    }
    return h_meas;
}

MatrixXd Update::H()
{
    MatrixXd H_meas;
    H_meas << 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    return H_meas;
}

MatrixXd Update::K()
{
    MatrixXd _K;
    _K = Update::ComputeK();
    return _K;
}

MatrixXd Update::R_k()
{
    Dynamic2D R_meas;
    R_meas = 0.05 * Dynamic2D::Identity(9, 9);
    return R_meas;
}

MatrixXd Update::UpdateCovar(MatrixXd K)
{
    Dynamic2D I;
    I = Dynamic2D::Identity(10, 10);
    return (I - K * Update::H) * predict->PredictedCovariance;
}

MatrixXd Update::UpdateEst(MatrixXd K, sensor_msgs::Imu msg_imu, sensor_msgs::Imu msg_cam_imu, nav_msgs::Odometry msg_cam_odom, sensor_msgs::LaserScan msg_lidar, sensor_msgs::NavSatFix msg_gps)
{
    // TODO: Give arguments to the functions K, y, h
    Vector10d temp;
    // MatrixXd K = Update::ComputeK();
    temp = State::stateToMatrix(predict->current_state) + K * (Update::y_lidar(sensor_msgs::Imu msg_imu, sensor_msgs::Imu msg_cam_imu, nav_msgs::Odometry msg_cam_odom, sensor_msgs::LaserScan msg_lidar, sensor_msgs::NavSatFix msg_gps) - Update::H * State::stateToMatrix(predict->current_state));
    temp = temp + K * (Update::y_gps(sensor_msgs::Imu msg_imu, sensor_msgs::Imu msg_cam_imu, nav_msgs::Odometry msg_cam_odom, sensor_msgs::LaserScan msg_lidar, sensor_msgs::NavSatFix msg_gps) - Update::H * temp);
    return temp;
}

MatrixXd Update::ComputeK()
{
    return predict->PredictedCovariance * Update::H * ((Update::H * predict->PredictedCovariance * Update::H.transpose() + Update::R_k).inverse());
}