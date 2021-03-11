#include "prediction.h"

Eigen::Matrix3d Prediction::R_q(State *s){
    geometry_msgs::Quaternion q = s->q;
    Eigen::Matrix3d rot_q;
    rot_q(0,0) = q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z;
    rot_q(0,1) = 2*(q.x*q.y + q.w*q.z);
    rot_q(0,2) = 2*(q.x*q.z - q.w*q.y);
    rot_q(1,0) = 2*(q.x*q.y - q.w*q.z);
    rot_q(1,1) = q.w*q.w - q.x*q.x + q.y*q.y - q.z*q.z;
    rot_q(1,2) = 2*(q.w*q.x + q.y*q.z);
    rot_q(2,0) = 2*(q.x*q.z + q.w*q.y);
    rot_q(2,1) = 2*(-q.w*q.x + q.y*q.z);
    rot_q(2,2) = q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z;

    return rot_q;
}

State* Prediction::f(State* prev_state, State* current_state, State* imu_measured_state){
    State* new_state = State();
    
    float d_time = ros::Time::now().toSec() - current_state->time_sec.toSec() ;
    
    Eigen::Vector3d a_imu;
    a_imu << imu_measured_state->a.linear.x, imu_measured_state->a.linear.y, imu_measured_state->a.linear.z;

    Eigen::Matrix3d d_vel;
    d_vel = Prediction::R_q(current_state) * a_imu;

    new_state->p.x = prev_state->p.x + current_state->v.linear.x * d_time + 0.5 * pow(d_time,2)*(d_vel(0,0));
    new_state->p.y = prev_state->p.y + current_state->v.linear.y * d_time + 0.5 * pow(d_time,2)*(d_vel(1,0));
    new_state->p.z = prev_state->p.z + current_state->v.linear.z * d_time + 0.5 * pow(d_time,2)*(d_vel(2,0) + imu_measured_state->g);  

    
    new_state->v.linear.x = prev_state->v.linear.x + d_vel(0,0) * d_time;
    new_state->v.linear.y = prev_state->v.linear.y + d_vel(1,0) * d_time;
    new_state->v.linear.z = prev_state->v.linear.z + (d_vel(2,0) + imu_measured_state->g)* d_time;

    Eigen::Vector4d quat;
    quat << current_state->q.w, current_state->q.x, current_state->q.y, current_state->q.z;

    Eigen::Vector3d angvel_imu;
    angvel_imu << imu_measured_state->v.angular.x, imu_measured_state->v.angular.y, imu_measured_state->v.angular.z;


    Eigen::Matrix4d _S;
    _S <<   0, angvel_imu(0,0), angvel_imu(1,0), angvel_imu(2,0),
            angvel_imu(0,0), 0, angvel_imu(2,0), -angvel_imu(1,0),
            angvel_imu(1,0), -angvel_imu(2,0), 0, angvel_imu(1,0),
            angvel_imu(2,0), angvel_imu(1,0), -angvel_imu(0,0), 0;

    Eigen::Vector4d new_quat;
    new_quat = _S * quat;

    new_state->q.w = -0.5 * new_quat(0,0);
    new_state->q.x = 0.5 * new_quat(1,0);
    new_state->q.y = 0.5 * new_quat(2,0);
    new_state->q.z = 0.5 * new_quat(3,0);        

    return new_state;
}

Eigen::Matrix10d Prediction::F(State* prev_state, State* current_state){
    float t = current_state->time_sec - prev_state->time_sec;
    if(!t)  t = 0.001; // t = 0 sec   
    
    Eigen::Matrix3d a_imu_x;
    a_imu_x <<  0, -imu_measured_state->a.linear.z-imu_measured_state->g, imu_measured_state->a.linear.y,
                imu_measured_state->a.linear.z+imu_measured_state->g, 0, -imu_measured_state->a.linear.x,
                -imu_measured_state->a.linear.y, imu_measured_state->a.linear.x, 0;

    Eigen::Matrix3d w_imu;
    w_imu << imu_measured_state->a.angular.x, 

    Eigen::Matrix3d J_q;
    J_q = -Prediction::R_q(current_state) * a_imu_x * t;
    State* temp = imu_measured_state;

    float mod_w = sqrt(imu_measured_state->v.angular.x*imu_measured_state->v.angular.x + imu_measured_state->v.angular.y*imu_measured_state->v.angular.y + imu_measured_state->v.angular.z*imu_measured_state->v.angular.z);
    temp->q.w = cos(mod_w*t*0.5)
    temp->q.x = imu_measured_state->v.angular.x*sin(mod_w*t*0.5)/mod_w;
    temp->q.y = imu_measured_state->v.angular.y*sin(mod_w*t*0.5)/mod_w;
    temp->q.z = imu_measured_state->v.angular.z*sin(mod_w*t*0.5)/mod_w;
    J_q3 = Prediction::R_q(temp).transpose();
    
    // Check the jacobian matrix, especially for quaternion equations.
    
    Eigen::Matrix10d F;
    F <<    1, 0, 0, t, 0, 0, 0, 0, 0, 0,
            0, 1, 0, 0, t, 0, 0, 0, 0, 0,
            0, 0, 1, 0, 0, t, 0, 0, 0, 0,
            0, 0, 0, 1, 0, 0, J_q(0,0), J_q(0,1), J_q(0,2), 0,
            0, 0, 0, 0, 1, 0, J_q(1,0), J_q(1,1), J_q(1,2), 0,
            0, 0, 0, 0, 0, 1, J_q(2,0), J_q(2,1), J_q(2,2), 0,
            0, 0, 0, 0, 0, 0, J_q3(0,0), J_q3(0,1), J_q3(0,2), 0,
            0, 0, 0, 0, 0, 0, J_q3(1,0), J_q3(1,1), J_q3(1,2), 0,
            0, 0, 0, 0, 0, 0, J_q3(2,0), J_q3(2,1), J_q3(2,2), 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 1;

    return F;
}

Eigen::Matrix10d Prediction::PredictedCovariance(State* prev_state, State* current_state){
    noiseQk <<  0.5, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0.5, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0.5, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0.5, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0.5, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0.5, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0.5, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0.5, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0.5, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0.5;

                
        Eigen::Matrix10d jacobi = F(prev_state,new_state);
        Eigen::Matrix10d jacobiTransposed= jacobi.transpose();

        return jacobi * Covariance_P * jacobiTransposed + noiseQk;
}

void Prediction::predict(State* prev_state, State* current_state){
    new_state = Prediction::f(prev_state, current_state);
    Covariance_KplusOne = Prediction::PredictedCovariance(prev_state, current_state);
}

