#include "state.h"

State::State(){
    this->p = geometry_msgs::Point();
    this->q = geometry_msgs::Quaternion();    
    this->v = geometry_msgs::Twist();    
    this->time_sec = ros::Time::now();
    
}
	
State::State(geometry_msgs::Point p, geometry_msgs::Quaternion q, geometry_msgs::Twist v, ros::Time time_now = ros::Time::now()){
    this->p = p;
    this->q = q;
    this->v = v;
    this->time_now = time_now;
} 

State::State(Eigen::MatrixXd stateVector){
    this->p.x = stateVector(0,0);
    this->p.y = stateVector(1,0);
    this->p.z = stateVector(2,0);

    this->q.w = stateVector(3,0);
    this->q.x = stateVector(4,0);
    this->q.y = stateVector(5,0);
    this->q.z = stateVector(6,0);

    this->v.linear.x = stateVector(7,0);
    this->v.linear.y = stateVector(8,0);
    this->v.linear.z = stateVector(9,0);

} 

Eigen::Vector10d State::stateToMatrix(){
    Eigen::Vector10d vec(10,1);
    vec(0,0) = p.x;
    vec(1,0) = p.y;
    vec(2,0) = p.z;

    vec(3,0) = q.x;
    vec(4,0) = q.y;
    vec(5,0) = q.z;
    vec(6,0) = q.w;

    vec(7,0) = v.linear.x;
    vec(8,0) = v.linear.y;
    vec(9,0) = v.linear.z;


    return vec;
}
