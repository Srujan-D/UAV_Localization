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

State::State(MatrixXd stateVector){
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

Vector10d State::stateToMatrix(State st){
    Vector10d vec(10,1);
    vec(0,0) = st.p.x;
    vec(1,0) = st.p.y;
    vec(2,0) = st.p.z;

    vec(3,0) = st.q.x;
    vec(4,0) = st.q.y;
    vec(5,0) = st.q.z;
    vec(6,0) = st.q.w;

    vec(7,0) = st.v.linear.x;
    vec(8,0) = st.v.linear.y;
    vec(9,0) = st.v.linear.z;


    return vec;
}
