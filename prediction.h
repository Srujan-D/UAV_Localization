#include "state.h"


class Prediction{
    // private:

    public:
        State* prev_state = State(), current_state = State(), new_state = State(), imu_measured_state = State();
        sensor_msgs::LaserScan::ConstPtr scan;
        geometry_msgs::Odometry::CosntPtr cam;
        sensor_msgs::Imu::CosntPtr imu;

        Eigen::MatrixXd noiseQk = Eigen::MatrixXd::Zero(10,10);
        Eigen::MatrixXd Covariance_KplusOne = Eigen::MatrixXd::Zero(10,10);
        Eigen::MatrixXd Covariance_K = Eigen::MatrixXd::Zero(10,10);

    // public:
        Prediction();
        ~Prediction();
        Eigen::Matrix3d R_q(State *s);
        State* f(State* prev_state, State* current_state, State* imu_measured_state);
        Eigen::MatrixXd F(State* prev_state, State* current_state);
        Eigen::MatrixXd PredictedCovariance(State* prev_state, State* current_state);
        void predict(State* prev_state, State* current_state);
};


Prediction::Prediction(/* args */)
{
}

Prediction::~Prediction()
{
}
