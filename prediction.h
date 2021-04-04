#include "state.h"


class Prediction{
    // private:

    public:
        State* prev_state = State(), current_state = State(), new_state = State(), imu_measured_state = State();
        sensor_msgs::LaserScan::ConstPtr scan;
        geometry_msgs::Odometry::CosntPtr cam;
        sensor_msgs::Imu::CosntPtr imu;

       MatrixXd noiseQk =MatrixXd::Zero(10,10);
       MatrixXd Covariance_KplusOne =MatrixXd::Zero(10,10);
       MatrixXd Covariance_K =MatrixXd::Zero(10,10);

    // public:
        Prediction();
        ~Prediction();
        Matrix3d R_q(State *s);
        State* f(State* prev_state, State* current_state, State* imu_measured_state);
        MatrixXd F(State* prev_state, State* current_state);
        MatrixXd PredictedCovariance(State* prev_state, State* current_state);
        void predict(State* prev_state, State* current_state);
};


Prediction::Prediction(/* args */)
{
}

Prediction::~Prediction()
{
}
