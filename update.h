#include "prediction.h"

class Update{
    // private:
    public:
        Prediction* predict;
        bool isGPS = false;
    // public:
        Update(/* args */);
        ~Update();
        Eigen::MatrixXd H();
        Eigen::MatrixXd K();
        Eigen::VectorXd y();
        Eigen::MatrixXd h();
        Eigen::MatrixXd R_k();
        Eigen::MatrixXd UpdateCovar();
        Eigen::MatrixXd UpdateEst();
        Eigen::MatrixXd ComputeK();
};

Update::Update(/* args */)
{
}

Update::~Update()
{
}

