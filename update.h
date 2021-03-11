#include "prediction.h"

class Update{
    private:
        /* data */
        Prediction* predict;
        bool isGPS = false;
    public:
        Update(/* args */);
        ~Update();
        MatrixXd H();
        MatrixXd K();
        VectorXd y();
        MatrixXd h();
        MatrixXd R_k();
        MatrixXd UpdateCovar();
        MatrixXd UpdateEst();
        MatrixXd ComputeK();
};

Update::Update(/* args */)
{
}

Update::~Update()
{
}

