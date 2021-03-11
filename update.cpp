#include "update.h"

Eigen::VectorXd Update::y(){
    Eigen::Vector9d y_meas;
    if(isGPS){
        // TODO
    }
    else{
        // TODO
    }
    return y_meas;
}

Eigen::MatrixXd Update::h(){
    Eigen::MatrixXd h_meas;
    if(isGPS){
        // TODO
    }
    else{
        // TODO
    }
    return h_meas;
}

Eigen::MatrixXd Update::H(){
    Eigen::MatrixXd H_meas;
    if(isGPS){
        // TODO
    }
    else{
        // TODO
    }
    return H_meas;
}

Eigen::MatrixXd Update::K(){
    Eigen::MatrixXd _K;
    if(isGPS){
        // TODO
    }
    else{
        // TODO
    }
    return _K;
}

Eigen::MatrixXd Update::R_k(){
    Eigen::MatrixXd R_meas;
    if(isGPS){
        // TODO
    }
    else{
        // TODO
    }
    return R_meas;
}

Eigen::MatrixXd Update::UpdateCovar(){
    Eigen::Dynamic2D I;
    I = Eigen::Dynamic2D::Identity(10,10);
    return (I - Update::K * Update::H) * predict->PredictedCovariance;
}

Eigen::MatrixXd Update::UpdateEst(){
    // TODO: Give arguments to the functions K, y, h
    return predict->current_state + Update::K*(Update::y - Update::h);
}

Eigen::MatrixXd Update::ComputeK(){
    return predict->PredictedCovariance * Update::H * ((Update::H * predict->PredictedCovariance * Update::H.transpose() + Update::R_k).inverse());
}

