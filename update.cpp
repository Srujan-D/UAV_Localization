#include "update.h"

VectorXd Update::y(){
    Vector9d y_meas;
    if(isGPS){
        // TODO
    }
    else{
        // TODO
    }
    return y_meas;
}

MatrixXd Update::h(){
    MatrixXd h_meas;
    if(isGPS){
        // TODO
    }
    else{
        // TODO
    }
    return h_meas;
}

MatrixXd Update::H(){
    MatrixXd H_meas;
    if(isGPS){
        // TODO
    }
    else{
        // TODO
    }
    return H_meas;
}

MatrixXd Update::K(){
    MatrixXd _K;
    if(isGPS){
        // TODO
    }
    else{
        // TODO
    }
    return _K;
}

MatrixXd Update::R_k(){
    MatrixXd R_meas;
    if(isGPS){
        // TODO
    }
    else{
        // TODO
    }
    return R_meas;
}

MatrixXd Update::UpdateCovar(){
    Dynamic2D I;
    I = Dynamic2D::Identity(10,10);
    return (I - Update::K * Update::H) * predict->PredictedCovariance;
}

MatrixXd Update::UpdateEst(){
    // TODO: Give arguments to the functions K, y, h
    return predict->current_state + Update::K*(Update::y - Update::h);
}

MatrixXd Update::ComputeK(){
    return predict->PredictedCovariance * Update::H * ((Update::H * predict->PredictedCovariance * Update::H.transpose() + Update::R_k).inverse());
}

