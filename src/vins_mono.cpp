#include "multirotor_sim/vins_mono.h"

VinsMono::VinsMono()
{
}

void VinsMono::imuCallback(const double &t, const Vector6d &z, const Matrix6d &R) //Take from James Jackson's MHE on MQCE
{
    _current_t = t;
    if(_imu.empty())
        return;

    ImuFunctor &imu(_imu.back());

    imu.integrate(t, z, R);
    imu.estimateXj(_state_hist[imu.from_idx_].X.data(), _state_hist[imu.from_idx_].v.data(), _state.X.data(), _state.v.data()); //See https://magiccvs.byu.edu/gitlab/lab/mqce_software_examples/-/blob/master/src/mhe/mhe.cpp
}
    
void VinsMono::imageCallback(const double& t, const ms::ImageFeat& z, const Eigen::Matrix2d& R_pix, const Matrix1d& R_depth)
{
}