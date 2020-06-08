#include "multirotor_sim/vins_mono.h"

VinsMono::VinsMono()
{
}

void VinsMono::imuCallback(const double &t, const Vector6d &z, const Matrix6d &R)
{
}
    
void VinsMono::imageCallback(const double& t, const ms::ImageFeat& z, const Eigen::Matrix2d& R_pix, const Matrix1d& R_depth)
{
}