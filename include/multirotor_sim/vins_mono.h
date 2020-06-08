#ifndef VINS_MONO
#define VINS_MONO

#include "multirotor_sim/state.h"
#include "multirotor_sim/estimator_base.h"
#include "multirotor_sim/state.h"
#include "multirotor_sim/factors/imu.h"

#include <eigen3/Eigen/Core>
#include <vector>
#include <deque>

namespace ms = multirotor_sim;

class VinsMono : public multirotor_sim::EstimatorBase
{
public:
    VinsMono();

    void imuCallback(const double &t, const Vector6d &z, const Matrix6d &R);
    void imageCallback(const double& t, const ms::ImageFeat& z, const Eigen::Matrix2d& R_pix, const Matrix1d& R_depth);

private:
    ms::State _state;
    std::vector<ms::State> _state_hist;
    double _current_t;
    std::deque<ImuFunctor> _imu;
    //What to use for camera vectors. Also how to tell if features are the same from
    //frame to frame...
};

#endif