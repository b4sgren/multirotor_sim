#ifndef VINS_MONO
#define VINS_MONO

#include "multirotor_sim/state.h"
#include "multirotor_sim/estimator_base.h"
#include "multirotor_sim/state.h"
#include "multirotor_sim/factors/imu.h"
#include "multirotor_sim/factors/feature.h"

#include <eigen3/Eigen/Core>
#include <vector>
#include <deque>
#include <map>
#include <string>
#include <ceres/ceres.h>

namespace ms = multirotor_sim;

struct Feature
{
    //The values in _pix_i, _lambda_i and _node_i are not in the history
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Feature() = default;
    Feature(const Eigen::Vector2d &pix, double lamb, int node): _pix_i(pix), _lambda_i(lamb), _node_i(node) {}
    
    void append(const Eigen::Vector2d &pix, double lamb, int node)
    {
        _pix_hist.push_back(pix);
        _lambda_hist.push_back(lamb);
        _node_hist.push_back(node);
    }

    void updateOriginInfo()
    {
        _pix_i = _pix_hist.front();
        _lambda_i = _lambda_hist.front();
        _node_i = _node_hist.front();
    }

    void remove()
    {
        if(_node_hist.size() > 0)
        {
            _pix_hist.pop_front();
            _lambda_hist.pop_front();
            _node_hist.pop_front();
        }
    }

    bool isTracked()
    {
        return _node_hist.size() >= 1 ? true : false;
    }

    Eigen::Vector2d _pix_i;
    double _lambda_i;
    int _node_i;
    std::deque<Eigen::Vector2d> _pix_hist;
    std::deque<double> _lambda_hist;
    std::deque<int> _node_hist;
};

class VinsMono : public multirotor_sim::EstimatorBase
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
    VinsMono(const std::string &filename);

    void imuCallback(const double &t, const Vector6d &z, const Matrix6d &R);
    void imageCallback(const double& t, const ms::ImageFeat& z, const Eigen::Matrix2d& R_pix, const Matrix1d& R_depth);

private:
    void optimize();
    void addParameterBlocks(ceres::Problem &problem);
    void addImuFactors(ceres::Problem &problem);
    void addFeatureFactors(ceres::Problem &problem);

    Eigen::Matrix3d _K;
    Eigen::Matrix3d _R_c_from_b;
    Eigen::Vector3d _p_b_cb;
    int _counter;
    int _current_kf_id;
    ms::State _state;
    std::vector<ms::State> _state_hist;
    double _current_t;
    std::deque<ImuFunctor> _imu;
    std::deque<std::vector<FeatureFunctor>> _feature_funcs; 
    std::map<int, Feature> _features;
};

#endif