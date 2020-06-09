#ifndef FEATURE
#define FEATURE

#include <Eigen/Core>
#include "multirotor_sim/factors/shield.h"

class FeatureFunctor
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
    FeatureFunctor(const Eigen::Matrix3d &K, const Eigen::Matrix3d &R_c_from_b, const Eigen::Vector3d &p_cb,
                   const Eigen::Vector2d &pix_i, const Eigen::Vector2d &pix_j, const Eigen::Matrix3d &Xi);

    Eigen::Vector3d backProjectionFunction(const Eigen::Vector2d &pix);

    void update(const Eigen::Vector2d &pix); //Update pixel location when frame i leaves the sliding window

    template<typename T> //For ceres optimizer
    bool operator()(const T* _xi, const T* _xj, const T* lambda_i, T* residuals) const;

private:
    Eigen::Matrix3d _K;
    Eigen::Matrix3d _R_c_from_b;
    Eigen::Vector3d _p_c_wrt_b;
    Eigen::Vector2d _pix_i;
    Eigen::Vector2d _pix_j;
    Eigen::Matrix3d _Xi; // square root of information matrix
};

#endif //FEATURE