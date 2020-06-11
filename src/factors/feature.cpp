#include "multirotor_sim/factors/feature.h"
#include "geometry/xform.h"

FeatureFunctor::FeatureFunctor(int feat_id, const Eigen::Matrix3d &K, const Eigen::Matrix3d &R_c_from_b, const Eigen::Vector3d &p_cb,
                               const Eigen::Vector2d &pix_i, const Eigen::Vector2d &pix_j, const Eigen::Matrix3d &Xi):
    _feat_id(feat_id), _K(K), _R_c_from_b(R_c_from_b), _p_c_wrt_b(p_cb), _pix_i(pix_i), _pix_j(pix_j), _Xi(Xi) {}

void FeatureFunctor::update(const Eigen::Vector2d &pix)
{
    _pix_i = pix;
}

Eigen::Vector3d FeatureFunctor::backProjectionFunction(const Eigen::Vector2d &pix) const //This may need to be templated
{
    // Homogeneous Pixel coordinates
    Eigen::Vector3d mi; 
    mi << pix(0), pix(1), 1.0;

    // Make calibrated homogeneous pixel coordinates
    Eigen::Vector3d eps_i = _K.inverse() * mi;

    // Return the unit vector pointing at the featur from the camera
    return eps_i / eps_i.norm();
}

template<typename T>
bool FeatureFunctor::operator()(const T* _xi, const T*_xj, const T* lambda_i, T* residual) const
{
    using Vec3T = Eigen::Matrix<T, 3, 1>;
    xform::Xform<T> xi(_xi), xj(_xj);
    Eigen::Vector3d Phat_j = backProjectionFunction(_pix_j);
    Eigen::Vector3d Phat_i = backProjectionFunction(_pix_i);
    Vec3T Pest_j;

    //Put Pest_j in the body i frame
    Pest_j = 1.0/(*lambda_i) * _R_c_from_b.transpose() * Phat_i + _p_c_wrt_b;
    // Put Pest_j in the inertial frame
    Pest_j = xi.q().rota(Pest_j) + xi.t() - xj.t();
    // Put Pest_j in body j frame
    Pest_j = xj.q().rotp(Pest_j) - _p_c_wrt_b;
    //Put Pest_j in the camera frame
    Pest_j = _R_c_from_b * Pest_j;
    Pest_j = Pest_j / Pest_j.norm(); //.template norm()??

    //Calculate b1 and b2 vectors
    Eigen::Vector3d e1(0., 0., 1.);
    Eigen::Vector3d b1 = Phat_j.cross(e1);
    b1 = b1 / b1.norm();
    Eigen::Vector3d b2 = Phat_j.cross(b1);
    b2 = b2 / b2.norm();
    Eigen::Matrix<double, 2, 3> B;
    B.row(0) = b1;
    B.row(1) = b2;

    Eigen::Map<Eigen::Matrix<T, 3, 1>> r(residual);
    r = _Xi * B * (Phat_j - Pest_j);

    return true;
}