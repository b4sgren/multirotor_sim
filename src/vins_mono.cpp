#include "multirotor_sim/vins_mono.h"
#include <algorithm>
#include <utility>
#include <ceres/ceres.h>
#include "multirotor_sim/utils.h"

VinsMono::VinsMono(const std::string &filename)
{
    Eigen::Vector2d f, center;
    get_yaml_eigen("focal_len", filename, f);
    get_yaml_eigen("cam_center", filename, center);
    get_yaml_eigen("p_b_c", filename, _p_b_cb);
    get_yaml_eigen("R_b_c", filename, _R_c_from_b);
    get_yaml_eigen("x0", filename, _state.arr);
    _K << f(0), 0, center(0), 0, f(1), center(1), 0, 0, 1;
    _counter = 0;
    _current_kf_id = 0;
    _current_t = 0.0;
}

void VinsMono::imuCallback(const double &t, const Vector6d &z, const Matrix6d &R) //Taken from James Jackson's MHE on MQCE
{
    _current_t = t;
    if(_imu.empty())
        return;

    ImuFunctor &imu(_imu.back());

    imu.integrate(t, z, R);
    // Estimate state by propagating using IMU
    imu.estimateXj(_state_hist[imu.from_idx_].X.data(), _state_hist[imu.from_idx_].v.data(), _state.X.data(), _state.v.data()); //See https://magiccvs.byu.edu/gitlab/lab/mqce_software_examples/-/blob/master/src/mhe/mhe.cpp
}

void VinsMono::optimize()
{

}
    
void VinsMono::imageCallback(const double& t, const ms::ImageFeat& z, const Eigen::Matrix2d& R_pix, const Matrix1d& R_depth)
{
    Eigen::Matrix3d I{Eigen::Matrix3d::Identity()};
    std::vector<FeatureFunctor> funcs;
    for(int i{0}; i < z.pixs.size(); ++i)
    {
        int feat_id = z.feat_ids[i];
        double lambda = 1.0/z.depths[i];
        Eigen::Vector2d pix = z.pixs[i];
        if(_features.find(feat_id) != _features.end())
        {
            _features[feat_id].append(pix, lambda, feat_id); //If this is not a keyframe then we will need to remove it afterwards
            FeatureFunctor temp(feat_id, _K, _R_c_from_b, _p_b_cb, _features[feat_id]._pix_i, pix, I); //Calculate covariance matrix
            funcs.push_back(temp); //Calculate covariance matrix
        }
        else  //Feature is a new feature
        {
            Feature feat = Feature(pix, lambda, feat_id);
            _features.insert(std::pair<int, Feature>(feat_id, feat));
        }
    }
    //Call to run optimization here
    //optimize();

    //Is it a keyframe
    if(!(_counter % 3 == 0 && _counter != 0))
    {
        for(auto id : z.feat_ids)
        {
            _features[id].remove();
        }
        ++_current_kf_id;
    }
    else
    {
        // Append the current feature info to the functors
        _feature_funcs.push_back(funcs);
        if(_feature_funcs.size() > 30) //30 nodes is max for the optimization. If more than 30: trim
        {
            std::vector<FeatureFunctor> node_im1{_feature_funcs.front()};
            _feature_funcs.pop_front();
            for(auto functor : node_im1)
            {
                int feat_id{functor.getId()};
                _features[feat_id].updateOriginInfo();
                _features[feat_id].remove();
                if(!_features[feat_id].isTracked())
                    _features.erase(feat_id);
            }
            //Iterate over the nodes.
            for(auto it{_feature_funcs.begin()}; it != _feature_funcs.end(); ++it) //Is there a better way to do this. more efficient way??
            {
                for(auto f : (*it))
                {
                    for(auto functor : node_im1)
                    {
                        int feat_id{functor.getId()};
                        if(f.getId() == feat_id) //if the feature appears in a future keyframe then update the original location
                        {
                            f.update(_features[feat_id]._pix_i);
                        }
                    }
                }
            }
        }
   }
   ++_counter;
}