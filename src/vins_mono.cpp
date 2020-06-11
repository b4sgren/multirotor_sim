#include "multirotor_sim/vins_mono.h"
#include <algorithm>
#include <utility>

VinsMono::VinsMono()
{
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
    
void VinsMono::imageCallback(const double& t, const ms::ImageFeat& z, const Eigen::Matrix2d& R_pix, const Matrix1d& R_depth)
{
    Eigen::Matrix3d I{Eigen::Matrix3d::Identity()};
    /*
    Steps:
    1. Find features which have been found before
    2. Define functors for these features at this current step
    3. Run optimization
    4. Add new features that were found for the first time
    5. Determine if this frame is a keyframe
    5b. If a new keyframe then bump off old keyframe (if necessary) and update parameters for all functors
    */
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
            for(auto it{_feature_funcs.begin()}; it != _feature_funcs.end(); ++it) //Is there a better way to do this
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