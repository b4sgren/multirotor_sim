#include "simulator.h"
#include <Eigen/StdVector>

using namespace std;

Simulator::Simulator() :
  seed_(std::chrono::system_clock::now().time_since_epoch().count()),
  env_(seed_),
  generator_(seed_),
  uniform_(0.0, 1.0)
{
  srand(seed_);
}

Simulator::Simulator(bool prog_indicator) :
  seed_(std::chrono::system_clock::now().time_since_epoch().count()),
  env_(seed_),
  generator_(seed_),
  uniform_(0.0, 1.0),
  prog_indicator_(prog_indicator)
{
  uint64_t seed = std::chrono::system_clock::now().time_since_epoch().count();
  generator_ = std::default_random_engine(seed);
  srand(seed);
}

Simulator::Simulator(bool prog_indicator, uint64_t seed):
    seed_(seed),
    env_(seed_),
    generator_(seed_),
    uniform_(0.0, 1.0),
    prog_indicator_(prog_indicator)
{
  srand(seed);
}

Simulator::~Simulator()
{
    if (prog_indicator_)
        cout << endl;

    if (log_.is_open())
      log_.close();
}


void Simulator::load(string filename)
{
  t_ = 0;
  get_yaml_node("tmax", filename, tmax_);

  // Log
  get_yaml_node("log_filename", filename, log_filename_);
  if (!log_filename_.empty())
  {
    log_.open(log_filename_);
  }

  // Load IMU parameters
  Vector4d q_b_u;
  get_yaml_node("imu_update_rate", filename, imu_update_rate_);
  dt_ = 1.0 / imu_update_rate_;
  get_yaml_eigen("p_b_u", filename, p_b_u_);
  get_yaml_eigen("q_b_u", filename, q_b_u);
  q_b_u_ = Quatd(q_b_u);

  // Accelerometer
  double accel_noise, accel_walk, accel_init;
  get_yaml_node("use_accel_truth", filename, use_accel_truth_);
  get_yaml_node("accel_init_stdev", filename, accel_init);
  get_yaml_node("accel_noise_stdev", filename, accel_noise);
  get_yaml_node("accel_bias_walk", filename, accel_walk);
  accel_bias_ =  !use_accel_truth_ * accel_init * Vector3d::Random(); // Uniformly random init within +-accel_walk
  accel_noise_stdev_ = !use_accel_truth_ * accel_noise;
  accel_walk_stdev_ = !use_accel_truth_ * accel_walk;
  accel_noise_.setZero();

  // Gyro
  double gyro_noise, gyro_walk, gyro_init;
  get_yaml_node("use_gyro_truth", filename, use_gyro_truth_);
  get_yaml_node("gyro_noise_stdev", filename, gyro_noise);
  get_yaml_node("gyro_init_stdev", filename, gyro_init);
  get_yaml_node("gyro_bias_walk", filename, gyro_walk);
  gyro_bias_ = gyro_init * Vector3d::Random()  * !use_gyro_truth_; // Uniformly random init within +-gyro_walk
  gyro_noise_stdev_ = gyro_noise * !use_gyro_truth_;
  gyro_walk_stdev_ = gyro_walk * !use_gyro_truth_;
  gyro_noise_.setZero();

  // Camera
  double pixel_noise;
  Vector2d focal_len;
  get_yaml_node("use_camera_truth", filename, use_camera_truth_);
  get_yaml_node("camera_update_rate", filename, camera_update_rate_);
  get_yaml_eigen("cam_center", filename, cam_center_);
  get_yaml_eigen("image_size", filename, image_size_);
  get_yaml_eigen("q_b_c", filename, q_b_c_.arr_);
//  get_yaml_eigen("p_b_c", filename, t_b_c_);
  get_yaml_eigen("t_b_c", filename, t_b_c_);
  get_yaml_eigen("focal_len", filename, focal_len);
  get_yaml_node("pixel_noise_stdev", filename, pixel_noise);
  pixel_noise_stdev_ = !use_camera_truth_ * pixel_noise;
  pixel_noise_.setZero();
  cam_F_ << focal_len(0,0), 0, 0, 0, focal_len(1,0), 0; // Copy focal length into 2x3 matrix for future use
  next_global_feature_id_ = 0;

  // Altimeter
  double altimeter_noise;
  get_yaml_node("use_altimeter_truth", filename, use_altimeter_truth_);
  get_yaml_node("altimeter_update_rate", filename, altimeter_update_rate_);
  get_yaml_node("altimeter_noise_stdev", filename, altimeter_noise);
  altimeter_noise_stdev_ = altimeter_noise * !use_altimeter_truth_;
  altimeter_noise_ = 0;

  // Depth
  double depth_noise;
  get_yaml_node("use_depth_truth", filename, use_depth_truth_);
  get_yaml_node("init_depth", filename, init_depth_);
  get_yaml_node("depth_update_rate", filename, depth_update_rate_);
  get_yaml_node("depth_noise_stdev", filename, depth_noise);
  get_yaml_node("feat_move_prob", filename, feat_move_prob_);
  depth_noise_stdev_ = depth_noise * !use_depth_truth_;
  depth_noise_ = 0;

  // Truth
  double att_noise, pos_noise;
  get_yaml_node("truth_update_rate", filename, truth_update_rate_);
  get_yaml_node("use_attitude_truth", filename, use_attitude_truth_);
  get_yaml_node("use_position_truth", filename, use_position_truth_);
  get_yaml_node("attitude_noise_stdev", filename, att_noise);
  get_yaml_node("position_noise_stdev", filename, pos_noise);
  get_yaml_node("truth_time_offset", filename, mocap_time_offset_);
  get_yaml_node("truth_transmission_noise", filename, mocap_transmission_noise_);
  get_yaml_node("truth_transmission_time", filename, mocap_transmission_time_);
  get_yaml_eigen("p_b_m", filename, p_b_m_);
  get_yaml_eigen("q_b_m", filename, q_b_m_.arr_);
  attitude_noise_stdev_ = att_noise * !use_attitude_truth_;
  position_noise_stdev_ = pos_noise * !use_position_truth_;

  cont_.load(filename);
  env_.load(filename);
  dyn_.load(filename);

  // EKF
  get_yaml_node("feature_update_active", filename, feature_update_active_);
  get_yaml_node("drag_update_active", filename, drag_update_active_);
  get_yaml_node("depth_update_active", filename, depth_update_active_);
  get_yaml_node("altimeter_update_active", filename, altimeter_update_active_);
  get_yaml_node("attitude_update_active", filename, attitude_update_active_);
  get_yaml_node("position_update_active", filename, position_update_active_);

  att_R_ = att_noise * att_noise * I_3x3;
  pos_R_ = pos_noise * pos_noise * I_3x3;
  acc_R_ = accel_noise * accel_noise * I_3x3;
  feat_R_ = pixel_noise * pixel_noise * I_2x2;
  alt_R_ << altimeter_noise * altimeter_noise;
  depth_R_ << depth_noise * depth_noise;

  // To get initial measurements
  last_imu_update_ = 0.0;
  last_camera_update_ = 0.0;
  last_altimeter_update_ = 0.0;

  // Compute initial control and corresponding acceleration
  cont_.computeControl(dyn_.get_state(), t_, u_);
  dyn_.compute_imu(u_);
  imu_.segment<3>(0) = dyn_.get_imu_accel() + accel_bias_ + accel_noise_ - dynamics::gravity_;
  imu_.segment<3>(3) = dyn_.get_state().segment<3>(dynamics::WX) + gyro_bias_ + gyro_noise_;
  imu_prev_.setZero();
  imu_prev_.segment<3>(0) = -dynamics::gravity_;

  // Start Progress Bar
  if (prog_indicator_)
    prog_.init(std::round(tmax_/dt_), 40);
}

bool Simulator::run()
{
  if (t_ < tmax_ - dt_ / 2.0) // Subtract half time step to prevent occasional extra iteration
  {
    // Propagate forward in time and get new control input and true acceleration
    dyn_.run(dt_, u_);
    t_ += dt_;
    cont_.computeControl(dyn_.get_state(), t_, u_);
    dyn_.compute_imu(u_); // True acceleration is based on current control input
    if (prog_indicator_)
        prog_.print(t_/dt_);

    log_state();
    return true;
  }
  else
  {
    prog_.finished();
    return false;
  }
}

void Simulator::log_state()
{
  if (log_.is_open())
  {
    log_.write((char*)&t_, sizeof(double));
    log_.write((char*)dyn_.get_state().data(), sizeof(double)*dyn_.get_state().rows());
  }
}

void Simulator::update_camera_pose()
{
  Quatd q_I_b = Quatd(dyn_.get_state().segment<4>(dynamics::QW));
  t_I_c_ = dyn_.get_state().segment<3>(dynamics::PX) + q_I_b.rota(t_b_c_);
  q_I_c_ = q_I_b * q_b_c_;

  for (auto it = tracked_points_.begin(); it != tracked_points_.end(); it++)
  {
    if (uniform_(generator_) < feat_move_prob_)
    {
      env_.move_point(it->env_id);
    }
  }
}

void Simulator::tracked_features(std::vector<int>& ids) const
{
  ids.clear();
  for (auto it = tracked_points_.begin(); it != tracked_points_.end(); it++)
  {
    ids.push_back(it->global_id);
  }
}


void Simulator::get_imu_meas(std::vector<measurement_t, Eigen::aligned_allocator<measurement_t>>& meas_list)
{
    if (t_ >= last_imu_update_ + 1.0/imu_update_rate_)
    {
      double dt = t_ - last_imu_update_;
      last_imu_update_ = t_;
      imu_prev_ = imu_;

      // Bias random walks and IMU noise
      if (!use_accel_truth_)
      {
        Vector3d accel_walk;
        random_normal_vec(accel_walk, accel_walk_stdev_, normal_, generator_);
        accel_bias_ += accel_walk * accel_walk_stdev_ * dt;
        random_normal_vec(accel_noise_, accel_noise_stdev_, normal_,  generator_);
      }
      if (!use_gyro_truth_)
      {
        Vector3d gyro_walk;
        random_normal_vec(gyro_walk, gyro_walk_stdev_, normal_,  generator_);
        gyro_bias_ += gyro_walk * dt;
        random_normal_vec(gyro_noise_, gyro_noise_stdev_, normal_,  generator_);
      }

      // Populate accelerometer and gyro measurements
      imu_.segment<3>(0) = dyn_.get_imu_accel() + accel_bias_ + accel_noise_;
      imu_.segment<3>(3) = dyn_.get_imu_gyro() + gyro_bias_ + gyro_noise_;

      // Collect x/y acceleration measurements for drag update
      measurement_t acc_meas;
      acc_meas.t = t_;
      acc_meas.type = ACC;
      acc_meas.z = get_acc();
      acc_meas.R = acc_R_;
      acc_meas.active = drag_update_active_;
      meas_list.push_back(acc_meas);
    }
}


void Simulator::get_camera_meas(std::vector<measurement_t, Eigen::aligned_allocator<measurement_t>>& meas_list)
{
    // If it's time to capture new measurements, then do it
    if (t_ > last_camera_update_ + 1.0/camera_update_rate_)
    {
      last_camera_update_ = t_;
      update_camera_pose();

      // Update feature measurements for currently tracked features
      for(auto it = tracked_points_.begin(); it != tracked_points_.end();)
      {
        if (update_feature(*it))
        {
          measurement_t meas;
          meas.t = t_;
          meas.type = FEAT;
          meas.z = get_pixel((*it));
          meas.R = feat_R_;
          meas.global_id = (*it).global_id;
          meas.depth = get_depth((*it));
          meas.active = feature_update_active_;
          camera_measurements_buffer_.push_back(meas);
          DBG("update feature - envID = %d globalID = %d\n",
              it->env_id, it->global_id);
          it++;
        }
        else
        {
          if (it->zeta(2,0) < 0)
          {
            DBG("clearing feature - envID = %d globalID = %d because went negative [%f, %f, %f]\n",
                it->env_id, it->global_id, it->zeta(0,0), it->zeta(1,0), it->zeta(2,0));
          }
          else if ((it->pixel.array() < 0).any() || (it->pixel.array() > image_size_.array()).any())
          {
            DBG("clearing feature - envID = %d globalID = %d because went out of frame [%f, %f]\n",
                it->env_id, it->global_id, it->pixel(0,0), it->pixel(1,0));
          }
          tracked_points_.erase(it);
        }
      }

      while (tracked_points_.size() < NUM_FEATURES)
      {
        // Add the new feature to our "tracker"
        feature_t new_feature;
        if (!get_random_feature_in_frame(new_feature)) break;
        tracked_points_.push_back(new_feature);
        DBG("new feature - envID = %d globalID = %d [%f, %f, %f], [%f, %f]\n",
            new_feature.env_id, new_feature.global_id, new_feature.zeta(0,0), new_feature.zeta(1,0),
            new_feature.zeta(2,0), new_feature.pixel(0,0), new_feature.pixel(1,0));

        // Pixel noise
        if (!use_camera_truth_)
          random_normal_vec(pixel_noise_, pixel_noise_stdev_, normal_, generator_);

        // Create a measurement for this new feature
        measurement_t meas;
        meas.t = t_;
        meas.type = FEAT;
        meas.z = get_pixel(new_feature) + pixel_noise_;
        meas.R = feat_R_;
        meas.global_id = new_feature.global_id;
        meas.depth = get_depth(new_feature, init_depth_);
        meas.active = true; // always true for new features
        camera_measurements_buffer_.push_back(meas);
      }
    }

    // Push out the measurement if it is time to send it
    if ((t_ > last_camera_update_ + camera_time_delay_) && (camera_measurements_buffer_.size() > 0));
    {
      for (auto it = camera_measurements_buffer_.begin(); it != camera_measurements_buffer_.end(); it++)
      {
        meas_list.push_back(*it);
      }
      camera_measurements_buffer_.clear();
    }
}


void Simulator::get_alt_meas(std::vector<measurement_t, Eigen::aligned_allocator<measurement_t>>& meas_list)
{
    if (t_ >= last_altimeter_update_ + 1.0/altimeter_update_rate_)
    {
      // Altimeter noise
      if (!use_altimeter_truth_)
        altimeter_noise_ = altimeter_noise_stdev_ * normal_(generator_);
      Matrix<double, 1, 1> noise(altimeter_noise_);

      last_altimeter_update_ = t_;
      measurement_t meas;
      meas.t = t_;
      meas.type = ALT;
      meas.z = get_altitude() + noise;
      meas.R = alt_R_;
      meas.active = altimeter_update_active_;
      meas_list.push_back(meas);
    }
}


void Simulator::get_mocap_meas(std::vector<measurement_t, Eigen::aligned_allocator<measurement_t>>& meas_list)
{
    if (t_ >= last_truth_update_ + 1.0/truth_update_rate_)
    {
      measurement_t att_meas;
      att_meas.t = t_ - mocap_time_offset_;
      att_meas.type = ATT;
      att_meas.z = get_attitude();
      att_meas.R = att_R_;
      att_meas.active = attitude_update_active_;

      measurement_t pose_meas;
      pose_meas.t = t_ - mocap_time_offset_;
      pose_meas.type = POS;
      pose_meas.z = get_position();
      pose_meas.R = pos_R_;
      pose_meas.active = position_update_active_;

      double pub_time = std::max(mocap_transmission_time_ + normal_(generator_) * mocap_transmission_noise_, 0.0) + t_;

      mocap_measurement_buffer_.push_back(std::pair<double, measurement_t>{pub_time, pose_meas});
      mocap_measurement_buffer_.push_back(std::pair<double, measurement_t>{pub_time, att_meas});
      last_truth_update_ = t_;
    }

    while (mocap_measurement_buffer_.size() > 0 && mocap_measurement_buffer_[0].first > t_)
    {
      meas_list.push_back(mocap_measurement_buffer_[0].second);
      mocap_measurement_buffer_.erase(mocap_measurement_buffer_.begin());
    }
}

void Simulator::get_measurements(std::vector<measurement_t, Eigen::aligned_allocator<measurement_t>>& meas_list)
{
  meas_list.clear();

  get_imu_meas(meas_list);
  get_camera_meas(meas_list);
  get_alt_meas(meas_list);
  get_mocap_meas(meas_list);
}

int Simulator::global_to_local_feature_id(const int global_id) const
{
  int i = 0;
  while (i < tracked_points_.size())
  {
    if (tracked_points_[i].global_id == global_id)
    {
      return i;
    }
    i++;
  }
  return -1;
}

Xformd Simulator::get_pose() const
{
  Xformd x;
  x.t_ = dyn_.get_state().segment<3>(dynamics::PX);
  x.q_ = dyn_.get_state().segment<4>(dynamics::QW);
  return x;
}

void Simulator::get_truth(xVector &x, const std::vector<int>& tracked_features) const
{
  x.block<10,1>(0,0) = dyn_.get_state().block<10,1>(0,0);
  x.block<3,1>(xB_A,0) = accel_bias_;
  x.block<3,1>(xB_G,0) = gyro_bias_;
  x(xMU,0) = dyn_.get_drag();
  for (auto it = tracked_features.begin(); it != tracked_features.end(); it++)
  {
    int i = global_to_local_feature_id(*it);
    if (i < 0)
    {
      continue;
      // This happens if the feature hasn't been added yet (because of the camera time offset)
      // This usually happens when no features are in the camera's field of view
//      std::stringstream err;
//      err << "File: " << __FILE__ << ", Line: " << __LINE__;
//      err << "\nEKF feature not found in truth - cannot compare";
//      throw std::runtime_error(err.str());
    }

    int xZETA = xZ + 5 * i;
    int xRHO = xZ + 5*i + 4;
    x.block<4,1>(xZETA,0) = Quatd::from_two_unit_vectors(e_z, tracked_points_[i].zeta).elements();
    x(xRHO,0) =1.0/ tracked_points_[i].depth;
  }
}


bool Simulator::update_feature(feature_t &feature) const
{
  if (feature.env_id > env_.get_points().rows() || feature.env_id < 0)
    return false;
  
  // Calculate the bearing vector to the feature
  Vector3d pt = env_.get_points().row(feature.env_id).transpose();
  feature.zeta = q_I_c_.rotp(pt - t_I_c_);
  feature.zeta /= feature.zeta.norm();
  feature.depth = (env_.get_points().row(feature.env_id).transpose() - t_I_c_).norm();
  
  // we can reject anything behind the camera
  if (feature.zeta(2) < 0.0)
    return false;
  
  // See if the pixel is in the camera frame
  proj(feature.zeta, feature.pixel);
  if ((feature.pixel.array() < 0).any() || (feature.pixel.array() > image_size_.array()).any())
    return false; 
  else
    return true;
}

bool Simulator::get_random_feature_in_frame(feature_t &feature)
{
  feature.env_id = env_.add_point(t_I_c_, q_I_c_, feature.zeta, feature.pixel, feature.depth);
  if (feature.env_id != -1)
  {
    feature.global_id = next_global_feature_id_++;
    return true;
  }
  else
  {
//    cout << "\nGround Plane is not in camera frame " << endl;
  }
  return false;
}

bool Simulator::is_feature_tracked(int env_id) const
{
  auto it = tracked_points_.begin();
  while (it < tracked_points_.end() && it->env_id != env_id)
  {
    it++;
  }
  return it != tracked_points_.end();    
}

void Simulator::proj(const Vector3d &zeta, Vector2d& pix) const
{
  double ezT_zeta = e_z.transpose() * zeta;  
  pix = cam_F_ * zeta / ezT_zeta + cam_center_;  
}


Vector4d Simulator::get_attitude()
{
  Vector3d noise;
  random_normal_vec(noise, attitude_noise_stdev_, normal_, generator_);
  Quatd q_I_b(dyn_.get_state().segment<4>(dynamics::QW)); // q_I^b
  Quatd q_I_m = q_I_b * q_b_m_; //  q_I^m = q_I^b * q_b^m
  return (q_I_m + noise).elements();
}


Vector3d Simulator::get_position()
{
  Vector3d noise;
  random_normal_vec(noise, position_noise_stdev_, normal_, generator_);
  Quatd q_I_b(dyn_.get_state().segment<4>(dynamics::QW)); // q_I^b
  Vector3d I_p_b_I_ = dyn_.get_state().segment<3>(dynamics::PX); // p_{b/I}^I
  Vector3d I_p_m_I_ = I_p_b_I_ + q_I_b.rota(p_b_m_); // p_{m/I}^I = p_{b/I}^I + R(q_I^b)^T (p_{m/b}^b)
  return I_p_m_I_;
}


Vector3d Simulator::get_acc()
{
  return get_imu_prev().segment<3>(0);
}

Vector2d Simulator::get_pixel(const feature_t &feature)
{
  Vector2d pixel_noise;
  random_normal_vec(pixel_noise, pixel_noise_stdev_, normal_, generator_);
  return feature.pixel + pixel_noise;
}

double Simulator::get_depth(const feature_t &feature, bool override)
{
  if (depth_update_active_ || override)
  {
    return feature.depth + depth_noise_stdev_ * normal_(generator_);
  }
  else
    return NAN;
}

Matrix<double, 1, 1> Simulator::get_altitude()
{
  /// TODO simulate sensor noise
  return -1.0 * dyn_.get_state().segment<1>(dynamics::PZ).array() + altimeter_noise_stdev_ * normal_(generator_);
}

Matrix6d Simulator::get_imu_noise_covariance() const
{
    Vector6d cov = (Vector6d() << accel_noise_stdev_*accel_noise_stdev_,
                                  accel_noise_stdev_*accel_noise_stdev_,
                                  accel_noise_stdev_*accel_noise_stdev_,
                                  gyro_noise_stdev_*gyro_noise_stdev_,
                                  gyro_noise_stdev_*gyro_noise_stdev_,
                                  gyro_noise_stdev_*gyro_noise_stdev_).finished();
    if (cov.norm() < 1e-8)
        return Matrix6d::Identity() * 1e-8;
    else
        return cov.asDiagonal();
}

Matrix6d Simulator::get_mocap_noise_covariance() const
{
  Vector6d cov = (Vector6d() << position_noise_stdev_*position_noise_stdev_,
                                position_noise_stdev_*position_noise_stdev_,
                                position_noise_stdev_*position_noise_stdev_,
                                attitude_noise_stdev_*attitude_noise_stdev_,
                                attitude_noise_stdev_*attitude_noise_stdev_,
                                attitude_noise_stdev_*attitude_noise_stdev_).finished();
  if (cov.norm() < 1e-8)
      return Matrix6d::Identity() * 1e-8;
  else
      return cov.asDiagonal();
}

