#include "nav_drone_mpc_controller/histogram.hpp"
#include <stdexcept>

namespace nav_drone_mpc_controller {
Histogram::Histogram(const int res)
  : resolution_{res}, z_dim_{360 / resolution_}, e_dim_{180 / resolution_}, weight_(e_dim_, z_dim_) {
  set_zero();
}

void Histogram::upsample() {
  if (resolution_ != ALPHA_RES * 2) {
    throw std::logic_error(
      "Invalid use of function upsample(). This function can only be used on a half resolution histogram.");
  }
  resolution_ = resolution_ / 2;
  z_dim_ = 2 * z_dim_;
  e_dim_ = 2 * e_dim_;
  Eigen::MatrixXf temp_dist(e_dim_, z_dim_);

  for (int i = 0; i < e_dim_; ++i) {
    for (int j = 0; j < z_dim_; ++j) {
      int i_lowres = floor(i / 2);
      int j_lowres = floor(j / 2);
      temp_dist(i, j) = weight_(i_lowres, j_lowres);
    }
  }
  weight_ = temp_dist;
}

void Histogram::downsample() {
  if (resolution_ != ALPHA_RES) {
    throw std::logic_error(
        "Invalid use of function downsample(). This function can only be used on a full resolution histogram.");
  }
  resolution_ = 2 * resolution_;
  z_dim_ = z_dim_ / 2;
  e_dim_ = e_dim_ / 2;
  Eigen::MatrixXf temp_dist(e_dim_, z_dim_);

  for (int i = 0; i < e_dim_; ++i) {
    for (int j = 0; j < z_dim_; ++j) {
      int i_high_res = 2 * i;
      int j_high_res = 2 * j;
      temp_dist(i, j) = weight_.block(i_high_res, j_high_res, 2, 2).mean();
    }
  }
  weight_ = temp_dist;
}

void Histogram::set_zero() { weight_.fill(0.f); }

bool Histogram::is_empty() const {
  int counter = 0;
  for (int e = 0; (e < e_dim_) && (0 == counter); e++) {
    for (int z = 0; (z < z_dim_) && (0 == counter); z++) {
      if (weight_(e, z) > FLT_MIN) {
        counter++;
      }
    }
  }
  return counter == 0;
}
  
void Histogram::go_binary(float theta_low, float theta_high) {  
  
  for (int e = 0; (e < e_dim_); e++) {
    for (int z = 0; (z < z_dim_); z++) {
      if (weight_(e, z) > theta_high) {
        set_weight(e, z, 1.0);
      } else if (weight_(e, z) < theta_low) {
        set_weight(e, z, 0.0);
      } else if (z > 0) {   // Avoid accessing invalid z index
        set_weight(e, z, weight_(e,z-1));
      } else if (e > 0) {
        set_weight(e, z, weight_(e-1, z_dim_-1));
      } else {
        set_weight(e, z, 1.0); // Desperate last call.  Just dont fly to e=0, z=0
      }        
    }
  }
  
}

}  // nav_drone_mpc_controller