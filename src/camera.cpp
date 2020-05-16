#include <iostream>
#include <camera.hpp>
#include <ros/ros.h>

namespace calib_camera {

Camera::Camera(const int id, const std::string frame)
               : id_(id), frame_(frame) {}

CameraData::CameraData(CameraPtr camera, const double u, const double v)
                       : camera_(camera), u_(u), v_(v) {}

Vector2 CameraData::getUV() const {
  return Vector2(u_, v_);
}

Vector2 ReprojectToImageData(const Point& target,
                             const Matrix3& intrinsic_matrix,
                             const std::vector<float>& k) {
  double x = target[0] / target[2];
  double y = target[1] / target[2];
  double z = 1;

//  Consider distortion parameters k
  double r = std::sqrt(x * x + y * y);
  double x_ = x * (1 + k[0] * std::pow(r, 2) + k[1] * std::pow(r, 4) );
  double y_ = y * (1 + k[0] * std::pow(r, 2) + k[1] * std::pow(r, 4) );

  Point pt_3D{x_, y_, z};
  Point proj_pt = intrinsic_matrix * pt_3D;

  return Vector2{proj_pt[0], proj_pt[1]};
}
}  // namespace calib_camera
