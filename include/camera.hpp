#pragma once
#include <memory>
#include <string>
#include <vector>

#include <types.hpp>

namespace calib_camera {
class Camera {
 public:
  explicit Camera(const int id = -1, const std::string frame = "camera");
  inline virtual ~Camera() {}

  inline int getId() { return id_; }
  inline std::string getFrame() { return frame_; }

 private:
  int id_;
  std::string frame_;
};  // class Camera

using CameraPtr = std::shared_ptr<Camera>;

struct CameraData {
  CameraData(CameraPtr camera, const double u, const double v);

  Vector2 getUV() const;

  CameraPtr camera_;
  double u_;
  double v_;
};  // struct CameraData

// Project 3D-point to 2D UV-plane
Vector2 ReprojectToImageData(const Point& target,
                             const Matrix3& intrinsic_matrix,
                             const std::vector<float>& k);

}  // namespace calib_camera
