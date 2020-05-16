#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <calib_camera.hpp>

namespace calib_camera {
/*----------------3D-2D camera calibration module------------------*/

class LocalParameterizationSE3 : public ceres::LocalParameterization {
 public:
  virtual ~LocalParameterizationSE3() {}

  // SE3 plus operation for Ceres
  //
  //  T * exp(x)
  //
  virtual bool Plus(double const* T_raw, double const* delta_raw,
                    double* T_plus_delta_raw) const {
    Eigen::Map<Sophus::SE3d const> const T(T_raw);
    Eigen::Map<Sophus::Vector6d const> const delta(delta_raw);
    Eigen::Map<Sophus::SE3d> T_plus_delta(T_plus_delta_raw);
    T_plus_delta = T * Sophus::SE3d::exp(delta);

    return true;
  }

  // Jacobian of SE3 plus operation for Ceres
  //
  // Dx T * exp(x)  with  x=0
  //
  virtual bool ComputeJacobian(double const* T_raw,
                               double* jacobian_raw) const {
    Eigen::Map<Sophus::SE3d const> T(T_raw);
    Eigen::Map<Eigen::Matrix<double, 6, 7> > jacobian(jacobian_raw);
    jacobian = T.Dx_this_mul_exp_x_at_0();
    return true;
  }

  virtual int GlobalSize() const { return Sophus::SE3d::num_parameters; }
  virtual int LocalSize() const { return Sophus::SE3d::DoF; }
};  // class LocalParameterizationSE3


class ReprojectionError {
 public:
  ReprojectionError(const Vector2& pt_camera,
                    const Point& pt_3D,
                    const Matrix3& intrinsic_matrix,
                    const std::vector<float>& k)
      : pt_camera_(pt_camera),
        pt_3D_(pt_3D),
        intrinsic_matrix_(intrinsic_matrix),
        k_(k) {}

  bool operator()(const double* const pose_ptr, double* residual) const {
    Eigen::Map<SE3 const> const pose(pose_ptr);
    Vector2 pt_3D_reproj
            = ReprojectToImageData(pose.inverse() * pt_3D_,
                                   intrinsic_matrix_,
                                   k_);

    residual[0] = (pt_camera_ - pt_3D_reproj).norm();

    return true;
  }

  static ceres::CostFunction* Create(const Vector2& pt_camera,
                                     const Point& pt_3D,
                                     const Matrix3& intrinsic_matrix,
                                     const std::vector<float>& k) {
    return new ceres::NumericDiffCostFunction<ReprojectionError, ceres::RIDDERS, 1, SE3::num_parameters>(
        new ReprojectionError(pt_camera, pt_3D, intrinsic_matrix, k));
  }

 private:
  const Vector2 pt_camera_;
  const Point pt_3D_;
  const Matrix3 intrinsic_matrix_;
  const std::vector<float> k_;
};

Pose Find_3D_2D_camera_Transform(const std::vector<CameraData>& points_camera,
                                 const std::vector<Point3Data>& points_3D,
                                 const SE3& init_guess_transform,
                                 const Matrix3& intrinsic_matrix,
                                 const std::vector<float>& k) {
  assert(points_camera.size() == points_3D.size());
  ceres::Problem problem;

  SE3 pose = init_guess_transform;
  // Specify local update rule for our parameter
  problem.AddParameterBlock(pose.data(),
                            SE3::num_parameters,
                            new LocalParameterizationSE3);

  for (size_t i = 0; i < points_camera.size(); ++i) {
    ceres::CostFunction* cost_function = ReprojectionError::Create(
        points_camera[i].getUV(), points_3D[i].point_, intrinsic_matrix, k);
    problem.AddResidualBlock(cost_function, NULL, pose.data());
  }

  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = true;
  options.gradient_tolerance = 1e-6;  // * Sophus::Constants<double>::epsilon();
  options.function_tolerance = 1e-6;
  options.linear_solver_type = ceres::DENSE_QR;
  options.max_linear_solver_iterations = 200;
  options.max_num_iterations = 200;
  ceres::Solver::Summary summary;
  Solve(options, &problem, &summary);

  std::cout << summary.BriefReport() << std::endl;

  // Details
  Matrix3 init_Rx = init_guess_transform.unit_quaternion().toRotationMatrix();
  Vector3 init_euler = init_Rx.eulerAngles(2, 1, 0);
  Matrix3 Rx = pose.unit_quaternion().toRotationMatrix();
  Vector3 euler = Rx.eulerAngles(2, 1, 0);

  std::cout << "init pose: (x, y, z, yaw, pitch, raw)(deg) = ("
            << init_guess_transform.translation().x() << ", "
            << init_guess_transform.translation().y() << ", "
            << init_guess_transform.translation().z() << ", "
            << RadToDeg(init_euler[0]) << ", " << RadToDeg(init_euler[1])
            << ", " << RadToDeg(init_euler[2]) << " )" << std::endl;
  std::cout << "final pose: (x, y, z, yaw, pitch, raw)(deg) = ("
            << pose.translation().x() << ", " << pose.translation().y() << ", "
            << pose.translation().z() << ", " << RadToDeg(euler[0]) << ", "
            << RadToDeg(euler[1]) << ", " << RadToDeg(euler[2]) << " )"
            << std::endl;

  return pose;
}
}  // namespace calib_camera
