#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include <sstream>
#include <string>
#include <vector>

#include <calib_camera.hpp>

using calib_camera::AngleAxis;
using calib_camera::CameraData;
using calib_camera::DegToRad;
using calib_camera::Matrix3;
using calib_camera::Orientation;
using calib_camera::Point;
using calib_camera::Point3Data;
using calib_camera::Pose;
using calib_camera::Vector2;
using calib_camera::Vector3;


int main(int argc, char **argv) {
  ros::init(argc, argv, "camera_3D_2D_calib_node");
  ros::NodeHandle nh("~");

  std::vector<calib_camera::CameraData> points_camera;
  std::vector<calib_camera::Point3Data> points_3D;

  // read data
  std::string path = ros::package::getPath("calib_camera");
  std::fstream file(path+"/dataset/correspondences.csv");

  if (file.good()) {
    std::string line;
    std::vector<double> pair_data;
    getline(file, line);

    while (getline(file, line, '\n')) {
      pair_data.clear();
      std::istringstream templine(line);
      std::string data;
      while (getline(templine, data, ',')) {
        pair_data.push_back(atof(data.c_str()));
      }
      const calib_camera::Point3Data point_3D
            = Point3Data(std::make_shared<calib_camera::Point3D>(), pair_data[0], pair_data[1], pair_data[2]);
      const calib_camera::CameraData point_camera
            = CameraData(std::make_shared<calib_camera::Camera>(), pair_data[3], pair_data[4]);
      points_3D.push_back(point_3D);
      points_camera.push_back(point_camera);
    }
    std::cout << "Total " << points_camera.size() << " correspondences loaded." << std::endl;
    file.close();
  } else {
    std::cout << "Error: " << (path + "/dataset/correspondences.csv")
              << " cannot be oppened." << std::endl;
    return 0;
  }

  // Given a initial guess
  std::vector<float> translation;
  std::vector<float> rotation;
  nh.getParam("/camera_3D_2D_calib_node/initial_guess_camera_TO_lidar/translation", translation);
  nh.getParam("/camera_3D_2D_calib_node/initial_guess_camera_TO_lidar/rotation", rotation);

  Point p_radar = Point(translation[0], translation[1], translation[2]);
  auto r = AngleAxis(DegToRad(rotation[0]), Vector3::UnitZ())
          * AngleAxis(DegToRad(rotation[1]), Vector3::UnitY())
          * AngleAxis(DegToRad(rotation[2]), Vector3::UnitX());
  Orientation o_radar = Orientation(r);
  Pose initial_guess(o_radar, p_radar);

  // Set intrinsic matrix
  Matrix3 intrinsic_matrix;
  std::vector<float> matrix;
  nh.getParam("/camera_3D_2D_calib_node/intrinsic_parameters/matrix", matrix);

  intrinsic_matrix << matrix[0], matrix[1], matrix[2],
                      matrix[3], matrix[4], matrix[5],
                      matrix[6], matrix[7], matrix[8];

  std::vector<float> k;
  nh.getParam("/camera_3D_2D_calib_node/distortion_parameters/k", k);
  // Compute extrinc parameters between camera and lidar
  auto final_pose = Find_3D_2D_camera_Transform(points_camera,
                                                points_3D,
                                                initial_guess,
                                                intrinsic_matrix,
                                                k);
  int size = points_camera.size();
  for (int i = 0; i < std::min(10, size); i++) {
    Point p_test = points_3D[i].point_;
    Vector2 v_test = points_camera[i].getUV();
    p_test = intrinsic_matrix * (final_pose.inverse() * p_test);
    p_test[0] = p_test[0] / p_test[2];
    p_test[1] = p_test[1] / p_test[2];
    std::cout << "Projection = (" << p_test[0] << ", " << p_test[1] << ")"
              << ", Target = (" << v_test[0] << ", " << v_test[1] << ")" << std::endl;
  }


  while (ros::ok()) {
    ros::Duration(1).sleep();
    ros::spinOnce();
  }
}
