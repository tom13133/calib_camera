////////////////////////////////////////////////////////////////////////////////
//
// Filename:      calib_camera.hpp
// Authors:       Yu-Han, Hsueh
//
//////////////////////////////// FILE INFO /////////////////////////////////////
//
// calibration module for 3D-3D correspondences
//
/////////////////////////////////// LICENSE ////////////////////////////////////
//
// Copyright (C) 2020 Yu-Han, Hsueh <zero000.ece07g@nctu.edu.tw>
//
// This file is part of {calib_camera}.
//
//////////////////////////////////// NOTES /////////////////////////////////////
//
//
////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <vector>
#include <camera.hpp>
#include <point3d.hpp>

namespace calib_camera {
// return SE3(computed transform)
Pose Find_3D_2D_camera_Transform(const std::vector<CameraData>& points_camera,
                                 const std::vector<Point3Data>& points_lidar,
                                 const SE3& init_guess_transform,
                                 const Matrix3& intrinsic_matrix,
                                 const std::vector<float>& k);
}  // namespace calib_camera
