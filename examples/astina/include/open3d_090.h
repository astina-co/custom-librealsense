#pragma once

// OPEN3D 0.9.0 からコピー
// Copied from OPEN3D 0.9.0 - astina 2020/03/25

/*
The MIT License (MIT)

Open3D: www.open3d.org
Copyright (c) 2018 www.open3d.org

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

#include <numeric>
#include <random>

#include <Open3D/Geometry/HalfEdgeTriangleMesh.h>
#include <Open3d/3rdparty/Eigen/Eigen/Eigen>
#include <fmt/format.h>
#include <librealsense2/rs.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/opencv.hpp>
#include <spdlog/spdlog.h>

class RANSACResult
{
public:
	RANSACResult() : fitness_(0), inlier_rmse_(0) {}
	~RANSACResult() {}

public:
	double fitness_;
	double inlier_rmse_;
};

RANSACResult EvaluateRANSACBasedOnDistance(const std::vector<Eigen::Vector3d> &points, const Eigen::Vector4d plane_model, std::vector<size_t> &inliers, double distance_threshold, double error);

Eigen::Vector4d GetPlaneFromPoints(const std::vector<Eigen::Vector3d> &points, const std::vector<size_t> &inliers);

std::tuple<Eigen::Vector4d, std::vector<size_t>> SegmentPlane(const double distance_threshold /* = 0.01 */, const int ransac_n /* = 3 */, const int num_iterations /* = 100 */,
															  const std::vector<Eigen::Vector3d> &points_);