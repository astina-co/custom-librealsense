#include "open3d_090.h"

RANSACResult EvaluateRANSACBasedOnDistance(const std::vector<Eigen::Vector3d> &points, const Eigen::Vector4d plane_model, std::vector<size_t> &inliers, double distance_threshold, double error)
{
	RANSACResult result;

	for (size_t idx = 0; idx < points.size(); ++idx)
	{
		Eigen::Vector4d point(points[idx](0), points[idx](1), points[idx](2), 1);
		const double distance = std::abs(plane_model.dot(point));

		if (distance < distance_threshold)
		{
			error += distance;
			inliers.emplace_back(idx);
		}
	}

	const size_t inlier_num = inliers.size();
	if (inlier_num == 0)
	{
		result.fitness_ = 0;
		result.inlier_rmse_ = 0;
	}
	else
	{
		result.fitness_ = static_cast<double>(inlier_num) / static_cast<double>(points.size());
		result.inlier_rmse_ = error / std::sqrt(static_cast<double>(inlier_num));
	}
	return result;
}

Eigen::Vector4d GetPlaneFromPoints(const std::vector<Eigen::Vector3d> &points, const std::vector<size_t> &inliers)
{
	Eigen::Vector3d centroid(0, 0, 0);
	for (size_t idx : inliers)
	{
		centroid += points[idx];
	}
	centroid /= double(inliers.size());

	double xx = 0, xy = 0, xz = 0, yy = 0, yz = 0, zz = 0;

	for (size_t idx : inliers)
	{
		Eigen::Vector3d r = points[idx] - centroid;
		xx += r(0) * r(0);
		xy += r(0) * r(1);
		xz += r(0) * r(2);
		yy += r(1) * r(1);
		yz += r(1) * r(2);
		zz += r(2) * r(2);
	}

	const double det_x = yy * zz - yz * yz;
	const double det_y = xx * zz - xz * xz;
	const double det_z = xx * yy - xy * xy;

	Eigen::Vector3d abc;
	if (det_x > det_y && det_x > det_z)
	{
		abc = Eigen::Vector3d(det_x, xz * yz - xy * zz, xy * yz - xz * yy);
	}
	else if (det_y > det_z)
	{
		abc = Eigen::Vector3d(xz * yz - xy * zz, det_y, xy * xz - yz * xx);
	}
	else
	{
		abc = Eigen::Vector3d(xy * yz - xz * yy, xy * xz - yz * xx, det_z);
	}

	const double norm = abc.norm();
	// Return invalid plane if the points don't span a plane.
	if (norm == 0)
	{
		return Eigen::Vector4d::Zero();
	}
	abc /= abc.norm();
	const double d = -abc.dot(centroid);
	return Eigen::Vector4d(abc(0), abc(1), abc(2), d);
}

std::tuple<Eigen::Vector4d, std::vector<size_t>> SegmentPlane(const double distance_threshold /* = 0.01 */, const int ransac_n /* = 3 */, const int num_iterations /* = 100 */, const std::vector<Eigen::Vector3d> &points_)
{
	RANSACResult result;
	double error = 0;

	// Initialize the plane model ax + by + cz + d = 0.
	Eigen::Vector4d plane_model = Eigen::Vector4d(0, 0, 0, 0);
	// Initialize the best plane model.
	Eigen::Vector4d best_plane_model = Eigen::Vector4d(0, 0, 0, 0);

	// Initialize consensus set.
	std::vector<size_t> inliers;

	const size_t num_points = points_.size();
	std::vector<size_t> indices(num_points);
	std::iota(std::begin(indices), std::end(indices), 0);

	std::random_device rd;
	std::mt19937 rng(rd());

	// Return if ransac_n is less than the required plane model parameters.
	if (ransac_n < 3)
	{
		// utility::LogError("ransac_n should be set to higher than or equal to 3.");
		return std::make_tuple(best_plane_model, inliers);
	}
	if (num_points < size_t(ransac_n))
	{
		// utility::LogError("There must be at least 'ransac_n' points.");
		return std::make_tuple(best_plane_model, inliers);
	}

	for (int itr = 0; itr < num_iterations; itr++)
	{
		for (int i = 0; i < ransac_n; ++i)
		{
			std::swap(indices[i], indices[rng() % num_points]);
		}
		inliers.clear();
		for (int idx = 0; idx < ransac_n; ++idx)
		{
			inliers.emplace_back(indices[idx]);
		}

		// Fit model to num_model_parameters randomly selected points among the
		// inliers.
		plane_model = open3d::geometry::ComputeTrianglePlane(points_[inliers[0]], points_[inliers[1]], points_[inliers[2]]);
		if (plane_model.isZero(0))
		{
			continue;
		}

		error = 0;
		inliers.clear();
		const RANSACResult this_result = EvaluateRANSACBasedOnDistance(points_, plane_model, inliers, distance_threshold, error);
		if (this_result.fitness_ > result.fitness_ || (this_result.fitness_ == result.fitness_ && this_result.inlier_rmse_ < result.inlier_rmse_))
		{
			result = this_result;
			best_plane_model = plane_model;
		}
	}

	// Find the final inliers using best_plane_model.
	inliers.clear();
	for (size_t idx = 0; idx < points_.size(); ++idx)
	{
		Eigen::Vector4d point(points_[idx](0), points_[idx](1), points_[idx](2), 1);
		const double distance = std::abs(best_plane_model.dot(point));

		if (distance < distance_threshold)
		{
			inliers.emplace_back(idx);
		}
	}

	// Improve best_plane_model using the final inliers.
	best_plane_model = GetPlaneFromPoints(points_, inliers);

	// utility::LogDebug("RANSAC | Inliers: {:d}, Fitness: {:e}, RMSE: {:e}", inliers.size(), result.fitness_, result.inlier_rmse_);
	return std::make_tuple(best_plane_model, inliers);
}