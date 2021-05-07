#include "pointcloud.h"
#include "exception.h"
#include <tchar.h>

symb::scanner::PointCloud::PointCloud(const std::shared_ptr<open3d::geometry::PointCloud> &point_cloud) : point_cloud_(point_cloud) {}

symb::scanner::PointCloud::PointCloud(const symb::scanner::CalibrationData &calibration, const symb::scanner::Frame &frame, const float depth_scale, const utility::Range3Df range)
{
	if (!frame.from_stream(rs2_stream::RS2_STREAM_DEPTH))
	{
		throw error::InvalidParameterError("PointCloud given frame does not contain depth frame");
	}
	const rs2::frameset &rs2_frame = frame.raw();
	const rs2::depth_frame &rs2_depth_frame = rs2_frame.get_depth_frame();
	const int height = rs2_depth_frame.get_height();
	const int width = rs2_depth_frame.get_width();

	cv::Mat color_mat;
	bool has_color = false;
	if (frame.from_stream(rs2_stream::RS2_STREAM_COLOR))
	{
		const rs2::video_frame &rs2_color_frame = rs2_frame.get_color_frame();
		const int color_height = rs2_color_frame.get_height();
		const int color_width = rs2_color_frame.get_width();
		if (color_height == height && color_width == width)
		{
			has_color = true;
			color_mat = cv::Mat(cv::Size(width, height), CV_8UC3, const_cast<void *>(rs2_color_frame.get_data()), cv::Mat::AUTO_STEP);
		}
	}

	const uint16_t *filtered_depth_frame = reinterpret_cast<const uint16_t *>(rs2_depth_frame.get_data());

	const rs2_intrinsics intrinsics = rs2_depth_frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
	const Eigen::Matrix4d camera_extrinsics = calibration.get_extrinsics();

	for (int v = 0, pixel_index = 0; v < height; v++, pixel_index += width)
	{
		for (int u = 0; u < width; u++)
		{
			const float distance = static_cast<float>(filtered_depth_frame[pixel_index + u]) * depth_scale; // pixel (u, v) における物体の距離
			
			//// ゼロ距離(無効な点)を排除
			if (distance <= FLT_EPSILON)
			{
				continue;
			}

			float deprojected_point[3];
			float pixel[2] = {static_cast<float>(u), static_cast<float>(v)};
			rs2_deproject_pixel_to_point(deprojected_point, &intrinsics, pixel, distance);
			const Eigen::Vector3d target_coordinate_3d(static_cast<double>(deprojected_point[0]), static_cast<double>(deprojected_point[1]), static_cast<double>(deprojected_point[2]));

			const Eigen::Vector3d point = (camera_extrinsics * (Eigen::Vector4d() << target_coordinate_3d, 1).finished()).block<3, 1>(0, 0);

			// 記録する物体のX座標を狭める
			if (point(0) < range.x.min && range.x.min != 0.0f)
				continue;
			if (point(0) > range.x.max && range.x.max != 0.0f)
				continue;

			// 記録する物体のY座標を狭める
			if (point(1) < range.y.min && range.y.min != 0.0f)
				continue;
			if (point(1) > range.y.max && range.y.max != 0.0f)
				continue;

			// 記録する物体のZ座標を狭める
			if (point(2) < range.z.min && range.z.min != 0.0f)
				continue;
			if (point(2) > range.z.max && range.z.max != 0.0f)
				continue;

			if (has_color)
			{
				const cv::Vec3b bgr = color_mat.at<cv::Vec3b>(cv::Point(u, v));
				if (bgr(0) != 0 || bgr(1) != 0 || bgr(2) != 0)
				{
					point_cloud_->points_.emplace_back(point);
					Eigen::Vector3d color = (Eigen::Vector3d() << bgr(2) / 255.0, bgr(1) / 255.0, bgr(0) / 255.0).finished();
					point_cloud_->colors_.emplace_back(color); // rgb
				}
			}
			else
			{
				point_cloud_->points_.emplace_back(point);
				point_cloud_->colors_.emplace_back(Eigen::Vector3d::Ones());
			}
		}
	}
}

symb::scanner::PointCloud::PointCloud(const std::vector<PointCloud> &point_clouds)
{
	const Eigen::Vector3d zero(0, 0, 0);
	const Eigen::Vector3d white(1, 1, 1);
	size_t size = 0;
	for (const PointCloud &pointcloud: point_clouds)
	{
		size += pointcloud.point_cloud_->points_.size();
	}
	point_cloud_->points_.reserve(size);
	point_cloud_->colors_.reserve(size);
	point_cloud_->normals_.reserve(size);
	for (const PointCloud &pointcloud : point_clouds)
	{
		const std::shared_ptr<open3d::geometry::PointCloud> &op3pc = pointcloud.point_cloud_;
		const size_t count = op3pc->points_.size();
		point_cloud_->points_.insert(point_cloud_->points_.end(), op3pc->points_.begin(), op3pc->points_.end());
		if (op3pc->colors_.empty())
		{
			std::fill(point_cloud_->colors_.end(), point_cloud_->colors_.end() + count, white);
		}
		else
		{
			point_cloud_->colors_.insert(point_cloud_->colors_.end(), op3pc->colors_.begin(), op3pc->colors_.end());
		}
		if (op3pc->normals_.empty())
		{
			std::fill(point_cloud_->normals_.end(), point_cloud_->normals_.end() + count, zero);
		}
		else
		{
			point_cloud_->normals_.insert(point_cloud_->normals_.end(), op3pc->normals_.begin(), op3pc->normals_.end());
		}
	}
}

void symb::scanner::PointCloud::downSample(const double voxel_size)
{
	const std::shared_ptr<open3d::geometry::PointCloud> down_sampled = open3d::geometry::VoxelDownSample(*point_cloud_, voxel_size);
	point_cloud_->points_ = down_sampled->points_;
	point_cloud_->colors_ = down_sampled->colors_;
	point_cloud_->normals_ = down_sampled->normals_;
}

void symb::scanner::PointCloud::estimateNormals(const int knn)
{
	EstimateNormals(*point_cloud_, open3d::geometry::KDTreeSearchParamKNN(knn));
}

void symb::scanner::PointCloud::estimateNormalsHybrid(const double radius, const int knn)
{
	EstimateNormals(*point_cloud_, open3d::geometry::KDTreeSearchParamHybrid(radius, knn));
}

void symb::scanner::PointCloud::orientNormals(const Eigen::Vector3d &camera_coordinate)
{
	open3d::geometry::OrientNormalsTowardsCameraLocation(*point_cloud_, camera_coordinate);
}

void symb::scanner::PointCloud::removeStatisticalOutliers(const int nb_neighbors, const double std_ratio)
{
	open3d::geometry::RemoveStatisticalOutliers(*point_cloud_, nb_neighbors, std_ratio);
}

void symb::scanner::PointCloud::generateMesh(const int depth, const float samplesPerNode, const std::experimental::filesystem::path &save_path)
{
	const std::experimental::filesystem::path current_path = std::experimental::filesystem::current_path();
	const std::experimental::filesystem::path dll_path = absolute(current_path / "poissonrecon.dll");
	const std::experimental::filesystem::path tmp_path = absolute(current_path / "tmp.ply");

	ReconParams recon_params;
	char in_path[512];
	char out_path[512];
	sprintf_s(in_path, "%s", tmp_path.string().c_str());
	sprintf_s(out_path, "%s", save_path.string().c_str());

	recon_params.in = in_path;
	recon_params.out = out_path;
	recon_params.depth = depth; //値を大きくする方が詳細なメッシュになるが処理時間が長くなる。samplesPerNodeで調整する方が良さげ。
	recon_params.samplesPerNode = samplesPerNode; //この値を大きく(20.0とか)すると、点群の削りすぎを防止できる。
	recon_params.voxel = 0; //ダウンサンプリングしない場合は0 ※単位はmで、1.0で1m
	recon_params.crop = false; // crop（ノイズ除去）しない場合はfalse

	const auto &delete_tmp = [&tmp_path]()
	{
		try
		{
			if (std::experimental::filesystem::exists(tmp_path))
			{
				std::experimental::filesystem::remove(tmp_path);
			}
		}
		catch (...)
		{
		}
	};

	typedef int(CALLBACK * LPFN_DHFDLL)(ReconParams * params_);
	// const HINSTANCE hInst = ::LoadLibrary(_T(dll_path.string().c_str()));
	// if (nullptr != hInst)
	// {
	// 	const LPFN_DHFDLL Reconstruct = reinterpret_cast<LPFN_DHFDLL>(::GetProcAddress(hInst, "Reconstruct"));
	// 	if (Reconstruct)
	// 	{
	// 		open3d::io::WritePointCloudToPLY(tmp_path.string(), *point_cloud_);
	// 		int ret;
	// 		try
	// 		{
	// 			ret = Reconstruct(&recon_params);
	// 		}
	// 		catch (const std::invalid_argument &e)
	// 		{
	// 			delete_tmp();
	// 			::FreeLibrary(hInst);
	// 			throw error::InvalidParameterError(fmt::format("PointCloud::generateMesh({}, {}, {}) failed with invalid argument {}", depth, samplesPerNode, save_path.string(), e.what()));
	// 		}
	// 		delete_tmp();
	// 		if (ret != EXIT_SUCCESS)
	// 		{
	// 			::FreeLibrary(hInst);
	// 			throw error::BadStateError(fmt::format("PointCloud::generateMesh({}, {}, {}) failed with error {}", depth, samplesPerNode, save_path.string(), ret));
	// 		}
	// 	}
	// 	else
	// 	{
	// 		SPDLOG_ERROR("PointCloud::generateMesh Reconstruct Function not found");
	// 	}
	// 	::FreeLibrary(hInst);
	// }
	// else
	// {
	// 	SPDLOG_ERROR("PointCloud::generateMesh Unable to Load Library from {}", dll_path.string());
	// }
}

void symb::scanner::PointCloud::scaleUp(const float scale)
{
	for (Eigen::Vector3d &point : point_cloud_->points_)
	{
		point *= scale;
	}
}

void symb::scanner::PointCloud::extractLargestConnectedComponent(const double radius, const int knn)
{
	int min_points = knn - 1;

	open3d::geometry::KDTreeFlann kdtree(*point_cloud_);

	// precompute all neighbours
	SPDLOG_DEBUG("Precompute Neighbours");
	std::vector<std::vector<int>> nbs(point_cloud_->points_.size());

	const auto &l = [&](const int start, const int end) {
		SPDLOG_DEBUG("Loop {} -> {}", start, end);
		if (knn <= 0)
		{
			for (int idx = start; idx < end; ++idx)
			{
				std::vector<double> dists2;
				kdtree.SearchRadius(point_cloud_->points_[idx], radius, nbs[idx], dists2);
			}
		}
		else if (radius < 0.00000001)
		{
			for (int idx = start; idx < end; ++idx)
			{
				std::vector<double> dists2;
				kdtree.SearchKNN(point_cloud_->points_[idx], knn, nbs[idx], dists2);
			}
		}
		else
		{
			for (int idx = start; idx < end; ++idx)
			{
				std::vector<double> dists2;
				kdtree.SearchHybrid(point_cloud_->points_[idx], radius, knn, nbs[idx], dists2);
			}
		}
		SPDLOG_DEBUG("Loop {} -> {} Completed", start, end);
	};
	std::vector<std::thread> threads;
	const int end = int(point_cloud_->points_.size());
	const int d = end / 6 + 1;
	for (int i = 0; i < end; i += d)
	{
		const int dh = (i + d > end) ? end : i + d;
		threads.emplace_back(l, i, dh);
	}
	for (std::thread &thread : threads)
	{
		thread.join();
	}

	SPDLOG_DEBUG("Done Precompute Neighbours");

	// set all labels to undefined (-2)
	SPDLOG_DEBUG("Compute Clusters");
	std::vector<int> labels(point_cloud_->points_.size(), -2);
	int cluster_label = 0;
	for (size_t idx = 0; idx < point_cloud_->points_.size(); ++idx)
	{
		if (labels[idx] != -2)
		{ // label is not undefined
			continue;
		}

		// check density
		if (nbs[idx].size() < min_points)
		{
			labels[idx] = -1;
			continue;
		}

		std::unordered_set<int> nbs_next(nbs[idx].begin(), nbs[idx].end());
		std::unordered_set<int> nbs_visited;
		nbs_visited.insert(int(idx));

		labels[idx] = cluster_label;
		while (!nbs_next.empty())
		{
			int nb = *nbs_next.begin();
			nbs_next.erase(nbs_next.begin());
			nbs_visited.insert(nb);

			if (labels[nb] == -1)
			{ // noise label
				labels[nb] = cluster_label;
			}
			if (labels[nb] != -2)
			{ // not undefined label
				continue;
			}
			labels[nb] = cluster_label;

			if (nbs[nb].size() >= min_points)
			{
				for (int qnb : nbs[nb])
				{
					if (nbs_visited.count(qnb) == 0)
					{
						nbs_next.insert(qnb);
					}
				}
			}
		}

		cluster_label++;
	}

	SPDLOG_DEBUG("Done Compute Clusters: {:d}", cluster_label);

	int max_label = cluster_label;
	size_t max_label_count = 0;
	std::vector<size_t> cluster_count(cluster_label, 0);
	for (int label : labels)
	{
		if (label < 0)
			continue;
		cluster_count[label] += 1;
		if (cluster_count[label] > max_label_count)
		{
			max_label = label;
			max_label_count = cluster_count[label];
		}
	}

	SPDLOG_DEBUG("Max Label is {} with {} points", max_label, max_label_count);

	std::shared_ptr<open3d::geometry::PointCloud> new_point_cloud = std::make_shared<open3d::geometry::PointCloud>();
	new_point_cloud->points_.reserve(max_label_count);
	if (!point_cloud_->colors_.empty())
	{
		new_point_cloud->colors_.reserve(max_label_count);
	}
	if (!point_cloud_->normals_.empty())
	{
		new_point_cloud->normals_.reserve(max_label_count);
	}

	const size_t size = point_cloud_->points_.size();
	for (size_t i = 0; i < size; ++i)
	{
		if (labels[i] == max_label)
		{
			new_point_cloud->points_.emplace_back(point_cloud_->points_[i]);
			if (!point_cloud_->colors_.empty())
			{
				new_point_cloud->colors_.emplace_back(point_cloud_->colors_[i]);
			}
			if (!point_cloud_->normals_.empty())
			{
				new_point_cloud->normals_.emplace_back(point_cloud_->normals_[i]);
			}
		}
	}
	point_cloud_->points_ = new_point_cloud->points_;
	point_cloud_->colors_ = new_point_cloud->colors_;
	point_cloud_->normals_ = new_point_cloud->normals_;
}
