#include <filesystem>

#include <Open3D/Geometry/Octree.h>
#include <Open3D/Open3D.h>

#include <fmt/format.h>
#include <spdlog/spdlog.h>

int main(void)
{
	// pointcloud path
	const std::experimental::filesystem::path path_original = "C:/symb/data/data_2020_02_01_21_53_02_079/pointcloud_combined.ply";
	const std::experimental::filesystem::path path_filtered = "C:/symb/data/data_2020_02_01_21_53_02_079/pointcloud_filtered.ply";
	
	std::shared_ptr<open3d::geometry::PointCloud> point_cloud = std::make_shared <open3d::geometry::PointCloud> ();
	
	open3d::io::ReadPointCloudFromPLY(path_original.string(), *point_cloud);

	SPDLOG_INFO("Loaded Point Cloud with {} points", point_cloud->points_.size());

	double radius = 200;
	int knn = 20;
	size_t min_points = 19;
	
	open3d::geometry::KDTreeFlann kdtree(*point_cloud);

	// precompute all neighbours
	SPDLOG_INFO("Precompute Neighbours");
	std::vector<std::vector<int>> nbs(point_cloud->points_.size());
	
	const auto l = [&](const int start, const int end)
	{
		if (knn <= 0)
		{
			for (int idx = start; idx < end; ++idx)
			{
				std::vector<double> dists2;
				kdtree.SearchRadius(point_cloud->points_[idx], radius, nbs[idx], dists2);
			}
		}
		else if (radius < 0.00000001)
		{
			for (int idx = start; idx < end; ++idx)
			{
				std::vector<double> dists2;
				kdtree.SearchKNN(point_cloud->points_[idx], knn, nbs[idx], dists2);
			}
		}
		else
		{
			for (int idx = start; idx < end; ++idx)
			{
				std::vector<double> dists2;
				kdtree.SearchHybrid(point_cloud->points_[idx], radius, knn, nbs[idx], dists2);
			}
		}
	};
	std::vector<std::thread> threads;
	const int end = int(point_cloud->points_.size());
	const int d = end / 6 + 1;
	for (int i = 0; i < end; i += d)
	{
		const int dh = (i + d > end) ? end : i + d;
		SPDLOG_INFO("Loop {} -> {}", i, dh);
		threads.emplace_back(l, i, dh);
	}
	for (auto &&thread: threads)
	{
		thread.join();
	}
	
	SPDLOG_INFO("Done Precompute Neighbours");

	// set all labels to undefined (-2)
	SPDLOG_INFO("Compute Clusters");
	std::vector<int> labels(point_cloud->points_.size(), -2);
	int cluster_label = 0;
	for (size_t idx = 0; idx < point_cloud->points_.size(); ++idx)
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

	SPDLOG_INFO("Done Compute Clusters: {:d}", cluster_label);

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

	SPDLOG_INFO("Max Label is {} with {} points", max_label, max_label_count);

	std::shared_ptr<open3d::geometry::PointCloud> new_point_cloud = std::make_shared<open3d::geometry::PointCloud>();
	new_point_cloud->points_.reserve(max_label_count);
	if (!point_cloud->colors_.empty())
	{
		new_point_cloud->colors_.reserve(max_label_count);
	}
	if (!point_cloud->normals_.empty())
	{
		new_point_cloud->normals_.reserve(max_label_count);
	}

	const size_t size = point_cloud->points_.size();
	for (size_t i = 0; i < size; ++i)
	{
		if (labels[i] == max_label)
		{
			new_point_cloud->points_.emplace_back(point_cloud->points_[i]);
			if (!point_cloud->colors_.empty())
			{
				new_point_cloud->colors_.emplace_back(point_cloud->colors_[i]);
			}
			if (!point_cloud->normals_.empty())
			{
				new_point_cloud->normals_.emplace_back(point_cloud->normals_[i]);
			}
		}
	}
	
	open3d::io::WritePointCloudToPLY(path_filtered.string(), *new_point_cloud);

	/*
	 // �f�o�b�O�p
	std::map<int, Eigen::Vector3d> color_map;
	for (int label = 0; label < cluster_label; ++label)
	{
		color_map[label] = Eigen::Vector3d(static_cast<double>(rand()) / RAND_MAX, static_cast<double>(rand()) / RAND_MAX, static_cast<double>(rand()) / RAND_MAX);
	}
	
	for (size_t i = 0; i < size; ++i)
	{
		Eigen::Vector3d color = color_map[labels[i]];
		point_cloud->colors_[i] = color;
	}

	open3d::io::WritePointCloudToPLY(path_filtered.string(), *point_cloud);
	*/
}