#include "helper/corner_visualizer.h"
#include "exception.h"

#include "utility.h"

#include "defines.h"

symb::scanner::CalibrationData &id_to_data_(const std::string &id, std::map<std::shared_ptr<symb::scanner::helper::CameraInfoWithData>, symb::scanner::CalibrationData> &data)
{
	for (auto &&t : data)
	{
		if (t.first->id() == id)
		{
			return t.second;
		}
	}
	throw symb::scanner::error::BadStateError(fmt::format("Invalid ID at id_to_info ({}:{})", FILENAME, __LINE__));
}

void symb::scanner::helper::CornerVisualizer::show(std::map<std::shared_ptr<symb::scanner::helper::CameraInfoWithData>, symb::scanner::CalibrationData> &calibration_data,
												   const std::string &title) const
{
	static const Eigen::Vector3d x_translate_base(1, 0, 0);
	std::shared_ptr<open3d::geometry::PointCloud> pointcloud = std::make_shared<open3d::geometry::PointCloud>();
	for (const auto &device: corner_pointcloud_)
	{
		const std::string id = device.first;
		const char pole = id[0];
		const Eigen::Vector3d x_translate = x_translate_base * (pole - 'A');
		const std::vector<int> corner_ids = device.second.first;
		std::vector<Eigen::Vector3d> untransformed = device.second.second.points_;
		CalibrationData &calibration = id_to_data_(id, calibration_data);
		std::vector<Eigen::Vector3d> transformed = utility::transform(calibration.get_extrinsics(), untransformed);
		for (Eigen::Vector3d &point : transformed)
		{
			// point += x_translate;
		}
		const size_t count_before = pointcloud->points_.size();
		const size_t count_after = count_before + transformed.size();
		pointcloud->points_.reserve(count_after);
		pointcloud->points_.insert(pointcloud->points_.end(), transformed.begin(), transformed.end());

		const Eigen::Vector3d color = utility::id_to_color(id);
		pointcloud->colors_.reserve(count_after);
		for (size_t i = count_before; i < count_after; i++)
		{
			pointcloud->colors_.emplace_back(color);
		}
	}

	open3d::visualization::Visualizer visualizer;
	visualizer.CreateVisualizerWindow(title, 1600, 900);
	visualizer.AddGeometry(pointcloud);
	visualizer.Run();
	visualizer.DestroyVisualizerWindow();
}
