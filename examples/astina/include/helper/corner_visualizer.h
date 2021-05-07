#pragma once

#include <map>
#include <memory>

#include "fmt/format.h"

#include "calibration.h"
#include "helper/camera_info_with_data.h"

#include <Open3D/Geometry/Pointcloud.h>
#include <Open3D/Visualization/Visualizer/Visualizer.h>

namespace symb
{
namespace scanner
{

namespace helper
{

class CornerVisualizer
{
public:
	CornerVisualizer(const std::map<std::string, std::pair<std::vector<int>, open3d::geometry::PointCloud>> &corner_pointcloud) : corner_pointcloud_(corner_pointcloud){};

	void show(std::map<std::shared_ptr<CameraInfoWithData>, CalibrationData> &calibration_data, const std::string &title = "Corners") const;

private:
	std::map<std::string, std::pair<std::vector<int>, open3d::geometry::PointCloud>> corner_pointcloud_;
};

} // namespace helper

} // namespace scanner

} // namespace symb