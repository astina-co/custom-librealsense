#pragma once

#define _USE_MATH_DEFINES

#include "pointcloud.h"
#include "helper/camera_info_with_data.h"

namespace symb
{
namespace scanner
{

namespace Alignment
{

void align_to_board(const CalibrationBoard &calibration_board, std::map<std::shared_ptr<helper::CameraInfoWithData>, CalibrationData> &calibration_data);

void align_to_each_left(std::map<std::string, std::pair<std::vector<int>, open3d::geometry::PointCloud>> &corner_pointclouds,
						std::map<std::shared_ptr<helper::CameraInfoWithData>, CalibrationData> &calibration_data);

void align_to_front_side_each_level(const CalibrationBoard &calibration_board, 
	std::map<std::string, std::pair<std::vector<int>, open3d::geometry::PointCloud>> &corner_pointclouds,
	std::map<std::shared_ptr<helper::CameraInfoWithData>, CalibrationData> &calibration_data);

void align_to_center_front_back_together(std::map<std::string, std::pair<std::vector<int>, open3d::geometry::PointCloud>> &corner_pointclouds,
										 std::map<std::shared_ptr<helper::CameraInfoWithData>, CalibrationData> &calibration_data);

void align_to_floor(const symb::scanner::CalibrationBoard &calibration_board, std::map<std::string, std::pair<std::vector<int>, open3d::geometry::PointCloud>> &corner_pointclouds,
					std::map<std::shared_ptr<symb::scanner::helper::CameraInfoWithData>, symb::scanner::CalibrationData> &calibration_data,
					std::map<std::string, symb::scanner::PointCloud> &raw_pointclouds);

void realign_all(const CalibrationBoard &calibration_board, std::map<std::string, std::pair<std::vector<int>, 
					open3d::geometry::PointCloud>> &corner_pointclouds,
					std::map<std::shared_ptr<helper::CameraInfoWithData>, CalibrationData> &calibration_data);

void dump_evaluation(const CalibrationBoard &calibration_board, std::map<std::string, std::pair<std::vector<int>, open3d::geometry::PointCloud>> &corner_pointclouds,
					 std::map<std::shared_ptr<helper::CameraInfoWithData>, CalibrationData> &calibration_data);

};

} // namespace scanner

} // namespace symb