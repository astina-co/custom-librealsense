#include "alignment.h"
#include "defines.h"

#include "Open3d/Visualization/Visualizer/Visualizer.h"
#include <fmt/format.h>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/opencv.hpp>
#include <spdlog/spdlog.h>

#include "board.h"

#include "open3d_090.h"

#include "exception.h"

typedef std::pair<std::vector<int>, std::vector<Eigen::Vector3d>> align_points;

void symb::scanner::Alignment::align_to_board(const CalibrationBoard &calibration_board, std::map<std::shared_ptr<helper::CameraInfoWithData>, CalibrationData> &calibration_data)
{
	SPDLOG_DEBUG("Alignment::align_to_board");
	const double cos_p = cos(M_PI);
	const double sin_p = sin(M_PI);
	for (auto &&data : calibration_data)
	{
		if (data.first->side() == Side::Rear)
		{
			Eigen::Vector3d flip_position(static_cast<double>(calibration_board.get_square_count_x()) * calibration_board.get_square_length(), 0, 0);

			Eigen::Matrix4d un_flip;
			un_flip << cos_p, 0, sin_p, flip_position(0), 0, 1, 0, flip_position(1), -sin_p, 0, cos_p, flip_position(2), 0, 0, 0, 1;
			data.second.pretransform(un_flip.inverse());

			Eigen::Matrix4d offset_thickness;
			offset_thickness << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, static_cast<double>(calibration_board.get_thickness()), 0, 0, 0, 1;
			data.second.pretransform(offset_thickness.inverse());
		}

		Eigen::Matrix4d offset_x;
		offset_x << 1, 0, 0, static_cast<double>(calibration_board.get_square_count_x()) * calibration_board.get_square_length() / 2.0f, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
		data.second.pretransform(offset_x.inverse());
	}
}

static align_points offset_by_thickness_for_intersection(const align_points &intersection_points, const symb::scanner::CalibrationBoard &calibration_board,
														 const Eigen::Vector3d &rough_direction = Eigen::Vector3d(0, 0, 1))
{
	const int w = calibration_board.get_square_count_x() - 1;
	const int h = calibration_board.get_square_count_y() - 1;

	std::vector<int> ret_ids;
	std::vector<Eigen::Vector3d> ret_points;
	for (int y = 0; y < h; y++)
	{
		for (int x = 0; x < w; x++)
		{
			const std::tuple<bool, int> intersection_id_result = symb::scanner::utility::coord2id(x, y, w, h);
			const bool id_valid = std::get<0>(intersection_id_result);
			const int intersection_id = std::get<1>(intersection_id_result);
			if (!id_valid)
			{
				SPDLOG_ERROR("Invalid coordinate ({},{})", FILENAME, __LINE__);
				continue;
			}

			const std::tuple<bool, Eigen::Vector3d> original_point_result = symb::scanner::utility::get_point(intersection_id, intersection_points);
			const bool original_found = std::get<0>(original_point_result);
			const Eigen::Vector3d &original_point = std::get<1>(original_point_result);
			if (!original_found)
			{
				continue;
			}

			align_points adj_points = std::make_pair(std::vector<int>(), std::vector<Eigen::Vector3d>());
			;
			for (int i = -2; i < 3; i++)
			{
				for (int j = -2; j < 3; j++)
				{
					const std::tuple<bool, int> adj_intersection_id_result = symb::scanner::utility::coord2id(x + j, y + i, w, h);
					const bool adj_id_valid = std::get<0>(adj_intersection_id_result);
					const int adj_intersection_id = std::get<1>(adj_intersection_id_result);
					if (!adj_id_valid)
					{
						// キャリブボード外の座標指定時の処理
						// 処理する必要がないので処理をスキップする
						continue;
					}
					std::tuple<bool, Eigen::Vector3d> adj_point_result = symb::scanner::utility::get_point(adj_intersection_id, intersection_points);
					const bool adj_found = std::get<0>(adj_point_result);
					const Eigen::Vector3d &adj_point = std::get<1>(adj_point_result);
					if (adj_found)
					{
						adj_points.first.emplace_back(adj_intersection_id);
						adj_points.second.emplace_back(adj_point);
					}
				}
			}
			if (adj_points.first.size() <= 4)
			{
				continue;
			}

			open3d::geometry::PointCloud cloud;
			cloud.points_ = adj_points.second;
			std::tuple<Eigen::Vector4d, std::vector<size_t>> result = SegmentPlane(0.01, 3, 250, cloud.points_);
			Eigen::Vector3d plane_normal = std::get<0>(result).block<3, 1>(0, 0);
			if (plane_normal.dot(rough_direction) < 0)
			{
				plane_normal *= -1;
			}

			Eigen::Vector3d offset_point = original_point + calibration_board.get_thickness() * plane_normal;
			ret_ids.emplace_back(intersection_id);
			ret_points.emplace_back(offset_point);
		}
	}

	return std::make_pair(ret_ids, ret_points);
}

static int convert_intersection_id_to_reverse(const int intersection_id, const symb::scanner::CalibrationBoard &calibration_board)
{
	const int w = calibration_board.get_square_count_x() - 1;
	const int x = intersection_id % w;
	const int y = static_cast<int>(static_cast<double>(intersection_id) / static_cast<double>(w));
	const int inv_x = w - x - 1;
	return w * y + inv_x;
}

static std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>> find_match_pairs(const align_points &front_alignment_points, const align_points &back_alignment_points,
																							  const symb::scanner::CalibrationBoard &calibration_board)
{
	const align_points &front_intersection_points = front_alignment_points;
	const align_points &back_intersection_points = offset_by_thickness_for_intersection(back_alignment_points, calibration_board);

	std::vector<Eigen::Vector3d> front_points;
	std::vector<Eigen::Vector3d> back_points;

	for (const int front_intersection_id : front_intersection_points.first)
	{
		const int back_id = convert_intersection_id_to_reverse(front_intersection_id, calibration_board);
		const std::tuple<bool, Eigen::Vector3d> front_back_relation = symb::scanner::utility::get_point(back_id, back_intersection_points);
		if (!std::get<0>(front_back_relation))
		{
			continue;
		}
		const std::tuple<bool, Eigen::Vector3d> front_point_result = symb::scanner::utility::get_point(front_intersection_id, front_intersection_points);
		const Eigen::Vector3d &front_point = std::get<1>(front_point_result);
		const std::tuple<bool, Eigen::Vector3d> back_point_result = symb::scanner::utility::get_point(back_id, back_intersection_points);
		const Eigen::Vector3d &back_point = std::get<1>(back_point_result);

		front_points.emplace_back(front_point);
		back_points.emplace_back(back_point);
	}

	return std::make_pair(front_points, back_points);
}

static double mat_norm_row_direction(const Eigen::MatrixXd &direction_matrix)
{
	double sum = 0.0;
	const size_t n = direction_matrix.rows();
	// SPDLOG_DEBUG("mat_norm_row_direction n={} dim={}", n, direction_matrix.cols());
	for (size_t i = 0; i < n; i++)
	{
		Eigen::VectorXd direction = direction_matrix.row(i).transpose();
		const double norm = direction.norm();
		sum += norm;
	}
	const double mean = sum / n;
	return mean;
}

// https://github.com/zjudmd1015/icp/blob/master/icp.cpp
/*
The MIT License(MIT)

Copyright(c) 2018 Miaoding Dai

Permission is hereby granted,
free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and / or sell copies of the Software,
and to permit persons to whom the Software is furnished to do so,
subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies
or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/
static std::tuple<Eigen::Matrix4d, double, double> best_fit_transform(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B)
{
	/*
	Notice:
	1/ JacobiSVD return U,S,V, S as a vector, "use U*S*Vt" to get original Matrix;
	2/ matrix type 'MatrixXd' or 'MatrixXf' matters.
	*/
	Eigen::Matrix4d T = Eigen::MatrixXd::Identity(4, 4);
	Eigen::Vector3d centroid_A = Eigen::Vector3d::Zero();
	Eigen::Vector3d centroid_B = Eigen::Vector3d::Zero();
	Eigen::MatrixXd AA = A;
	Eigen::MatrixXd BB = B;
	const int row = A.rows();

	for (int i = 0; i < row; i++)
	{
		centroid_A += A.block<1, 3>(i, 0).transpose();
		centroid_B += B.block<1, 3>(i, 0).transpose();
	}
	centroid_A /= row;
	centroid_B /= row;
	for (int i = 0; i < row; i++)
	{
		AA.block<1, 3>(i, 0) = A.block<1, 3>(i, 0) - centroid_A.transpose();
		BB.block<1, 3>(i, 0) = B.block<1, 3>(i, 0) - centroid_B.transpose();
	}

	const Eigen::MatrixXd H = AA.transpose() * BB;

	const Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
	const Eigen::MatrixXd U = svd.matrixU();
	const Eigen::VectorXd S = svd.singularValues();
	const Eigen::MatrixXd V = svd.matrixV();
	Eigen::MatrixXd Vt = V.transpose();

	Eigen::Matrix3d R = Vt.transpose() * U.transpose();

	if (R.determinant() < 0)
	{
		Vt.block<1, 3>(2, 0) *= -1;
		R = Vt.transpose() * U.transpose();
	}

	const Eigen::Vector3d t = centroid_B - R * centroid_A;

	T.block<3, 3>(0, 0) = R;
	T.block<3, 1>(0, 3) = t;

	// SPDLOG_DEBUG("Dims A({}x{}) B({}x{})", A.rows(), A.cols(), B.rows(), B.cols());
	// SPDLOG_DEBUG("T={}", matrix_to_string(T));
	const double before_error = mat_norm_row_direction(A - B);
	// SPDLOG_DEBUG("A={}", matrix_to_string(A));
	Eigen::MatrixXd transformed_a(A);
	transformed_a.conservativeResize(A.rows(), 4);
	transformed_a.col(3).setOnes(); // N x 4
	// SPDLOG_DEBUG("A with ones={}", matrix_to_string(transformed_a));
	transformed_a.transposeInPlace(); // 4 x N
	transformed_a = (T * transformed_a); // 4 x N
	transformed_a.transposeInPlace(); // N x 4
	// SPDLOG_DEBUG("A transformed={}", matrix_to_string(transformed_a));
	transformed_a.conservativeResize(A.rows(), 3); // N x 3
	// SPDLOG_DEBUG("A transformed shrinked={}", matrix_to_string(transformed_a));
	const double after_error = mat_norm_row_direction(transformed_a - B);

	return std::make_tuple(T, before_error, after_error);
}

static symb::scanner::CalibrationData &id_to_data(const std::string &id, std::map<std::shared_ptr<symb::scanner::helper::CameraInfoWithData>, symb::scanner::CalibrationData> &data)
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

void symb::scanner::Alignment::align_to_each_left(std::map<std::string, std::pair<std::vector<int>, open3d::geometry::PointCloud>> &corner_pointclouds,
												  std::map<std::shared_ptr<helper::CameraInfoWithData>, CalibrationData> &calibration_data)
{
	SPDLOG_DEBUG("Alignment::align_to_each_left");
	struct profile
	{
		Side side;
		char target_pole;
		char source_pole;
	};
	for (const struct profile p : std::vector<struct profile>{{Side::Front, 'A', 'D'}, {Side::Rear, 'C', 'B'}})
	{
		const char target_pole = p.target_pole;
		const char source_pole = p.source_pole;
		for (const char &level : std::vector<char>{'1', '2', '3', '4', '5'})
		{
			const std::string target_id = fmt::format("{}{}", target_pole, level);
			const std::string source_id = fmt::format("{}{}", source_pole, level);
			// SPDLOG_DEBUG("Source: {} Target: {}", source_id, target_id);
			const std::pair<std::vector<int>, open3d::geometry::PointCloud> target_alignment = corner_pointclouds[target_id];
			const std::pair<std::vector<int>, open3d::geometry::PointCloud> source_alignment = corner_pointclouds[source_id];
			std::vector<int> target_corner_ids = target_alignment.first;
			std::vector<int> source_corner_ids = source_alignment.first;

			CalibrationData &target_data = id_to_data(target_id, calibration_data);
			CalibrationData &source_data = id_to_data(source_id, calibration_data);

			std::vector<Eigen::Vector3d> target_points, source_points;
			const size_t target_count = target_corner_ids.size();
			assert(target_count == target_alignment.second.points_.size());
			for (size_t i = 0; i < target_count; i++)
			{
				const int corner_id = target_corner_ids[i];
				const std::vector<int>::iterator it = std::find(source_corner_ids.begin(), source_corner_ids.end(), corner_id);
				if (it != source_corner_ids.end())
				{
					// SPDLOG_DEBUG("target_pose {} source_pose", matrix4d_to_string(target_data.get_extrinsics()), matrix4d_to_string(source_data.get_extrinsics()));
					const int source_index = std::distance(source_corner_ids.begin(), it);
					const Eigen::Vector3d untransformed_target_point = target_alignment.second.points_[i];
					const Eigen::Vector3d untransformed_source_point = source_alignment.second.points_[source_index];
					const Eigen::Vector3d target_point = symb::scanner::utility::transform_single(target_data.get_extrinsics(), untransformed_target_point);
					const Eigen::Vector3d source_point = symb::scanner::utility::transform_single(source_data.get_extrinsics(), untransformed_source_point);
					target_points.emplace_back(target_point);
					source_points.emplace_back(source_point);
					// SPDLOG_DEBUG("target_corner_ids[{}]={} source_corner_ids[{}]={}", i, corner_id, source_index, source_corner_ids[source_index]);
					// SPDLOG_DEBUG("un-transformed Target Point ({} {} {}) Source Point ({} {} {})", untransformed_target_point.x(), untransformed_target_point.y(), untransformed_target_point.z(),
					//			 untransformed_source_point.x(), untransformed_source_point.y(), untransformed_source_point.z());
					// SPDLOG_DEBUG("transformed Target Point ({} {} {}) Source Point ({} {} {})", target_point.x(), target_point.y(), target_point.z(), source_point.x(), source_point.y(),
					// source_point.z());
				}
			}
			SPDLOG_DEBUG("Fit   [Source {}] => [Target {}] -- overlapping points: {}", source_id, target_id, source_points.size());
			const Eigen::MatrixXd source_mat = utility::convert_pointcloud_to_mat(source_points);
			assert(source_mat.rows() == source_points.size() && source_mat.cols() == 3);
			const Eigen::MatrixXd target_mat = utility::convert_pointcloud_to_mat(target_points);
			assert(target_mat.rows() == target_points.size() && target_mat.cols() == 3);
			// SPDLOG_DEBUG("Source Matrix {}", utility::matrix_to_string(source_mat));
			// SPDLOG_DEBUG("Target Matrix {}", utility::matrix_to_string(target_mat));
			const std::tuple<Eigen::Matrix4d, double, double> transform_result = best_fit_transform(source_mat, target_mat);
			const Eigen::Matrix4d &t = std::get<0>(transform_result);
			const double before_error = std::get<1>(transform_result);
			const double after_error = std::get<2>(transform_result);
			SPDLOG_DEBUG("Align [Source {}] => [Target {}] -- error_before: {:> 7.4f}[mm] error_after: {:> 7.4f}[mm]", source_id, target_id, before_error * 1000.0, after_error * 1000.0);
			// const Eigen::Matrix4d ext_before = source_data.get_extrinsics();
			source_data.pretransform(t);
			// SPDLOG_DEBUG("Transform Matrix {} [id:{}]\n{} -> {}", utility::matrix_to_string(t), source_id, 
			// 	utility::matrix_to_string(ext_before), utility::matrix_to_string(source_data.get_extrinsics()));
		}
	}
}

static align_points get_merged_points(const char level, const std::vector<char> &poles, std::map<std::string, std::pair<std::vector<int>, open3d::geometry::PointCloud>> &corner_pointclouds,
									  std::map<std::shared_ptr<symb::scanner::helper::CameraInfoWithData>, symb::scanner::CalibrationData> &calibration_data)
{
	std::map<int, std::vector<Eigen::Vector3d>> group;
	for (const char pole : poles)
	{
		const std::string id = fmt::format("{}{}", pole, level);
		const std::vector<int> corner_ids = corner_pointclouds[id].first;
		std::vector<Eigen::Vector3d> transformed = symb::scanner::utility::transform(id_to_data(id, calibration_data).get_extrinsics(), corner_pointclouds[id].second.points_);
		const size_t count = corner_ids.size();
		assert(count == transformed.size());
		for (size_t i = 0; i < count; i++)
		{
			const int corner_id = corner_ids[i];
			group[corner_id].emplace_back(transformed[i]);
		}
	}

	return symb::scanner::utility::merge_points(group);
}

void symb::scanner::Alignment::align_to_front_side_each_level(const CalibrationBoard &calibration_board,
															  std::map<std::string, std::pair<std::vector<int>, open3d::geometry::PointCloud>> &corner_pointclouds,
															  std::map<std::shared_ptr<helper::CameraInfoWithData>, CalibrationData> &calibration_data)
{
	SPDLOG_DEBUG("Alignment::align_to_front_side_each_level");
	for (const char level : {'1', '2', '3', '4', '5'})
	{
		const align_points front_alignment_pointcloud = get_merged_points(level, {'A', 'D'}, corner_pointclouds, calibration_data);
		const align_points back_alignment_pointcloud = get_merged_points(level, {'B', 'C'}, corner_pointclouds, calibration_data);

		const std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>> pair = find_match_pairs(front_alignment_pointcloud, back_alignment_pointcloud, calibration_board);

		const Eigen::MatrixXd target_mat = utility::convert_pointcloud_to_mat(pair.first);
		assert(target_mat.rows() == pair.second.size() && target_mat.cols() == 3);

		const Eigen::MatrixXd source_mat = utility::convert_pointcloud_to_mat(pair.second);
		assert(source_mat.rows() == pair.first.size() && source_mat.cols() == 3);

		const std::tuple<Eigen::Matrix4d, double, double> transform_result = best_fit_transform(source_mat, target_mat);
		const Eigen::Matrix4d &t = std::get<0>(transform_result);
		const double before_error = std::get<1>(transform_result);
		const double after_error = std::get<2>(transform_result);
		// SPDLOG_DEBUG("Source Matrix {}", utility::matrix_to_string(source_mat));
		// SPDLOG_DEBUG("Target Matrix {}", utility::matrix_to_string(target_mat));
		SPDLOG_DEBUG("Align [Source Pole{{B, C}} Level{{{0}}}] => [Target Pole{{A, D}} Level{{{0}}}] -- error_before: {1:> 7.4f} error_after: {2:> 7.4f}", 
			level, before_error * 1000.0, after_error * 1000.0);
		for (const char pole : std::vector<char>{'B', 'C'})
		{
			const std::string id = fmt::format("{}{}", pole, level);
			CalibrationData &calibration = id_to_data(id, calibration_data);
			// const Eigen::Matrix4d ext_before = calibration.get_extrinsics();
			calibration.pretransform(t);
			// SPDLOG_DEBUG("Transform Matrix {} [id:{}]\n{} -> {}", utility::matrix_to_string(t), id, utility::matrix_to_string(ext_before), utility::matrix_to_string(calibration.get_extrinsics()));
		}
	}
}

static std::tuple<std::vector<std::string>, std::vector<int>, std::vector<Eigen::Vector3d>>
extract_level(const symb::scanner::Side side, const char level, std::map<std::string, std::pair<std::vector<int>, open3d::geometry::PointCloud>> &corner_pointclouds,
			  std::map<std::shared_ptr<symb::scanner::helper::CameraInfoWithData>, symb::scanner::CalibrationData> &calibration_data)
{
	std::vector<std::string> device_ids;
	const std::vector<char> poles = side == symb::scanner::Side::Front ? std::vector<char>{'A', 'D'} : std::vector<char>{'B', 'C'};
	std::map<int, std::vector<Eigen::Vector3d>> key_updated_points;
	for (const char pole : poles)
	{
		const std::string id = fmt::format("{}{}", pole, level);
		const Eigen::Matrix4d extrinsic = id_to_data(id, calibration_data).get_extrinsics();
		device_ids.emplace_back(id);
		const std::pair<std::vector<int>, open3d::geometry::PointCloud> e = corner_pointclouds[id];
		const std::vector<int> corner_ids = e.first;
		const std::vector<Eigen::Vector3d> points = e.second.points_;
		const size_t point_count = corner_ids.size();
		assert(point_count == points.size());
		for (size_t i = 0; i < point_count; i++)
		{
			const int corner_id = corner_ids[i];
			key_updated_points[corner_id].emplace_back(symb::scanner::utility::transform_single(extrinsic, points[i]));
		}
	}
	align_points merged = symb::scanner::utility::merge_points(key_updated_points);
	return std::make_tuple(device_ids, merged.first, merged.second);
}

void symb::scanner::Alignment::align_to_center_front_back_together(std::map<std::string, std::pair<std::vector<int>, open3d::geometry::PointCloud>> &corner_pointclouds,
																   std::map<std::shared_ptr<helper::CameraInfoWithData>, CalibrationData> &calibration_data)
{
	SPDLOG_DEBUG("Alignment::align_to_center_front_back_together");
	for (const auto &j :
		 std::vector<std::pair<char, std::vector<char>>>{std::make_pair('3', std::vector<char>{'2', '4'}), std::make_pair('2', std::vector<char>{'1'}), std::make_pair('4', std::vector<char>{'5'})})
	{
		const char target_level = j.first;
		for (const char &source_level : j.second)
		{
			std::vector<std::string> target_ids, source_ids;
			std::vector<std::pair<Side, int>> paired_corners;
			std::vector<Eigen::Vector3d> target_points, source_points;
			for (const Side side : std::vector<Side>{Side::Front, Side::Rear})
			{
				std::tuple<std::vector<std::string>, std::vector<int>, std::vector<Eigen::Vector3d>> target = extract_level(side, target_level, corner_pointclouds, calibration_data);
				std::tuple<std::vector<std::string>, std::vector<int>, std::vector<Eigen::Vector3d>> source = extract_level(side, source_level, corner_pointclouds, calibration_data);

				const std::vector<std::string> &target_ids_child = std::get<0>(target);
				const std::vector<std::string> &source_ids_child = std::get<0>(source);
				std::vector<int> target_corner_ids_child = std::get<1>(target);
				std::vector<int> source_corner_ids_child = std::get<1>(source);
				const std::vector<Eigen::Vector3d> target_points_child = std::get<2>(target);
				const std::vector<Eigen::Vector3d> source_points_child = std::get<2>(source);

				target_ids.reserve(target_ids.size() + target_ids_child.size());
				target_ids.insert(target_ids.end(), target_ids_child.begin(), target_ids_child.end());
				source_ids.reserve(source_ids.size() + source_ids_child.size());
				source_ids.insert(source_ids.end(), source_ids_child.begin(), source_ids_child.end());

				const size_t point_count = target_points_child.size();
				assert(point_count == target_corner_ids_child.size());
				for (size_t i = 0; i < point_count; i++)
				{
					const int corner_id = target_corner_ids_child[i];
					const std::vector<int>::iterator it = std::find(source_corner_ids_child.begin(), source_corner_ids_child.end(), corner_id);
					if (it != source_corner_ids_child.end())
					{
						paired_corners.emplace_back(std::make_pair(side, corner_id));
						target_points.emplace_back(target_points_child[i]);
						const int source_index = std::distance(source_corner_ids_child.begin(), it);
						source_points.emplace_back(source_points_child[source_index]);
					}
				}
			}
			const size_t n = target_points.size();
			const Eigen::MatrixXd target_mat = utility::convert_pointcloud_to_mat(target_points);
			const Eigen::MatrixXd source_mat = utility::convert_pointcloud_to_mat(source_points);
			const std::tuple<Eigen::Matrix4d, double, double> transform_result = best_fit_transform(source_mat, target_mat);
			const Eigen::Matrix4d &t = std::get<0>(transform_result);
			const double &before_error = std::get<1>(transform_result);
			const double &after_error = std::get<2>(transform_result);
			// SPDLOG_DEBUG("Source Matrix {}", utility::matrix_to_string(source_mat));
			// SPDLOG_DEBUG("Target Matrix {}", utility::matrix_to_string(target_mat));
			SPDLOG_DEBUG("Align [Source Level{{{0}}}] => [Target Level{{{1}}}] -- point_count: {2} error_before: {3:> 7.4f} error_after: {4:> 7.4f}", source_level, target_level, n,
						 before_error * 1000.0, after_error * 1000.0);
			for (const std::string &id : source_ids)
			{
				CalibrationData &calibration = id_to_data(id, calibration_data);
				// const Eigen::Matrix4d ext_before = calibration.get_extrinsics();
				calibration.pretransform(t);
				// SPDLOG_DEBUG("Transform Matrix {} [id:{}]\n{} -> {}", utility::matrix_to_string(t), id, utility::matrix_to_string(ext_before), utility::matrix_to_string(calibration.get_extrinsics()));
			}
		}
	}
}

void symb::scanner::Alignment::align_to_floor(const symb::scanner::CalibrationBoard &calibration_board,
											  std::map<std::string, std::pair<std::vector<int>, open3d::geometry::PointCloud>> &corner_pointclouds,
											  std::map<std::shared_ptr<symb::scanner::helper::CameraInfoWithData>, symb::scanner::CalibrationData> &calibration_data,
											  std::map<std::string, symb::scanner::PointCloud> &raw_pointclouds)
{
	SPDLOG_DEBUG("Alignment::align_to_floor");
	const double normal_thresh = cos(M_PI / 6.0);
	const float board_half = static_cast<float>(calibration_board.get_square_count_x() * calibration_board.get_square_length()) / 2.0f;

	std::mutex m;
	std::shared_ptr<open3d::geometry::PointCloud> floor_points = std::make_shared<open3d::geometry::PointCloud>();
	std::vector<std::thread> threads;
	threads.reserve(4);

	for (const std::pair<std::string, PointCloud> pointcloud : raw_pointclouds)
	{
		SPDLOG_DEBUG("Has poincloud: {} : size({})", pointcloud.first, pointcloud.second.ptr()->points_.size());
	}

	const Eigen::Vector3d direction_up = Eigen::Vector3d(0, 1, 0);
	for (const char pole : std::vector<char>{'A', 'B', 'C', 'D'})
	{
		threads.emplace_back([&] {
			const std::string id = fmt::format("{}1", pole);
			const std::vector<Eigen::Vector3d> &untransformed = raw_pointclouds[id].ptr()->points_;

			// SPDLOG_DEBUG("Unmerged untransformed Point Count {} {}", id, untransformed.size());

			const Eigen::Matrix4d extrinsics = id_to_data(id, calibration_data).get_extrinsics();
			const std::vector<Eigen::Vector3d> transformed = symb::scanner::utility::transform(extrinsics, untransformed);
			open3d::geometry::PointCloud filtered;
			// SPDLOG_DEBUG("Unmerged transform Point Count {} {}", id, transformed.size());

			for (const Eigen::Vector3d &point : transformed)
			{
				if (point.y() >= -0.05)
					continue;
				if (point.x() >= board_half)
					continue;
				if (point.x() <= -board_half)
					continue;
				if (point.z() >= board_half)
					continue;
				if (point.z() <= -board_half)
					continue;
				filtered.points_.emplace_back(point);
			}
			EstimateNormals(filtered, open3d::geometry::KDTreeSearchParamHybrid(0.03, 30));
			OrientNormalsTowardsCameraLocation(filtered, utility::transform_single(extrinsics, Eigen::Vector3d::Zero()));

			// SPDLOG_DEBUG("Unmerged filtered Point Count {} {}", id, filtered.points_.size());

			std::vector<Eigen::Vector3d> normal_filtered_points;
			const size_t count = filtered.points_.size();
			assert(count == filtered.normals_.size());
			for (size_t i = 0; i < count; i++)
			{
				const Eigen::Vector3d normal = filtered.normals_[i];
				const double dot = normal.dot(direction_up);
				if (dot <= normal_thresh)
					continue;
				const Eigen::Vector3d point = filtered.points_[i];
				normal_filtered_points.emplace_back(point);
			}

			const Eigen::Vector3d color = utility::id_to_color(id);

			// SPDLOG_DEBUG("Unmerged Normal Point Count {} {}", id, normal_filtered_points.size());

			const size_t point_count = normal_filtered_points.size();
			{
				std::lock_guard<std::mutex> _(m);
				floor_points->points_.reserve(floor_points->points_.size() + point_count);
				floor_points->points_.insert(floor_points->points_.end(), normal_filtered_points.begin(), normal_filtered_points.end());
				const size_t color_count_before = floor_points->colors_.size();
				const size_t color_count_after = color_count_before + point_count;
				floor_points->colors_.reserve(floor_points->points_.size());
				for (size_t i = color_count_before; i < color_count_after; i++)
				{
					floor_points->colors_.emplace_back(color);
				}
			}
		});
	}

	for (std::thread &thread : threads)
	{
		thread.join();
	}

	PointCloud pointcloud(floor_points);
	pointcloud.extractLargestConnectedComponent(0.02, 30);
	floor_points = pointcloud.ptr();
	
	// open3d::visualization::Visualizer v;
	// v.CreateVisualizerWindow("floor", 1600, 900);
	// v.AddGeometry(floor_points);
	// v.Run();
	// v.DestroyVisualizerWindow();

	SPDLOG_DEBUG("Point Count size {}", floor_points->points_.size());

	const std::tuple<Eigen::Vector4d, std::vector<size_t>> plane = SegmentPlane(0.003, 3, 250, floor_points->points_);
	const Eigen::Vector4d &plane_model = std::get<0>(plane);

	const double to_floor_y = -plane_model[3] / plane_model[1];
	Eigen::Matrix4d down_to_floor = Eigen::Matrix4d::Identity();
	down_to_floor(1, 3) = -to_floor_y;

	SPDLOG_DEBUG("to_floor_y: {} [mm]", to_floor_y * 1000.0);

	const Eigen::Vector3d true_up = plane_model.block<3, 1>(0, 0);
	Eigen::Vector3d rotate_axis = Eigen::Vector3d(0, 1, 0).cross(true_up);
	rotate_axis.normalize();

	SPDLOG_DEBUG("rotate_axis: ({}, {}, {})", rotate_axis.x(), rotate_axis.y(), rotate_axis.z());

	const double rotate_angle = acos(true_up(1));

	SPDLOG_DEBUG("rotate_angle: {}deg", rotate_angle / M_PI * 180.0f);

	const Eigen::Matrix3d rotate_to_true_up_3d = Eigen::Matrix3d(Eigen::AngleAxisd(rotate_angle, rotate_axis));
	Eigen::Matrix4d rotate_to_true_up = Eigen::Matrix4d::Identity();
	rotate_to_true_up.block<3, 3>(0, 0) = rotate_to_true_up_3d;
	rotate_to_true_up = rotate_to_true_up.inverse();

	const Eigen::Matrix4d t = rotate_to_true_up * down_to_floor;
	
	for (auto &&d : calibration_data)
	{
		CalibrationData &calibration = d.second;
		// const Eigen::Matrix4d ext_before = calibration.get_extrinsics();
		calibration.pretransform(t);
		// SPDLOG_DEBUG("Transform Matrix {} [id:{}]\n{} -> {}", utility::matrix_to_string(t), d.first->id(), utility::matrix_to_string(ext_before), utility::matrix_to_string(calibration.get_extrinsics()));
	}
}

enum adjacent_side
{
	up, // 上
	back, // 裏隣(裏の面)
	side, // 表隣(同じ面)
	down, // 下
	count // 管理用
};

static std::vector<std::tuple<adjacent_side, std::string>> get_adj_ids(const std::string &id)
{
	std::vector<std::tuple<adjacent_side, std::string>> out;
	out.reserve(3);
	const char pole = id[0];
	const unsigned int pole_int = pole - 'A';
	const unsigned int level = id[1] - '0';

	// 上
	if (0 < level && level < 5)
	{
		out.emplace_back(std::make_tuple(adjacent_side::up, fmt::format("{}{}", pole, level + 1)));
	}

	// 裏隣
	static const char back_map_pole[4] = {'B', 'A', 'D', 'C'};
	out.emplace_back(std::make_tuple(adjacent_side::back, fmt::format("{}{}", back_map_pole[pole_int], level)));

	// 表隣
	static const char side_map_pole[4] = {'D', 'C', 'B', 'A'};
	out.emplace_back(std::make_tuple(adjacent_side::side, fmt::format("{}{}", side_map_pole[pole_int], level)));

	// 下
	if (1 < level && level < 6)
	{
		out.emplace_back(std::make_tuple(adjacent_side::down, fmt::format("{}{}", pole, level - 1)));
	}

	return out;
}

static align_points get_reverse_offset_points(const symb::scanner::CalibrationBoard &calibration_board, const align_points &input, const bool &rear_to_front)
{
	const Eigen::Vector3d &rough_direction = rear_to_front ? Eigen::Vector3d(0, 0, 1) : Eigen::Vector3d(0, 0, -1);
	const align_points offset_points = offset_by_thickness_for_intersection(input, calibration_board, rough_direction);
	const size_t count = offset_points.second.size();
	assert(count == offset_points.first.size());
	std::vector<int> out_ids;
	out_ids.reserve(count);
	for (size_t i = 0; i < count; i++)
	{
		const int out_id = convert_intersection_id_to_reverse(offset_points.first[i], calibration_board);
		out_ids.emplace_back(out_id);
	}
	return std::make_pair(out_ids, offset_points.second);
}

static std::tuple<align_points, align_points> extract_match_points(const align_points &source, const align_points &target)
{
	align_points source_copy = source;
	std::vector<int> overlap_corner_ids;
	std::vector<Eigen::Vector3d> source_points, target_points;
	const size_t count = target.first.size();
	assert(count == target.second.size());
	for (size_t i = 0; i < count; i++)
	{
		const int target_corner_id = target.first[i];
		std::vector<int>::iterator it = std::find(source_copy.first.begin(), source_copy.first.end(), target_corner_id);
		if (it != source_copy.first.end())
		{
			const int source_index = std::distance(source_copy.first.begin(), it);
			overlap_corner_ids.emplace_back(target_corner_id);
			target_points.emplace_back(target.second[i]);
			source_points.emplace_back(source_copy.second[source_index]);
		}
	}
	return std::make_tuple(std::make_pair(overlap_corner_ids, source_points), std::make_pair(overlap_corner_ids, target_points));
}

void symb::scanner::Alignment::realign_all(const CalibrationBoard &calibration_board, std::map<std::string, std::pair<std::vector<int>, open3d::geometry::PointCloud>> &corner_pointclouds,
										   std::map<std::shared_ptr<helper::CameraInfoWithData>, CalibrationData> &calibration_data)
{
	SPDLOG_DEBUG("Alignment::realign_all");
	for (const char &level : std::vector<char>{'3', '4', '2', '5', '1'})
	{
		for (const char &pole : std::vector<char>{'A', 'D', 'C', 'B'})
		{
			const Side side = pole == 'A' || pole == 'D' ? Side::Front : Side::Rear;
			const std::string id = fmt::format("{}{}", pole, level);
			const std::vector<std::tuple<adjacent_side, std::string>> adjacent_ids = get_adj_ids(id);
			std::map<int, std::vector<Eigen::Vector3d>> adj_points;
			std::ostringstream oss;
			for (const std::tuple<adjacent_side, std::string> &adj : adjacent_ids)
			{
				const adjacent_side &adj_side = std::get<0>(adj);
				const std::string &adj_id = std::get<1>(adj);
				const std::vector<int> adj_corner_ids = corner_pointclouds[adj_id].first;
				const Eigen::Matrix4d adj_extrinsics = id_to_data(adj_id, calibration_data).get_extrinsics();
				const std::vector<Eigen::Vector3d> adj_untransformed = corner_pointclouds[adj_id].second.points_;
				const std::vector<Eigen::Vector3d> adj_transformed = utility::transform(adj_extrinsics, adj_untransformed);
				align_points adj_alignment_points = std::make_pair(adj_corner_ids, adj_transformed);
				if (adj_side == adjacent_side::back)
				{
					const bool rear_to_front = side == Side::Front;
					adj_alignment_points = get_reverse_offset_points(calibration_board, adj_alignment_points, rear_to_front);
					oss << "reversed(" << adj_id << ")|";
				}
				else
				{
					oss << adj_id << "|";
				}
				const size_t count = adj_alignment_points.first.size();
				assert(count == adj_alignment_points.second.size());
				for (size_t i = 0; i < count; i++)
				{
					adj_points[adj_alignment_points.first[i]].emplace_back(adj_alignment_points.second[i]);
				}
			}
			CalibrationData &calibration = id_to_data(id, calibration_data);
			const Eigen::Matrix4d extrinsics = calibration.get_extrinsics();
			const std::vector<Eigen::Vector3d> untransformed = corner_pointclouds[id].second.points_;
			const std::vector<Eigen::Vector3d> transformed = utility::transform(extrinsics, untransformed);
			const align_points points = std::make_pair(corner_pointclouds[id].first, transformed);

			const align_points adj_points_merged = symb::scanner::utility::merge_points(adj_points);

			const size_t merge_count = adj_points_merged.first.size();

			const std::tuple<align_points, align_points> match = extract_match_points(points, adj_points_merged);
			const align_points &source_points = std::get<0>(match);
			const align_points &target_points = std::get<1>(match);

			const size_t match_count = source_points.first.size();

			const Eigen::MatrixXd source_mat = utility::convert_pointcloud_to_mat(source_points.second);
			const Eigen::MatrixXd target_mat = utility::convert_pointcloud_to_mat(target_points.second);
			const std::tuple<Eigen::Matrix4d, double, double> transform_result = best_fit_transform(source_mat, target_mat);

			// SPDLOG_DEBUG("source: {}", utility::matrix_to_string(source_mat));
			// SPDLOG_DEBUG("target: {}", utility::matrix_to_string(target_mat));

			const Eigen::Matrix4d &t = std::get<0>(transform_result);
			const double &before_error = std::get<1>(transform_result);
			const double &after_error = std::get<2>(transform_result);
			// SPDLOG_DEBUG("Source Matrix {}", utility::matrix_to_string(source_mat));
			// SPDLOG_DEBUG("Target Matrix {}", utility::matrix_to_string(target_mat));
			// SPDLOG_DEBUG("t: \n{}", symb::scanner::utility::matrix_to_string(t));
			SPDLOG_DEBUG("Re-Align [Source |{0}] => [Target {1}] -- merges: {2:<3}, matches: {3} error_before: {4:> 7.4f}[mm] error_after: {5:> 7.4f}[mm]", oss.str(), id, merge_count, match_count,
						 before_error * 1000.0, after_error * 1000.0);
			// const Eigen::Matrix4d ext_before = calibration.get_extrinsics();
			calibration.pretransform(t);
			// SPDLOG_DEBUG("Transform Matrix {} [id:{}]\n{} -> {}", utility::matrix_to_string(t), id, utility::matrix_to_string(ext_before), utility::matrix_to_string(calibration.get_extrinsics()));
		}
	}
}

void symb::scanner::Alignment::dump_evaluation(const CalibrationBoard &calibration_board, std::map<std::string, std::pair<std::vector<int>, open3d::geometry::PointCloud>> &corner_pointclouds,
											   std::map<std::shared_ptr<helper::CameraInfoWithData>, CalibrationData> &calibration_data)
{
	std::map<int, std::vector<Eigen::Vector3d>> group;
	for (const auto &calibration : calibration_data)
	{
		const std::string id = calibration.first->id();
		const std::vector<Eigen::Vector3d> transformed = utility::transform(calibration.second.get_extrinsics(), corner_pointclouds[id].second.points_);
		align_points alignment_points = std::make_pair(corner_pointclouds[id].first, transformed);
		if (calibration.first->side() == Side::Rear)
		{
			alignment_points = get_reverse_offset_points(calibration_board, std::make_pair(alignment_points.first, alignment_points.second), true);
		}
		const size_t count = alignment_points.first.size();
		assert(count == alignment_points.second.size());
		for (size_t i = 0; i < count; i++)
		{
			group[alignment_points.first[i]].emplace_back(alignment_points.second[i]);
		}
	}

	std::vector<double> diffs;

	const align_points means = utility::merge_points(group);
	const size_t count = means.first.size();
	assert(count == means.second.size());
	if (count <= 0)
	{
		SPDLOG_ERROR("Evaluate: Failed [No Points]");
		return;
	}
	diffs.reserve(count);
	for (size_t i = 0; i < count; i++)
	{
		const Eigen::RowVector3d mean_point = means.second[i].transpose();
		const Eigen::MatrixXd group_mat = utility::convert_pointcloud_to_mat(group[means.first[i]]);
		const size_t n = group_mat.rows();
		Eigen::MatrixXd means_mat = Eigen::MatrixXd::Zero(n, 3);
		for (size_t j = 0; j < n; j++)
		{
			means_mat.row(j) = mean_point;
		}
		const double diff = mat_norm_row_direction(group_mat - means_mat);
		diffs.emplace_back(diff * 1000.0);
	}
	std::sort(diffs.begin(), diffs.end());
	const double sum = std::accumulate(diffs.begin(), diffs.end(), decltype(diffs)::value_type(0));
	const double mean = sum / static_cast<double>(count);
	const double median = (count % 2 == 0) ? (diffs[count / 2 - 1] + diffs[count / 2]) / 2 : diffs[count / 2];
	const double max = diffs[count - 1];
	SPDLOG_INFO("Evaluation: Alignment Error [Mean: {0:> 10.5f}] [Median: {1:> 10.5f}] [Max: {2:> 10.5f}]", mean, median, max);
}
