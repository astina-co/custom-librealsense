#include "utility.h"

#include <spdlog/spdlog.h>
#include "exception.h"
#include <calibration.h>
#include <helper/camera_info_with_data.h>
#include "defines.h"

bool symb::scanner::utility::is_dir(const std::experimental::filesystem::path &path) { return std::experimental::filesystem::exists(path) && std::experimental::filesystem::is_directory(path); }

bool symb::scanner::utility::mkdir_if_not_exist(const std::experimental::filesystem::path &path) noexcept
{
	try
	{
		// ディレクトリがなく、作成に失敗した場合のみ false
		return !(!is_dir(path) && !std::experimental::filesystem::create_directories(path));
	}
	catch (std::experimental::filesystem::filesystem_error &e)
	{
		SPDLOG_ERROR("filesystem::create_directories({}) FAILED with error {}", path.string(), e.what());
		return false;
	}
}

std::string symb::scanner::utility::get_date_time()
{
	const std::chrono::system_clock::time_point p = std::chrono::system_clock::now();
	const __time64_t t = std::chrono::system_clock::to_time_t(p);
	struct tm tt = {};
	localtime_s(&tt, &t);

	return fmt::format("{:04}_{:02}_{:02}_{:02}_{:02}_{:02}_{:03}", //
					   tt.tm_year + 1900, tt.tm_mon + 1, tt.tm_mday, tt.tm_hour, tt.tm_min, tt.tm_sec, //
					   std::chrono::duration_cast<std::chrono::milliseconds>(p.time_since_epoch()).count() % 1000);
}

bool symb::scanner::utility::contains(const std::string &find_in, const std::string &target) { return find_in.find(target) != std::string::npos; }

std::string symb::scanner::utility::join(const std::vector<std::string> &str_list, std::string &out) {
	for (std::string i: str_list) {
		out += i;
	}
	return out;
}

std::vector<std::string> symb::scanner::utility::split(const std::string &str, const std::string &separator)
{
	const size_t separator_length = separator.length();
	std::vector<std::string> result;

	if (separator_length == 0)
	{
		result.emplace_back(str);
	}
	else
	{
		size_t offset = std::string::size_type(0);

		while (true)
		{
			const size_t pos = str.find(separator, offset);

			if (pos == std::string::npos)
			{
				result.emplace_back(str.substr(offset));
				break;
			}

			result.emplace_back(str.substr(offset, pos - offset));
			offset = pos + separator_length;
		}
	}

	return result;
}

rs2::sensor symb::scanner::utility::get_color_sensor(const rs2::device &device)
{
	for (const rs2::sensor &sensor : device.query_sensors())
	{
		if (sensor.supports(rs2_camera_info::RS2_CAMERA_INFO_NAME) && std::string(sensor.get_info(rs2_camera_info::RS2_CAMERA_INFO_NAME)) == "RGB Camera")
		{
			return sensor;
		}
	}

	throw error::InvalidParameterError("Unable to retrieve Color Sensor from given device");
}

std::mutex m_;

rs2::video_stream_profile symb::scanner::utility::get_profile(const rs2::sensor &sensor, const rs2_stream stream_type, const rs2_format format, const int fps, const int width, const int height, const int index)
{
	// sensor.get_stream_profiles()をマルチスレッドで同時実行すると低確率でクラッシュ
	std::lock_guard<std::mutex> _(m_);
	try
	{
		for (const rs2::stream_profile &raw_profile : sensor.get_stream_profiles())
		{
			if (raw_profile.is<rs2::video_stream_profile>())
			{
				const rs2::video_stream_profile &profile = raw_profile.as<rs2::video_stream_profile>();

				if (profile.stream_type() == stream_type && profile.format() == format && profile.fps() == fps && profile.width() == width && profile.height() == height && profile.stream_index() == index)
				{
					return profile;
				}
			}
		}
	}
	catch (const rs2::error &e)
	{
		throw error::RecoverableError(fmt::format("symb::scanner::utility::get_profile threw {} at {}({})", e.what(), e.get_failed_function(), e.get_failed_args()));
	}
	throw error::InvalidParameterError(fmt::format("Invalid Stream Configuration Type: {}, Format: {}, FPS: {}, Resolution: {}x{}, Index: {}", rs2_stream_to_string(stream_type), rs2_format_to_string(format), fps, width, height, index));
}

Eigen::Vector3d symb::scanner::utility::transform_single(const Eigen::Matrix4d &matrix, const Eigen::Vector3d &point)
{
	const Eigen::Vector3d result = (matrix * Eigen::Vector4d(point.x(), point.y(), point.z(), 1)).block<3, 1>(0, 0);
	return result;
}

std::vector<Eigen::Vector3d> symb::scanner::utility::transform(const Eigen::Matrix4d &matrix, const std::vector<Eigen::Vector3d> &pointcloud)
{
	std::vector<Eigen::Vector3d> transformed;
	transformed.reserve(pointcloud.size());
	for (const Eigen::Vector3d &point : pointcloud)
	{
		transformed.emplace_back(symb::scanner::utility::transform_single(matrix, point));
	}
	return transformed;
}

std::tuple<bool, int> symb::scanner::utility::coord2id(const int x, const int y, const int w, const int h)
{
	if (x < 0 || x >= w || y < 0 || y >= h)
	{
		return std::make_tuple(false, 0);
	}
	return std::make_tuple(true, w * y + x);
}

std::tuple<bool, Eigen::Vector3d> symb::scanner::utility::get_point(const int id, const std::pair<std::vector<int>, std::vector<Eigen::Vector3d>> &points)
{
	const size_t size = points.first.size();
	for (size_t k = 0; k < size; k++)
	{
		// adj_point = intersection_points.get(adj_intersection_id, None) ???
		if (points.first[k] == id)
		{
			return std::make_tuple(true, points.second[k]);
		}
	}
	return std::make_tuple(false, Eigen::Vector3d());
}

Eigen::Vector3d symb::scanner::utility::sum(const std::vector<Eigen::Vector3d> &points)
{
	Eigen::Vector3d sum(0, 0, 0);
	for (const Eigen::Vector3d &point : points)
	{
		sum += point;
	}
	return sum;
}

Eigen::Vector3d symb::scanner::utility::mean(const std::vector<Eigen::Vector3d> &points) { return sum(points) / points.size(); }

Eigen::MatrixXd symb::scanner::utility::convert_pointcloud_to_mat(const std::vector<Eigen::Vector3d> &pointcloud)
{
	const size_t n = pointcloud.size();
	Eigen::MatrixXd mat = Eigen::MatrixXd::Zero(n, 3);
	for (size_t i = 0; i < n; i++)
	{
		mat.block<1, 3>(i, 0) = pointcloud[i].transpose();
	}
	return mat;
}

std::pair<std::vector<int>, std::vector<Eigen::Vector3d>> symb::scanner::utility::merge_points(const std::map<int, std::vector<Eigen::Vector3d>> &map)
{
	const size_t group_count = map.size();
	std::vector<int> corner_ids;
	corner_ids.reserve(group_count);
	std::vector<Eigen::Vector3d> points;
	points.reserve(group_count);
	for (const auto &subset : map)
	{
		corner_ids.emplace_back(subset.first);
		const Eigen::Vector3d mean = symb::scanner::utility::mean(subset.second);
		points.emplace_back(mean);
	}
	return std::make_pair(corner_ids, points);
}

Eigen::Vector3d symb::scanner::utility::id_to_color(const std::string &id)
{
	const static Eigen::Vector3d level_add = Eigen::Vector3d::Ones() * 127.0 / 255.0;
	const static Eigen::Vector3d pole_map[4] = {
		Eigen::Vector3d(1, 0, 0),
		Eigen::Vector3d(127.0 / 255.0, 1, 0),
		Eigen::Vector3d(0, 1, 1),
		Eigen::Vector3d(127.0 / 255.0, 0, 1),
	};
	const Eigen::Vector3d color = (pole_map[id[0] - 'A'] + level_add * (id[1] - '1')).normalized();
	return color;
}

std::string symb::scanner::utility::matrix_to_string(const Eigen::MatrixXd &matrix)
{
	const size_t rows = matrix.rows();
	const size_t cols = matrix.cols();
	std::ostringstream oss;
	oss << "\n[";
	for (size_t i = 0; i < rows; i++)
	{
		oss << "[";
		for (size_t j = 0; j < cols - 1; j++)
		{
			oss << fmt::format("{: .6f}", matrix(i, j)) << ", ";
		}
		oss << fmt::format("{: .6f}]\n", matrix(i, cols - 1));
	}
	oss << "]\n";
	return oss.str();
}