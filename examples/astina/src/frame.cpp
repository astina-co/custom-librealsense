#include "frame.h"
#include "exception.h"

#include "open3d_090.h"
#include "utility.h"

#include "defines.h"

symb::scanner::Frame symb::scanner::Frame::get_aligned_to(const rs2_stream stream_type) const
{
	if (!initialized_)
	{
		throw error::BadStateError(fmt::format("Frame is Empty ({}:{})", FILENAME, __LINE__));
	}
	if (!from_stream(rs2_stream::RS2_STREAM_DEPTH))
	{
		throw error::BadStateError(fmt::format("Frame does not contain {} Frame ({}:{})", rs2_stream_to_string(rs2_stream::RS2_STREAM_DEPTH), FILENAME, __LINE__));
	}
	if (!from_stream(stream_type))
	{
		throw error::BadStateError(fmt::format("Frame does not contain {} Frame ({}:{})", rs2_stream_to_string(stream_type), FILENAME, __LINE__));
	}
	try
	{
		return Frame(rs2::align(stream_type).process(frameset_));
	}
	catch (const rs2::error &e)
	{
		throw error::UnRecoverableError(fmt::format("Frame Unable to align frameset [{}({}) {}] ({}:{})", e.get_failed_function(), e.get_failed_args(), e.what(), FILENAME, __LINE__));
	}
}

rs2::frameset symb::scanner::Frame::raw() const
{
	if (!initialized_)
	{
		throw error::BadStateError(fmt::format("Frame is Empty ({}:{})", FILENAME, __LINE__));
	}
	return frameset_;
}

bool symb::scanner::Frame::from_stream(const rs2_stream stream_type) const
{
	if (!initialized_)
	{
		throw error::BadStateError(fmt::format("Frame is Empty ({}:{})", FILENAME, __LINE__));
	}
	return frames_.find(stream_type) != frames_.end();
}

cv::Mat symb::scanner::Frame::get_image(const rs2_stream stream_type, const bool convert, const bool filter) const
{
	if (!initialized_)
	{
		throw error::BadStateError(fmt::format("Frame is Empty ({}:{})", FILENAME, __LINE__));
	}
	if (!from_stream(stream_type))
	{
		throw error::BadStateError(fmt::format("Frame does not contain {} stream ({}:{})", rs2_stream_to_string(stream_type), FILENAME, __LINE__));
	}

	rs2::frameset tmp_frameset = frameset_;
	
	if (stream_type == rs2_stream::RS2_STREAM_DEPTH && filter)
	{
		tmp_frameset = rs2::align(rs2_stream::RS2_STREAM_COLOR).process(tmp_frameset);
	}

	rs2::frame frame;
	switch (stream_type)
	{
	default:
	case rs2_stream::RS2_STREAM_DEPTH:
		frame = tmp_frameset.get_depth_frame();
		if (filter)
		{
			const rs2::disparity_transform depth_to_disparity(true); //!<
			const rs2::disparity_transform disparity_to_depth(false); //!<
			const rs2::spatial_filter spatial_filter(0.71f, 25, 5.0f, 0); //!<
			const rs2::threshold_filter threshold_filter(0.16f, 1.4f); //!<
			frame = depth_to_disparity.process(frame);
			frame = spatial_filter.process(frame);
			frame = disparity_to_depth.process(frame);
			frame = threshold_filter.process(frame);
		}
		break;
	case rs2_stream::RS2_STREAM_COLOR:
		frame = tmp_frameset.get_color_frame();
		break;
	case rs2_stream::RS2_STREAM_INFRARED:
		frame = tmp_frameset.get_infrared_frame();
		break;
	};

	// SPDLOG_TRACE("Creating Image addr: {}", frame.get_data());
	const rs2::video_stream_profile profile = frame.get_profile().as<rs2::video_stream_profile>();
	const cv::Size size(profile.width(), profile.height());

	int type = CV_8UC3;
	const void *data = frame.get_data();

	const rs2::colorizer color_mapper;

	switch (profile.format())
	{
	case RS2_FORMAT_Z16:
		if (convert)
		{
			data = color_mapper.colorize(frame).get_data();
		}
		else
		{
			type = CV_16UC1;
		}
		break;
	case RS2_FORMAT_Y8:
		type = CV_8UC1;
		break;
	default:
		break;
	}
	return cv::Mat(size, type, const_cast<void *>(data), cv::Mat::AUTO_STEP).clone();
}

std::pair<std::vector<int>, std::vector<std::vector<cv::Point2f>>> symb::scanner::CalibrationFrame::get_markers()
{
	if (has_markers)
	{
		return markers_;
	}

	const rs2::video_stream_profile profile = get_calibration_profile();

	const rs2::video_frame frame = raw().get_color_frame();
	if (!frame)
	{
		throw error::InvalidParameterError(fmt::format("Frame has no Color Frame ({}:{})", FILENAME, __LINE__));
	}

	cv::Mat clean_canvas = cv::Mat(cv::Size(profile.width(), profile.height()), CV_8UC3, const_cast<void *>(frame.get_data()), cv::Mat::AUTO_STEP);
	cv::cvtColor(clean_canvas, clean_canvas, CV_BGR2GRAY);
	// cv::cvtColor(clean_canvas, clean_canvas, CV_GRAY2BGR);

	std::vector<int> ids;
	std::vector<std::vector<cv::Point2f>> corners;
	std::vector<std::vector<cv::Point2f>> rejected;
	const std::shared_ptr<cv::aruco::DetectorParameters> d_parameters = cv::aruco::DetectorParameters::create();
	d_parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
	cv::aruco::detectMarkers(clean_canvas, dictionary_, corners, ids, d_parameters, rejected, get_intrinsics(), get_dist_coeffs());
	//cv::aruco::refineDetectedMarkers(clean_canvas, board_, corners, ids, rejected, get_intrinsics(), get_dist_coeffs(), \
	//	10.0f, 3.f, true, cv::noArray(), d_parameters);

	if (ids.empty())
	{
		throw error::NoMarkersFoundError(fmt::format("Unable to detect any markers ({}:{})", FILENAME, __LINE__), *this);
	}

	markers_ = std::make_pair(ids, corners);

	return markers_;
}

std::pair<std::vector<int>, std::vector<cv::Point2f>> symb::scanner::CalibrationFrame::get_charuco()
{
	if (has_charuco)
	{
		return charuco_;
	}

	const std::pair<std::vector<int>, std::vector<std::vector<cv::Point2f>>> markers = get_markers();

	const std::vector<int> ids = markers.first;
	const std::vector<std::vector<cv::Point2f>> corners = markers.second;

	std::vector<int> charuco_ids;
	std::vector<cv::Point2f> charuco_corners;

	const rs2::video_frame frame = raw().get_color_frame();
	if (!frame)
	{
		throw error::InvalidParameterError(fmt::format("Frame has no Color Frame ({}:{})", FILENAME, __LINE__));
	}

	const rs2::video_stream_profile profile = get_calibration_profile();
	cv::Mat clean_canvas = cv::Mat(cv::Size(profile.width(), profile.height()), CV_8UC3, const_cast<void *>(frame.get_data()), cv::Mat::AUTO_STEP);
	cv::cvtColor(clean_canvas, clean_canvas, CV_BGR2GRAY);
	// cv::cvtColor(clean_canvas, clean_canvas, CV_GRAY2BGR);

	// determine Charuco corners (マーカーの角の決定)
	cv::aruco::interpolateCornersCharuco(corners, ids, clean_canvas, board_, charuco_corners, charuco_ids, get_intrinsics(), get_dist_coeffs());

	if (charuco_ids.empty())
	{
		throw error::InsufficientMarkersError(fmt::format("Unable to determine charuco corners. Found {}. ({}:{})", ids.size(), FILENAME, __LINE__), static_cast<int>(ids.size()), *this);
	}

	charuco_ = std::make_pair(charuco_ids, charuco_corners);
	return charuco_;
}

std::pair<std::vector<cv::Vec3d>, std::vector<cv::Vec3d>> symb::scanner::CalibrationFrame::get_single_pose()
{
	if (has_single_pose)
	{
		return single_pose_;
	}

	if (board_ == nullptr)
	{
		throw error::BadStateError(fmt::format("Parameter Board must be non-null at get_single_pose() ({}:{})", FILENAME, __LINE__));
	}

	const std::pair<std::vector<int>, std::vector<std::vector<cv::Point2f>>> markers = get_markers();

	std::vector<cv::Vec3d> r_vectors;
	std::vector<cv::Vec3d> t_vectors;
	cv::aruco::estimatePoseSingleMarkers(markers.second, board_->getMarkerLength(), get_intrinsics(), get_dist_coeffs(), r_vectors, t_vectors);

	single_pose_ = std::make_pair(r_vectors, t_vectors);

	return single_pose_;
}
std::pair<cv::Vec3d, cv::Vec3d> symb::scanner::CalibrationFrame::get_pose()
{
	if (has_charuco_pose)
	{
		return charco_pose_;
	}

	const std::pair<std::vector<int>, std::vector<cv::Point2f>> charuco = get_charuco();
	cv::Vec3d r_vec, t_vec;
	const bool valid = cv::aruco::estimatePoseCharucoBoard(charuco.second, charuco.first, board_, get_intrinsics(), get_dist_coeffs(), r_vec, t_vec);

	// 得られたマーカー情報を基にボードの原点を推定
	if (!valid)
	{
		throw error::CornerEstimationError(fmt::format("Frame unable to estimate board corner. Found {}. ({}:{})", charuco.first.size(), FILENAME, __LINE__), static_cast<int>(charuco.first.size()),
										   *this);
	}

	charco_pose_ = std::make_pair(r_vec, t_vec);
	return charco_pose_;
}

Eigen::Matrix4d symb::scanner::CalibrationFrame::get_pose_mat()
{
	const std::pair<cv::Vec3d, cv::Vec3d> pose = get_pose();
	cv::Mat r_mat;
	cv::Rodrigues(pose.first, r_mat);
	Eigen::Matrix3d r_mat_eigen;
	cv::cv2eigen(r_mat, r_mat_eigen);
	Eigen::Matrix4d extrinsics = Eigen::Matrix4d::Identity();
	extrinsics.block<3, 3>(0, 0) = r_mat_eigen;

	const Eigen::Vector3d t_vec(pose.second[0], pose.second[1], pose.second[2]);
	extrinsics.block<3, 1>(0, 3) = t_vec;
	extrinsics = extrinsics.inverse();
	return extrinsics;
}

cv::Mat symb::scanner::CalibrationFrame::get_calibrated_image()
{
	if (has_image)
	{
		return drawn_image;
	}

	cv::Mat canvas = get_image(rs2_stream::RS2_STREAM_COLOR);
	const std::pair<std::vector<int>, std::vector<std::vector<cv::Point2f>>> markers = get_markers();
	const std::pair<std::vector<int>, std::vector<cv::Point2f>> charuco = get_charuco();
	const std::pair<std::vector<cv::Vec3d>, std::vector<cv::Vec3d>> single_pose = get_single_pose();
	const std::pair<cv::Vec3d, cv::Vec3d> pose = get_pose();
	const cv::Vec3d r_vec = pose.first;
	const cv::Vec3d t_vec = pose.second;

	// show each marker pose
	cv::aruco::drawDetectedMarkers(canvas, markers.second, markers.first);

	const std::vector<cv::Vec3d> r_vectors = single_pose.first;
	const std::vector<cv::Vec3d> t_vectors = single_pose.second;

	for (int i = 0; i < static_cast<int>(markers.first.size()); i++)
	{
		cv::aruco::drawAxis(canvas, get_intrinsics(), get_dist_coeffs(), r_vectors[i], t_vectors[i], 0.1f);
	}

	// draw Charuco corners & Charuco IDs (マーカーの角とIDを描画)
	cv::aruco::drawDetectedCornersCharuco(canvas, charuco.second, charuco.first, cv::Scalar(255, 0, 0));

	// カメラ座標系の描画(デバッグ用)
	// cv::aruco::drawAxis(canvas, camera_matrix, dist_coeffs, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 1), 0.1f);

	// ボードの原点を描画
	cv::aruco::drawAxis(canvas, get_intrinsics(), get_dist_coeffs(), r_vec, t_vec, 0.1f);

	drawn_image = canvas.clone();

	return drawn_image;
}

cv::Mat symb::scanner::CalibrationFrame::get_aligned_depth_image()
{
	if (has_depth)
	{
		return depth_image;
	}
	depth_image = get_image(rs2_stream::RS2_STREAM_DEPTH, false, true);
	return depth_image;
}

// bi - linear interpolation
// c.f https: // algorithm.joho.info/image-processing/bi-linear-interpolation/
static double interpolate(const cv::Mat &image, const float x, const float y)
{
	const int i = static_cast<int>(y);
	const int j = static_cast<int>(x);
	const int width = image.cols;
	const int height = image.rows;

	if (j >= width - 1 || i >= height - 1)
	{
		return image.at<uint16_t>(MAX(0, MIN(x, width - 1)), MAX(0, MIN(y, height - 1)));
	}

	const float dy = y - static_cast<float>(i);
	const float dx = x - static_cast<float>(j);

	uint16_t v00 = image.at<uint16_t>(i, j);
	uint16_t v10 = image.at<uint16_t>(i + 1, j);
	uint16_t v01 = image.at<uint16_t>(i, j + 1);
	uint16_t v11 = image.at<uint16_t>(i + 1, j + 1);

	std::vector<uint16_t> valid_values;
	for (const uint16_t &v : std::vector<uint16_t>{v00, v10, v01, v11})
	{
		if (v > 0)
		{
			valid_values.emplace_back(v);
		}
	}

	if (valid_values.empty())
	{
		return 0.0;
	}

	unsigned int sum = 0;
	for (const uint16_t &v : valid_values)
	{
		sum += v;
	}
	const double mean_value = sum / static_cast<double>(valid_values.size());
	if (v00 <= 0)
	{
		v00 = mean_value;
	}
	if (v10 <= 0)
	{
		v10 = mean_value;
	}
	if (v01 <= 0)
	{
		v01 = mean_value;
	}
	if (v11 <= 0)
	{
		v11 = mean_value;
	}

	return (1 - dx) * (1 - dy) * static_cast<double>(v00) + dx * (1 - dy) * static_cast<double>(v01) + (1 - dx) * dy * static_cast<double>(v10) + dx * dy * static_cast<double>(v11);
}

static std::tuple<double, std::vector<Eigen::Vector3d>> remove_depth_noise_by_plane_hypothesis(const std::vector<Eigen::Vector3d> &unfiltered)
{
	Eigen::Vector4d plane_coeff;
	std::vector<size_t> inliers;
	std::tie(plane_coeff, inliers) = SegmentPlane(0.003, 5, 200, unfiltered);

	const size_t N = unfiltered.size();
	const Eigen::MatrixXd unfiltered_mat = symb::scanner::utility::convert_pointcloud_to_mat(unfiltered);
	Eigen::MatrixXd plane_calc = unfiltered_mat;
	assert(plane_calc.rows() == N && plane_calc.cols() == 3);
	plane_calc.conservativeResize(N, 4);
	plane_calc.col(3).setOnes(); // N x 4
	double sum = 0.0;
	for (size_t i = 0; i < N; i++)
	{
		const double diff = plane_coeff.dot(plane_calc.row(i));
		sum += abs(diff);
	}
	const double plane_error = (sum / static_cast<double>(N)) * 1000.0;

	const Eigen::Vector3d xy_plane_coeff(plane_coeff(0), plane_coeff(1), plane_coeff(3));
	Eigen::MatrixXd xy_homo = unfiltered_mat;
	xy_homo.col(2).setOnes();

	assert(xy_homo.rows() == N && xy_homo.cols() == 3);
	const Eigen::VectorXd zs_on_plane = -1 * (xy_homo * xy_plane_coeff) / plane_coeff(2);
	assert(zs_on_plane.rows() == N && zs_on_plane.cols() == 1);
	std::vector<Eigen::Vector3d> filtered = unfiltered;
	for (size_t i = 0; i < N; i++)
	{
		filtered[i](2) = zs_on_plane(i);
	}
	return std::make_tuple(plane_error, filtered);
}

static Eigen::Vector3d charuco_get_3d(const cv::Mat &depth_data, const float depth_scale, const float u, const float v, const cv::Mat &intrinsics)
{
	const double d = interpolate(depth_data, u, v);
	const double z = d * depth_scale;

	if (z <= DBL_EPSILON || 3.0 <= z)
	{
		return Eigen::Vector3d::Zero();
	}

	const double fx = static_cast<double>(intrinsics.at<float>(0, 0));
	const double fy = static_cast<double>(intrinsics.at<float>(1, 1));
	const double ppx = static_cast<double>(intrinsics.at<float>(0, 2));
	const double ppy = static_cast<double>(intrinsics.at<float>(1, 2));

	assert(abs(fx) > DBL_EPSILON && abs(fy) > DBL_EPSILON);

	const double x = (u - ppx) * z / fx;
	const double y = (v - ppy) * z / fy;

	return {x, y, z};
}

std::tuple<double, std::pair<std::vector<int>, open3d::geometry::PointCloud>> symb::scanner::CalibrationFrame::get_corner_pointcloud()
{
	if (has_corner_pointcloud)
	{
		return corner_pointcloud_;
	}

	std::vector<int> aligned_ids;
	open3d::geometry::PointCloud aligned_pointcloud;

	const cv::Mat cv_intrinsics = get_intrinsics();
	const std::pair<std::vector<int>, std::vector<cv::Point2f>> charuco = get_charuco();
	const std::vector<int> charuco_ids = charuco.first;
	const std::vector<cv::Point2f> charuco_corners = charuco.second;

	assert(charuco_ids.size() == charuco_corners.size());
	const size_t charuco_count = charuco_ids.size();

	const cv::Mat depth_mat = get_aligned_depth_image();
	const float depth_scale = rs2::sensor_from_frame(raw().get_depth_frame())->get_option(rs2_option::RS2_OPTION_DEPTH_UNITS);

	for (size_t i = 0; i < charuco_count; i++)
	{
		const float u = charuco_corners[i].x;
		const float v = charuco_corners[i].y;
		const Eigen::Vector3d xyz = charuco_get_3d(depth_mat, depth_scale, u, v, cv_intrinsics);
		if (xyz != Eigen::Vector3d::Zero())
		{
			aligned_ids.emplace_back(charuco_ids[i]);
			aligned_pointcloud.points_.emplace_back(xyz);
			// SPDLOG_DEBUG("id{} u{} v{}", charuco_ids[i], u, v);
		}
	}

	const std::tuple<double, std::vector<Eigen::Vector3d>> de_noised_points = remove_depth_noise_by_plane_hypothesis(aligned_pointcloud.points_);
	const double error = std::get<0>(de_noised_points);
	aligned_pointcloud.points_ = std::get<1>(de_noised_points);
	corner_pointcloud_ = std::make_tuple(error, std::make_pair(aligned_ids, aligned_pointcloud));
	return corner_pointcloud_;
}

rs2::video_stream_profile symb::scanner::Frame::get_calibration_profile()
{
	if (has_intrinsics)
	{
		return calibration_profile_.as<rs2::video_stream_profile>();
	}

	retrieve_intrinsics();

	return calibration_profile_.as<rs2::video_stream_profile>();
}

cv::Mat symb::scanner::Frame::get_intrinsics()
{
	if (has_intrinsics)
	{
		return cv_intrinsics_;
	}

	retrieve_intrinsics();

	return cv_intrinsics_;
}

cv::Mat symb::scanner::Frame::get_dist_coeffs()
{
	if (has_intrinsics)
	{
		return cv_dist_coeffs_;
	}

	retrieve_intrinsics();

	return cv_dist_coeffs_;
}

void symb::scanner::Frame::retrieve_intrinsics()
{
	const rs2::video_frame frame = raw().get_color_frame();
	if (!frame)
	{
		throw error::InvalidParameterError(fmt::format("Frame has no Color Frame ({}:{})", FILENAME, __LINE__));
	}

	calibration_profile_ = frame.get_profile();

	rs2_intrinsics rs2_intrinsics = calibration_profile_.as<rs2::video_stream_profile>().get_intrinsics();
	try
	{
		cv_intrinsics_ = (cv::Mat_<float>(3, 3) << rs2_intrinsics.fx, 0, rs2_intrinsics.ppx, 0, rs2_intrinsics.fy, rs2_intrinsics.ppy, 0, 0, 1);
		cv_dist_coeffs_ = (cv::Mat_<float>(5, 1) << rs2_intrinsics.coeffs[0], rs2_intrinsics.coeffs[1], rs2_intrinsics.coeffs[2], rs2_intrinsics.coeffs[3], rs2_intrinsics.coeffs[4]);
	}
	catch (const std::exception &e)
	{
		throw error::InvalidParameterError(fmt::format("Frame Unable to retrieve intrinsics [{}] ({}:{})", e.what(), FILENAME, __LINE__));
	}

	has_intrinsics = true;
}
