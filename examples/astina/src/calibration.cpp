#include "calibration.h"
#include "exception.h"
#include "defines.h"

symb::scanner::CalibrationData::CalibrationDataPtr::CalibrationDataPtr(const std::experimental::filesystem::path &xml_path) : CalibrationDataPtr()
{
	if (!xml_path.has_extension())
	{
		throw error::InvalidParameterError(fmt::format("Calibration Load Invalid file path ({}:{})", FILENAME, __LINE__));
	}

	if (xml_path.extension() != ".xml")
	{
		throw error::InvalidParameterError(fmt::format("Calibration Load Invalid file extension [xml] ({}:{})", FILENAME, __LINE__));
	}

	if (!std::experimental::filesystem::exists(xml_path))
	{
		throw error::InvalidParameterError(fmt::format("Calibration Load File {} does not exist ({}:{})", xml_path.string(), FILENAME, __LINE__));
	}

	try
	{
		cv::FileStorage fs(xml_path.string(), cv::FileStorage::READ);
		cv::Mat cv_extrinsics;

		fs[depth_scale_index] >> _depth_scale;
		fs[intrinsics_index] >> _cv_intrinsics;
		fs[dist_coeffs_index] >> _cv_dist_coeffs;
		fs[extrinsics_index] >> cv_extrinsics;

		fs.release();

		cv::cv2eigen(_cv_intrinsics, _intrinsics);
		_intrinsics_inv = _intrinsics.inverse();
		cv::cv2eigen(cv_extrinsics, _extrinsics);

		const Eigen::Vector4d camera_coordinate = _extrinsics * Eigen::Vector4d(0, 0, 0, 1);
		_cv_camera_coordinate = cv::Vec3d(camera_coordinate(0), camera_coordinate(1), camera_coordinate(2));
	}
	catch (...)
	{
		throw error::InvalidParameterError(fmt::format("Calibration Load Unable to Read File ({}:{})", FILENAME, __LINE__));
	}
}

symb::scanner::CalibrationData::CalibrationDataPtr::CalibrationDataPtr(const std::experimental::filesystem::path &xml_path, const std::experimental::filesystem::path &png_before,
																	   const std::experimental::filesystem::path &png_after, const std::experimental::filesystem::path &png_depth)
	: CalibrationDataPtr(xml_path)
{
	const std::function<void(const std::experimental::filesystem::path &)> test_path = [](const std::experimental::filesystem::path &path)
	{
		if (!path.has_extension())
		{
			throw error::InvalidParameterError(fmt::format("Calibration Load Invalid file path ({}:{})", FILENAME, __LINE__));
		}

		if (path.extension() != ".png")
		{
			throw error::InvalidParameterError(fmt::format("Calibration Load Invalid file extension [png] ({}:{})", FILENAME, __LINE__));
		}

		if (!std::experimental::filesystem::exists(path))
		{
			throw error::InvalidParameterError(fmt::format("Calibration Load File does not exist ({}:{})", FILENAME, __LINE__));
		}
	};
	
	test_path(png_before);
	test_path(png_after);
	test_path(png_depth);

	const std::function<void(cv::Mat &, const std::experimental::filesystem::path)> read = [](cv::Mat & mat, const std::experimental::filesystem::path path)
	{
		try
		{
			mat = cv::imread(path.string(), cv::IMREAD_UNCHANGED);
		}
		catch (...)
		{
		}
	};

	read(_clean_canvas, png_before);
	read(_drawn_canvas, png_after);
	read(_depth_canvas, png_depth);
	_has_canvas = true;
}

symb::scanner::CalibrationData::CalibrationDataPtr::CalibrationDataPtr(const CalibrationBoard &calibration_board, const Side side, const Frameset &frameset)
{
	try
	{
		const double cos_p = cos(M_PI);
		const double sin_p = sin(M_PI);
		
		const cv::Ptr<cv::aruco::Dictionary> dictionary = calibration_board.get_dictionary(side);
		const cv::Ptr<cv::aruco::CharucoBoard> board = calibration_board.get_board(side);

		std::vector<Frame> frame_vector = frameset.vector();

		int entries = 0;
		Eigen::Vector3d t_sum = Eigen::Vector3d::Zero();
		Eigen::Vector4d q_sum = Eigen::Vector4d::Zero();

		bool first_q_unset = true;
		Eigen::Quaterniond first_q;

		for (const Frame &frame : frame_vector)
		{
			CalibrationFrame calibration_frame(frame, dictionary, board);
			const std::pair<cv::Vec3d, cv::Vec3d> pose = calibration_frame.get_pose();
			cv::Vec3d r_vec = pose.first;
			cv::Vec3d t_vec = pose.second;

			cv::Mat r_mat;

			t_sum += Eigen::Vector3d(t_vec[0], t_vec[1], t_vec[2]);

			// ボードと対応する回転ベクトル r_vec をボードと対応する回転行列 r_mat に変換する
			Rodrigues(r_vec, r_mat);
			Eigen::Matrix3d r_mat_eigen;
			cv::cv2eigen(r_mat, r_mat_eigen);
			Eigen::Quaterniond q(r_mat_eigen);

			if (first_q_unset)
			{
				first_q = q;
				first_q_unset = false;
			}
			else
			{
				if (first_q.dot(q) < 0.0f)
				{
					q = Eigen::Quaterniond(-q.w(), -q.x(), -q.y(), -q.z());
				}
			}

			q_sum += Eigen::Vector4d(q.w(), q.x(), q.y(), q.z());

			entries += 1;
		}

		Eigen::Vector3d t_ave = t_sum / entries;
		Eigen::Vector4d q_ave_v = q_sum / entries;

		Eigen::Quaterniond q_ave(q_ave_v[0], q_ave_v[1], q_ave_v[2], q_ave_v[3]);

		Eigen::Matrix3d r_mat(q_ave);
		Eigen::Matrix4d board_to_camera = Eigen::Matrix4d::Identity();
		board_to_camera.block<3, 3>(0, 0) = r_mat;
		board_to_camera.block<3, 1>(0, 3) = t_ave;
		
		Eigen::Matrix4d camera_to_board = board_to_camera.inverse();

		if (side == Side::Rear)
		{
			const Eigen::Matrix3d flip_rotation(Eigen::AngleAxisd(M_PI, Eigen::Vector3d(0, 1, 0)));
			Eigen::Matrix4d y_rot180 = Eigen::Matrix4d::Identity();
			y_rot180.block<3, 3>(0, 0) = flip_rotation;

			const double checker_board_width = static_cast<double>(calibration_board.get_square_length()) * static_cast<double>(calibration_board.get_square_count_x());
			const Eigen::Vector3d flip_translation = Eigen::Vector3d(checker_board_width, 0, 0);
			Eigen::Matrix4d x_shift = Eigen::Matrix4d::Identity();
			x_shift.block<3, 1>(0, 3) = flip_translation;
			
			Eigen::Matrix4d z_shift = Eigen::Matrix4d::Identity();
			z_shift(2, 3) = static_cast<double>(- calibration_board.get_thickness());

			camera_to_board = z_shift * (x_shift * (y_rot180 * camera_to_board));
		}

		const double center = -static_cast<double>(calibration_board.get_square_count_x()) * (static_cast<double>(calibration_board.get_square_length()) * 0.5f);
		Eigen::Matrix4d center_shift = Eigen::Matrix4d::Identity();
		center_shift(0, 3) = center;
		camera_to_board = center_shift * camera_to_board;

		_extrinsics = camera_to_board;

		const Eigen::Vector4d camera_coordinate = _extrinsics * Eigen::Vector4d(0, 0, 0, 1);
		_cv_camera_coordinate = cv::Vec3d(camera_coordinate(0), camera_coordinate(1), camera_coordinate(2));

		Frame frame = frameset[0];

		_depth_scale = rs2::sensor_from_frame(frame.raw().get_depth_frame())->get_option(rs2_option::RS2_OPTION_DEPTH_UNITS);
		_cv_intrinsics = frame.get_intrinsics();
		cv::cv2eigen(_cv_intrinsics, _intrinsics);
		_intrinsics_inv = _intrinsics.inverse();
		_cv_dist_coeffs = frame.get_dist_coeffs();
		_clean_canvas = frame.get_image(rs2_stream::RS2_STREAM_COLOR);
		CalibrationFrame calibration_frame(frame, dictionary, board);
		_drawn_canvas = calibration_frame.get_calibrated_image();
		_depth_canvas = calibration_frame.get_aligned_depth_image();
		_has_canvas = true;
	}
	catch (error::Error &)
	{
		throw;
	}
	catch (const std::exception &e)
	{
		throw error::UnRecoverableError(fmt::format("{} ({}:{})", e.what(), FILENAME, __LINE__));
	}
}

symb::scanner::CalibrationData::CalibrationDataPtr::CalibrationDataPtr(const CalibrationFrame &frame)
{
	CalibrationFrame calibration_frame = frame;
	_depth_scale = rs2::sensor_from_frame(calibration_frame.raw().get_depth_frame())->get_option(rs2_option::RS2_OPTION_DEPTH_UNITS);
	_cv_intrinsics = calibration_frame.get_intrinsics();
	cv::cv2eigen(_cv_intrinsics, _intrinsics);
	_intrinsics_inv = _intrinsics.inverse();
	_cv_dist_coeffs = calibration_frame.get_dist_coeffs();
	_extrinsics = Eigen::Matrix4d::Identity();
	_cv_camera_coordinate = cv::Vec3d(0, 0, 0);
	_clean_canvas = frame.get_image(rs2_stream::RS2_STREAM_COLOR);
	_drawn_canvas = calibration_frame.get_calibrated_image();
	_depth_canvas = calibration_frame.get_aligned_depth_image();
	_has_canvas = true;
	_is_private = true;
}

bool symb::scanner::CalibrationData::CalibrationDataPtr::is_valid() const
{
	try
	{
		// ReSharper disable once CppExpressionWithoutSideEffects
		get_depth_scale();
		// ReSharper disable once CppExpressionWithoutSideEffects
		get_extrinsics();
		// ReSharper disable once CppExpressionWithoutSideEffects
		get_intrinsics();
		// ReSharper disable once CppExpressionWithoutSideEffects
		get_intrinsics_inv();
		// ReSharper disable once CppExpressionWithoutSideEffects
		get_cv_dist_coeffs();
		// ReSharper disable once CppExpressionWithoutSideEffects
		get_cv_camera_coordinate();
		return true;
	}
	catch (const std::exception &e)
	{
		SPDLOG_WARN(e.what());
		return false;
	}
}

float symb::scanner::CalibrationData::CalibrationDataPtr::get_depth_scale() const
{
	if (_depth_scale == 0.0f && !_is_private)
	{
		throw error::BadStateError("Calibration Info Invalid [depth scale]");
	}

	return _depth_scale;
}

Eigen::Matrix3d symb::scanner::CalibrationData::CalibrationDataPtr::get_intrinsics() const
{
	if (_intrinsics.isIdentity() && !_is_private)
	{
		throw error::BadStateError(fmt::format("Calibration Info Invalid [intrinsics] ({}:{})", FILENAME, __LINE__));
	}

	return _intrinsics;
}
Eigen::Matrix3d symb::scanner::CalibrationData::CalibrationDataPtr::get_intrinsics_inv() const
{
	if (_intrinsics_inv.isIdentity() && !_is_private)
	{
		throw error::BadStateError(fmt::format("Calibration Info Invalid [intrinsics_inv] ({}:{})", FILENAME, __LINE__));
	}

	return _intrinsics_inv;
}
Eigen::Matrix4d symb::scanner::CalibrationData::CalibrationDataPtr::get_extrinsics() const
{
	if (_extrinsics.isIdentity() && !_is_private)
	{
		throw error::BadStateError(fmt::format("Calibration Info Invalid [extrinsics] ({}:{})", FILENAME, __LINE__));
	}

	return _extrinsics;
}

cv::Mat symb::scanner::CalibrationData::CalibrationDataPtr::get_cv_dist_coeffs() const
{
	if (_cv_dist_coeffs.empty() && !_is_private)
	{
		throw error::BadStateError(fmt::format("Calibration Info Invalid [coeffs] ({}:{})", FILENAME, __LINE__));
	}

	return _cv_dist_coeffs;
}

cv::Vec3d symb::scanner::CalibrationData::CalibrationDataPtr::get_cv_camera_coordinate() const
{
	if ((_extrinsics.isIdentity() || cv::countNonZero(_cv_camera_coordinate) == 0) && !_is_private)
	{
		throw error::BadStateError(fmt::format("Calibration Info Invalid [cv_camera_coordinate] ({}:{})", FILENAME, __LINE__));
	}

	return _cv_camera_coordinate;
}

Eigen::Vector3d symb::scanner::CalibrationData::CalibrationDataPtr::get_camera_coordinate() const
{
	const cv::Vec3d cv_camera_coordinate = get_cv_camera_coordinate();
	Eigen::Vector3d camera_coordinate;
	cv::cv2eigen(cv_camera_coordinate, camera_coordinate);
	return camera_coordinate;
}

void symb::scanner::CalibrationData::CalibrationDataPtr::pretransform(const Eigen::Matrix4d &transform)
{
	_extrinsics = transform * _extrinsics;
	const Eigen::Vector4d camera_coordinate_4d = _extrinsics * Eigen::Vector4d(0, 0, 0, 1);
	const Eigen::Vector3d camera_coordinate = camera_coordinate_4d.block<3, 1>(0, 0);
	cv::Mat cv_camera_coordinate;
	cv::eigen2cv(camera_coordinate, cv_camera_coordinate);
	_cv_camera_coordinate = cv_camera_coordinate;
}

void symb::scanner::CalibrationData::CalibrationDataPtr::save_xml(const std::experimental::filesystem::path &xml_path) const
{
	const size_t attempts = 2;
	for (size_t i = 0; i < attempts; i++)
	{
		if (i > 0)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(200));
		}
		try
		{
			if (!utility::mkdir_if_not_exist(xml_path.parent_path()))
			{
				throw error::InvalidParameterError(fmt::format("Unable to access parent directory ({}:{})", xml_path.parent_path().string(), FILENAME, __LINE__));
			}
			break;
		}
		catch (error::InvalidParameterError &)
		{
			if (i == attempts - 1)
			{
				throw;
			}
		}
		catch (const std::exception &e)
		{
			if (i == attempts - 1)
			{
				throw error::InvalidParameterError(fmt::format("{} ({}:{})", e.what(), FILENAME, __LINE__));
			}
		}
	}

	try
	{
		const Eigen::Matrix3d &intrinsics = get_intrinsics();
		const Eigen::Matrix4d &extrinsics = get_extrinsics();

		cv::Mat cv_intrinsics;
		cv::Mat cv_extrinsics;

		cv::eigen2cv(intrinsics, cv_intrinsics);
		cv::eigen2cv(extrinsics, cv_extrinsics);

		const cv::Mat &cv_dist_coeffs = get_cv_dist_coeffs();
		const cv::Vec3d &cv_camera_coordinate = get_cv_camera_coordinate();

		cv::FileStorage fs(xml_path.string(), cv::FileStorage::WRITE);
		fs << depth_scale_index << _depth_scale;
		fs << intrinsics_index << cv_intrinsics;
		fs << dist_coeffs_index << cv_dist_coeffs;
		fs << extrinsics_index << cv_extrinsics;
		fs << camera_coordinate_index << cv_camera_coordinate;
		fs.release();
	}
	catch (const std::exception &e)
	{
		if (exists(xml_path) && !utility::is_dir(xml_path))
		{
			remove(xml_path);
		}
		
		throw error::BadStateError(fmt::format("Save Calibration Data to XML Failed [{}] ({}:{})", e.what(), FILENAME, __LINE__));
	}
}

void symb::scanner::CalibrationData::CalibrationDataPtr::save_images(const std::experimental::filesystem::path &png_before_path, const std::experimental::filesystem::path &png_after_path,
																	 const std::experimental::filesystem::path &png_depth_path) const
{
	const auto &check_path = [](const std::experimental::filesystem::path &path, const size_t attempts) {
		for (size_t i = 0; i < attempts; i++)
		{
			if (i > 0)
			{
				std::this_thread::sleep_for(std::chrono::milliseconds(200));
			}
			try
			{
				if (!utility::mkdir_if_not_exist(path.parent_path()))
				{
					throw error::InvalidParameterError(fmt::format("Unable to access parent directory \"{}\" ({}:{})", path.parent_path().string(), FILENAME, __LINE__));
				}

				if (std::experimental::filesystem::exists(path))
				{
					throw error::InvalidParameterError(fmt::format("Following file already exists \"{}\" [Will not Overwrite] ({}:{})", path.string(), FILENAME, __LINE__));
				}
				return;
			}
			catch (error::InvalidParameterError &)
			{
				if (i == attempts - 1)
				{
					throw;
				}
			}
			catch (const std::exception &e)
			{
				if (i == attempts - 1)
				{
					throw error::InvalidParameterError(fmt::format("{} ({}:{})", e.what(), FILENAME, __LINE__));
				}
			}
		}
	};

	// check_path(png_before_path, 2);
	// check_path(png_after_path, 2);
	check_path(png_depth_path, 2);

	const auto &write_image = [](const class cv::Mat &canvas, const std::experimental::filesystem::path &path) {
		try
		{
			cv::imwrite(path.string(), canvas, {cv::IMWRITE_PNG_COMPRESSION, 0});
		}
		catch (const std::exception &e)
		{
			if (std::experimental::filesystem::exists(path) && !utility::is_dir(path))
			{
				std::experimental::filesystem::remove(path);
			}

			throw error::BadStateError(fmt::format("Unable to save image to {} [{}] ({}:{})", path.string(), e.what(), FILENAME, __LINE__));
		}
	};

	write_image(_clean_canvas, png_before_path);
	write_image(_drawn_canvas, png_after_path);
	write_image(_depth_canvas, png_depth_path);
}

symb::scanner::CalibrationData symb::scanner::CalibrationData::Identity(const CalibrationFrame &frame) { return CalibrationData(std::make_shared<CalibrationDataPtr>(frame)); }
