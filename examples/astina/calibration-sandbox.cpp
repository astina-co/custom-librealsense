#include<filesystem>

#include <Open3D/Geometry/Octree.h>
#include <Open3D/Open3D.h>

#include <opencv2/opencv.hpp>

#include "include/stream_config.h"
#include <corecrt_math_defines.h>
#include <fmt/format.h>
#include <librealsense2/rs_advanced_mode.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/types_c.h>
#include <spdlog/spdlog.h>

int main(void)
{
	// キャリブデータ
	const double A = 1.018859; // 短辺
	const double B = 1.187027; // 長辺
	const double C = -0.002; // 左デプスカメラの中心からの距離
	const double BASELINE = 0.055; // 左右カメラの距離
	const double D_OFF = 0.0005;
	const double L = sqrt(pow(A / 2, 2) + pow(B / 2, 2));
	const double phi = atan2(A, B);

	// カメラの設置角度
	const double deg = 40.6;
	const double deg_rad = deg / 180.0 * M_PI;

	// 柱の角度
	std::map<std::string, double> degs;
	degs["A"] = -(180 - deg);
	degs["B"] = -deg;
	degs["C"] = -(360 - deg);
	degs["D"] = -(180 + deg);

	// 高さ方向
	std::map<std::string, double> heights;
	heights["1"] = 0.200;
	heights["2"] = heights["1"] + 0.447;
	heights["3"] = heights["2"] + 0.388;
	heights["4"] = heights["3"] + 0.387;
	heights["5"] = heights["4"] + 0.448;

	Eigen::Matrix4d slide;
	slide << 1, 0, 0, C, 0, 1, 0, 0, 0, 0, 1, L + D_OFF, 0, 0, 0, 1;

	Eigen::Matrix4d z180;
	z180 << -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;

	std::map<std::string, Eigen::Matrix4d> results;

	// 柱ごとにループ
	for (auto &d : degs)
	{
		const std::string tower = d.first; // ID
		const double D = d.second; // 柱の角度

		const double theta = D / 180.0 * M_PI;
		const double sin_theta = sin(theta);
		const double cos_theta = cos(theta);

		Eigen::Matrix4d y406;
		y406 << cos_theta, 0, sin_theta, 0, 0, 1, 0, 0, -sin_theta, 0, cos_theta, 0, 0, 0, 0, 1;

		// 高さごとにループ
		for (auto &h : heights)
		{
			const std::string level = h.first;
			const double H = h.second;
			const std::string id = tower + level;

			const Eigen::Vector4d point(0, 0, 0, 1);

			// キャリブデータの行列
			Eigen::Matrix4d yH;
			yH << 1, 0, 0, 0, 0, 1, 0, H, 0, 0, 1, 0, 0, 0, 0, 1;

			// キャリブデータの逆行列？？
			Eigen::Matrix4d camera_to_board = (slide * yH * z180 * y406).inverse();

			results[id] = camera_to_board;

			Eigen::Vector4d test = camera_to_board * point;

			// SPDLOG_INFO(",{}, {: .4f}, {: .4f}, {: .4f}", id, point.x(), point.y(), point.z());
			SPDLOG_INFO(",{}, {: .4f}, {: .4f}, {: .4f}", id, test.x(), test.y(), test.z());
		}
	}

	std::experimental::filesystem::path parent_path;
	parent_path = "C:/symb/calibration/generated";
	std::experimental::filesystem::create_directories(parent_path);

	const std::string depth_scale_index = "depth_scale"; //!< キャリブレーションデータ保存の際のインデックス(撮影スケール)
	const std::string intrinsics_index = "cv_intrinsics"; //!< キャリブレーションデータ保存の際のインデックス(内部パラメータ)
	const std::string dist_coeffs_index = "cv_dist_coeffs"; //!< キャリブレーションデータ保存の際のインデックス(歪み)
	const std::string extrinsics_index = "cv_extrinsics"; //!< キャリブレーションデータ保存の際のインデックス(外部パラメータ)
	const std::string camera_coordinate_index = "cv_camera_coordinate"; //!< キャリブレーションデータ保存の際のインデックス(カメラの座標)

	std::map<std::string, std::string> serials;
	serials["A1"] = "917622061166";
	serials["A2"] = "917622060449";
	serials["A3"] = "917622061065";
	serials["A4"] = "917622060321";
	serials["A5"] = "917622060032";
	serials["B1"] = "917622061181";
	serials["B2"] = "917622060915";
	serials["B3"] = "904412060857";
	serials["B4"] = "917622060328";
	serials["B5"] = "917622060903";
	serials["C1"] = "917622060115";
	serials["C2"] = "917622061270";
	serials["C3"] = "917622061069";
	serials["C4"] = "917622060114";
	serials["C5"] = "917622060962";
	serials["D1"] = "902112061076";
	serials["D2"] = "917622060010";
	serials["D3"] = "917622060486";
	serials["D4"] = "917622061180";
	serials["D5"] = "917622060784";

	for (auto &r : results)
	{
		const std::string id = r.first;
		const Eigen::Matrix4d camera_to_board = r.second;
		const std::experimental::filesystem::path xml_path = parent_path / fmt::format("{}_{}.xml", id, serials[id]);

		double _depth_scale = 0;
		cv::Mat cv_intrinsics = (cv::Mat_<float>(3, 3) << 0, 0, 0, 0, 0, 0, 0, 0, 1); //!< 内部パラメータ
		cv::Mat cv_dist_coeffs = (cv::Mat_<float>(5, 1) << 0, 0, 0, 0, 0);

		cv::Mat cv_extrinsics;
		cv::eigen2cv(camera_to_board, cv_extrinsics);

		const Eigen::Vector4d camera_coordinate = camera_to_board * Eigen::Vector4d(0, 0, 0, 1);
		cv::Vec3d _cv_camera_coordinate = cv::Vec3d(camera_coordinate(0), camera_coordinate(1), camera_coordinate(2));

		/*cv::FileStorage fs(xml_path.string(), cv::FileStorage::WRITE);
		fs << depth_scale_index << _depth_scale;
		fs << intrinsics_index << cv_intrinsics;
		fs << dist_coeffs_index << cv_dist_coeffs;
		fs << extrinsics_index << cv_extrinsics;
		fs << camera_coordinate_index << _cv_camera_coordinate;
		fs.release();*/
	}

	// parser for analysis
	parent_path = "C:/symb/calibration/master";

	std::cout << std::endl;
	for (auto &r : serials)
	{
		const std::string id = r.first;
		const std::string serial = r.second;
		const std::experimental::filesystem::path xml_path = parent_path / fmt::format("{}_{}.xml", id, serial);

		cv::FileStorage fs(xml_path.string(), cv::FileStorage::READ);
		cv::Mat cv_extrinsics;

		fs[extrinsics_index] >> cv_extrinsics;

		fs.release();

		Eigen::Matrix4d extrinsics;
		cv::cv2eigen(cv_extrinsics, extrinsics);

		std::cout << id << ", ";
		for (int i = 0; i < 16; i++)
		{
			std::cout << extrinsics(i) << ", ";
		}

		std::cout << std::endl;
	}
	std::cout << std::endl;

	/*
	rs2::context ctx;
	rs2::device_list devices = ctx.query_devices();
	if (devices.size() <= 0)
	{
		return EXIT_FAILURE;
	}
	rs2::device dev = devices.front();
	std::vector<rs2::sensor> sensors = dev.query_sensors();
	int c = 0;
	cv::Mat infrared;
	cv::Mat depth;
	for (auto&& sensor: sensors)
	{
		for (auto &&profile: sensor.get_stream_profiles())
		{
			rs2::video_stream_profile vprofile = profile.as<rs2::video_stream_profile>();
			if (vprofile.stream_type() == rs2_stream::RS2_STREAM_INFRARED && vprofile.format() == RS2_FORMAT_Y8 && vprofile.fps() == 6 && vprofile.width() == 1280 && vprofile.height() == 720 &&
	vprofile.stream_index() == 2)
			{
				rs2_intrinsics intrinsics = vprofile.get_intrinsics();
				infrared = (cv::Mat_<float>(3, 3) << intrinsics.fx, 0, intrinsics.ppx, 0, intrinsics.fy, intrinsics.ppy, 0, 0, 1);
				c++;
			}
			if (vprofile.stream_type() == rs2_stream::RS2_STREAM_DEPTH && vprofile.format() == RS2_FORMAT_Z16 && vprofile.fps() == 6 && vprofile.width() == 1280 && vprofile.height() == 720)
			{
				rs2_intrinsics intrinsics = vprofile.get_intrinsics();
				depth = (cv::Mat_<float>(3, 3) << intrinsics.fx, 0, intrinsics.ppx, 0, intrinsics.fy, intrinsics.ppy, 0, 0, 1);
				c++;
			}
		}
		if (c == 2)
			break;
	}
	if (c != 2)
	{
		SPDLOG_INFO("BAD SETTINGS");
		return EXIT_FAILURE;
	}
	std::cout << "INFRARED" << std::endl;
	std::cout << infrared << std::endl;
	

	std::cout << "DEPTH" << std::endl;
	std::cout << depth << std::endl;
	*/

	static const int num_of_sides = 2; //!< キャリブレーションボードの面数

	cv::Ptr<cv::aruco::Dictionary> dictionaries[num_of_sides]; //!< ChAruCoマーカー辞書
	cv::Ptr<cv::aruco::CharucoBoard> boards[num_of_sides]; //!< ChAruCoボード
	dictionaries[0] = cv::aruco::Dictionary::create(8 * 35, 6, 0);
	boards[0] = cv::aruco::CharucoBoard::create(8, 35, 0.05f, 0.03f, dictionaries[0]);
	dictionaries[1] = cv::aruco::Dictionary::create(8 * 35, 6, 1);
	boards[1] = cv::aruco::CharucoBoard::create(8, 35, 0.05f, 0.03f, dictionaries[1]);

	parent_path = "C:/symb/calibration/enhanced_calibration";
	std::experimental::filesystem::create_directories(parent_path);

	bool stop = false;

	std::map<std::string, std::string> caps;
	// caps["B3"] = "904412060857";
	caps["B4"] = "917622060328";

	/*for (const auto &s : caps)
	{
		const std::string id = s.first;
		const std::string serial = s.second;
		const char idp = id[0];
		const std::experimental::filesystem::path xml_path = parent_path / fmt::format("{}_{}.xml", id, serial);
		const int idx = (idp == 'A') || (idp == 'D') ? 0 : 1;
		SPDLOG_INFO("Starting {} {}", id, serial);
		int width = 1280;
		int height = 720;
		int wh = width / 2;
		int hh = height / 2;
		int d = 20;
		int fps = 6;
		rs2::config config;
		config.disable_all_streams();
		config.enable_device(serial);
		config.enable_stream(RS2_STREAM_INFRARED, 1, width, height, RS2_FORMAT_Y8, fps);
		// config.enable_stream(RS2_STREAM_INFRARED, 2, width, height, RS2_FORMAT_Y8, fps);
		config.enable_stream(RS2_STREAM_DEPTH, 0, width, height, RS2_FORMAT_Z16, fps);
		// start pipeline
		rs2::pipeline pipeline;
		rs2::pipeline_profile pipeline_profile;
		try
		{
			pipeline_profile = pipeline.start(config);
		}
		catch (rs2::error &e)
		{
			SPDLOG_INFO("rs2::error {}", e.what());
			continue;
		}
		rs2::colorizer colorizer;
		std::vector<cv::Vec3d> inputs;
		cv::Vec3d sum(0, 0, 0);
		std::vector<Eigen::Quaterniond> q_inputs;
		bool q_unset = true;
		Eigen::Quaterniond first_rotation = Eigen::Quaterniond::Identity();
		Eigen::Quaterniond added_rotation = Eigen::Quaterniond::Identity();
		int cnt = 0;
		try
		{
			const auto sensor = rs2::sensor_from_frame(pipeline.wait_for_frames().get_infrared_frame());
			sensor->set_option(rs2_option::RS2_OPTION_EMITTER_ENABLED, 0);
			sensor->set_option(rs2_option::RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0);
			sensor->set_option(rs2_option::RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, 0);
			// sensor->as<rs2::roi_sensor>().set_region_of_interest(rs2::region_of_interest{wh - d, hh - d, wh + d, hh + d});
			sensor->set_option(rs2_option::RS2_OPTION_EXPOSURE, 77000);
		}
		catch (rs2::error &e)
		{
			SPDLOG_ERROR("Error: {}", e.what());
			pipeline.stop();
			return EXIT_FAILURE;
		}
		while (!stop && cnt < 1005) // Application still alive?
		{
			cv::Ptr<cv::aruco::Dictionary> dictionary = dictionaries[idx]; // A, D = 0 // B, C = 1
			cv::Ptr<cv::aruco::CharucoBoard> board = boards[idx];
			// wait for frames and get frameset
			rs2::frameset frameset = pipeline.wait_for_frames();
			// get single infrared frame from frameset
			// rs2::video_frame ir_frame = frameset.get_infrared_frame();
			// get left and right infrared frames from frameset
			rs2::video_frame ir_frame_left = frameset.get_infrared_frame(1);
			// rs2::video_frame ir_frame_right = frameset.get_infrared_frame(2);
			rs2::depth_frame depth_frame = frameset.get_depth_frame();
			rs2::video_frame colorized_frame = colorizer.colorize(depth_frame);
			cnt++;
			if (cnt < 5)
				continue;
			rs2_intrinsics intrinsics = ir_frame_left.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
			cv::Mat _cv_intrinsics = (cv::Mat_<float>(3, 3) << intrinsics.fx, 0, intrinsics.ppx, 0, intrinsics.fy, intrinsics.ppy, 0, 0, 1);
			cv::Mat _cv_dist_coeffs = (cv::Mat_<float>(5, 1) << intrinsics.coeffs[0], intrinsics.coeffs[1], intrinsics.coeffs[2], intrinsics.coeffs[3], intrinsics.coeffs[4]);
			cv::Mat _drawn_canvas;
			cv::Mat _clean_canvas = cv::Mat(cv::Size(width, height), CV_8UC1, (void *)ir_frame_left.get_data());
			cv::cvtColor(_clean_canvas, _clean_canvas, CV_GRAY2BGR);
			_clean_canvas.copyTo(_drawn_canvas);
			// cv::Mat dMat_right = cv::Mat(cv::Size(width, height), CV_8UC1, (void *)ir_frame_right.get_data());
			cv::Mat dMat_depth = cv::Mat(cv::Size(width, height), CV_8UC3, (void *)colorized_frame.get_data());
			std::vector<int> ids;
			std::vector<std::vector<cv::Point2f>> corners;
			std::vector<std::vector<cv::Point2f>> rejected;
			std::shared_ptr<cv::aruco::DetectorParameters> dp_parameters = cv::aruco::DetectorParameters::create();
			dp_parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
			cv::aruco::detectMarkers(_clean_canvas, dictionary, corners, ids, dp_parameters, rejected, _cv_intrinsics, _cv_dist_coeffs);
			if (ids.empty() || corners.empty())
			{
				SPDLOG_ERROR("Calibration unable to detect any markers");
				continue;
			}
			// show each marker pose
			cv::aruco::drawDetectedMarkers(_drawn_canvas, corners, ids);
			std::vector<cv::Vec3d> r_vectors;
			std::vector<cv::Vec3d> t_vectors;
			cv::aruco::estimatePoseSingleMarkers(corners, 0.05f, _cv_intrinsics, _cv_dist_coeffs, r_vectors, t_vectors);
			for (int i = 0; i < static_cast<int>(ids.size()); i++)
			{
				cv::aruco::drawAxis(_drawn_canvas, _cv_intrinsics, _cv_dist_coeffs, r_vectors[i], t_vectors[i], 0.1f);
			}
			std::vector<cv::Point2f> charuco_corners;
			std::vector<int> charuco_ids;
			// determine Charuco corners (マーカーの角の決定)
			interpolateCornersCharuco(corners, ids, _clean_canvas, board, charuco_corners, charuco_ids, _cv_intrinsics, _cv_dist_coeffs);
			if (charuco_ids.empty())
			{
				SPDLOG_ERROR(fmt::format("Calibration unable to determine charuco corners. Found {}.", ids.size()));
				continue;
			}
			// draw Charuco corners & Charuco IDs (マーカーの角とIDを描画)n
			cv::aruco::drawDetectedCornersCharuco(_drawn_canvas, charuco_corners, charuco_ids, cv::Scalar(255, 0, 0));
			cv::Vec3d r_vec, t_vec;
			const auto valid = estimatePoseCharucoBoard(charuco_corners, charuco_ids, board, _cv_intrinsics, _cv_dist_coeffs, r_vec, t_vec);
			// 得られたマーカー情報を基にボードの原点を推定
			if (!valid)
			{
				SPDLOG_ERROR(fmt::format("Calibration unable to estimate board corner. Found {}.", charuco_ids.size()));
				continue;
			}
			// カメラ座標系の描画(デバッグ用)
			// cv::aruco::drawAxis(canvas, camera_matrix, dist_coeffs, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 1), 0.1f);
			// ボードの原点を描画
			cv::aruco::drawAxis(_drawn_canvas, _cv_intrinsics, _cv_dist_coeffs,
								r_vec, // ボードと対応する回転ベクトル（回転行列ではない点に注意する）
								t_vec, // ボードと対応する並進ベクトル
								0.1f); // ボードの原点に表示する軸の長さを指定
			cv::imshow("clean", _clean_canvas);
			cv::imshow("drawn", _drawn_canvas);
			inputs.emplace_back(t_vec);
			sum += t_vec;
			const int N = static_cast<int>(inputs.size());
			cv::Vec3d t_ave = sum / N;
			double dev = 0;
			for (auto &point : inputs)
			{
				cv::Vec3d p = point - t_ave;
				double pp = p[0] * p[0] + p[1] * p[1] + p[2] * p[2];
				dev += pp;
			}
			dev /= N;
			dev = sqrt(dev);
			cv::Mat r_mat_;
			Eigen::Matrix3d r_mat;
			Rodrigues(r_vec, r_mat_);
			cv::cv2eigen(r_mat_, r_mat);
			Eigen::Quaterniond q(r_mat);
			if (q_unset)
			{
				first_rotation = q;
				q_unset = false;
			}
			else
			{
				if (first_rotation.dot(q) < 0.0f)
				{
					q = Eigen::Quaterniond(-q.w(), -q.x(), -q.y(), -q.z());
				}
			}
			float add_det = 1.0f / static_cast<float>(N);
			added_rotation = Eigen::Quaterniond(added_rotation.w() + q.w(), added_rotation.x() + q.x(), added_rotation.y() + q.y(), added_rotation.z() + q.z());
			float w = add_det * added_rotation.w();
			float x = add_det * added_rotation.x();
			float y = add_det * added_rotation.y();
			float z = add_det * added_rotation.z();
			float D = 1.0f / (w * w + x * x + y * y + z * z);
			w *= D;
			x *= D;
			y *= D;
			z *= D;
			Eigen::Quaterniond q_average = Eigen::Quaterniond(w, x, y, z);
			Eigen::Matrix3d r_ave_(q_average);
			cv::Mat r_ave;
			cv::eigen2cv(r_ave_, r_ave);
			const double center_dist = depth_frame.get_distance(width / 2, height / 2);
			const std::string out =
				fmt::format("{}, {:.10f}, {: .10f}, {: .10f}, {: .10f}, {: .10f},  {: .10f}, {: .10f}, {: .10f}, {: .10f},  {: .10f}, {: .10f}, {: .10f}, {: .10f},  {: .10f}, {: .10f}, {: .10f}", id,
							center_dist, dev, t_ave[0], t_ave[1], t_ave[2], t_vec[0], t_vec[1], t_vec[2], q.w(), q.x(), q.y(), q.z(), q_average.w(), q_average.x(), q_average.y(), q_average.z());
			std::cout << out << std::endl;
			Eigen::Matrix4d board_to_camera = Eigen::Matrix4d::Identity();
			for (auto i = 0; i < 3; i++)
			{
				for (auto j = 0; j < 3; j++)
				{
					board_to_camera(i, j) = r_ave.at<double>(i, j);
				}
				board_to_camera(i, 3) = t_ave[i];
			}
			Eigen::Matrix4d camera_to_board = board_to_camera.inverse();
			// cv::imshow("img_r", dMat_right);
			cv::imshow("img_d", dMat_depth);
			char c = static_cast<char>(cv::waitKey(1));
			if (c == 's')
			{
				auto sensor = rs2::sensor_from_frame(ir_frame_left);
				double _depth_scale = sensor->get_option(rs2_option::RS2_OPTION_DEPTH_UNITS);
				if (idp == 'B' || idp == 'C')
				{
					Eigen::Vector3d flip_position(static_cast<double>(0.4), 0, 0);
					const auto cos_p = cos(M_PI);
					const auto sin_p = sin(M_PI);
					Eigen::Matrix4d un_flip;
					un_flip << cos_p, 0, sin_p, flip_position(0), 0, 1, 0, flip_position(1), -sin_p, 0, cos_p, flip_position(2), 0, 0, 0, 1;
					camera_to_board = un_flip.inverse() * camera_to_board;
					Eigen::Matrix4d offset_thickness;
					offset_thickness << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, static_cast<double>(0.03f), 0, 0, 0, 1;
					camera_to_board = offset_thickness.inverse() * camera_to_board;
				}
				Eigen::Matrix4d offset_x;
				offset_x << 1, 0, 0, static_cast<double>(0.2), 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
				camera_to_board = offset_x.inverse() * camera_to_board;
				cv::Mat cv_extrinsics;
				cv::eigen2cv(camera_to_board, cv_extrinsics);
				const Eigen::Vector4d camera_coordinate = camera_to_board * Eigen::Vector4d(0, 0, 0, 1);
				cv::Vec3d _cv_camera_coordinate = cv::Vec3d(camera_coordinate(0), camera_coordinate(1), camera_coordinate(2));
				cv::FileStorage fs(xml_path.string(), cv::FileStorage::WRITE);
				fs << depth_scale_index << _depth_scale;
				fs << intrinsics_index << _cv_intrinsics;
				fs << dist_coeffs_index << _cv_dist_coeffs;
				fs << extrinsics_index << cv_extrinsics;
				fs << camera_coordinate_index << _cv_camera_coordinate;
				fs.release();
			}
			else if (c == 'q')
			{
				break;
			}
			else if (c == 'c')
			{
				stop = true;
				break;
			}
		}
		pipeline.stop();
		if (stop)
		{
			break;
		}
	}*/

	cv::destroyAllWindows();

	std::ifstream j("C:/Workspace/astina-body-scanner/build/sandbox/preset.json");
	const std::string json((std::istreambuf_iterator<char>(j)), std::istreambuf_iterator<char>());
	j.close();

	std::map<std::string, rs2::device> devices;

	rs2::context ctx;
	for (const auto &device : ctx.query_devices())
	{
		devices[device.get_info(rs2_camera_info::RS2_CAMERA_INFO_SERIAL_NUMBER)] = device;

		for (const auto &sensor : device.query_sensors())
		{
			if (sensor.supports(rs2_option::RS2_OPTION_LASER_POWER))
			{
				rs2::option_range range = sensor.get_option_range(rs2_option::RS2_OPTION_LASER_POWER);
				SPDLOG_INFO("MIN: {} MAX: {}", range.min, range.max);
				sensor.set_option(rs2_option::RS2_OPTION_LASER_POWER, range.min);
			}
			if (sensor.supports(rs2_option::RS2_OPTION_EMITTER_ENABLED))
			{
				sensor.set_option(rs2_option::RS2_OPTION_EMITTER_ENABLED, 0);
			}
		}
	}

	for (const auto &s : caps)
	{
		const std::string id = s.first;
		const std::string serial = s.second;
		const char idp = id[0];
		const std::experimental::filesystem::path xml_path = parent_path / fmt::format("{}_{}.xml", id, serial);

		const int idx = (idp == 'A') || (idp == 'D') ? 0 : 1;

		SPDLOG_INFO("Starting {} {}", id, serial);

		int width = 1280;
		int height = 720;
		int wh = width / 2;
		int hh = height / 2;
		int d = 20;
		int fps = 30;
		rs2::config config;
		config.disable_all_streams();
		config.enable_device(serial);
		config.enable_stream(RS2_STREAM_INFRARED, 1, width, height, RS2_FORMAT_Y8, fps);
		// config.enable_stream(RS2_STREAM_INFRARED, 2, width, height, RS2_FORMAT_Y8, fps);
		config.enable_stream(RS2_STREAM_DEPTH, 0, width, height, RS2_FORMAT_Z16, fps);

		// start pipeline
		rs2::pipeline pipeline;
		rs2::pipeline_profile pipeline_profile;

		try
		{
			pipeline_profile = pipeline.start(config);
		}
		catch (rs2::error &e)
		{
			SPDLOG_INFO("rs2::error {}", e.what());
			continue;
		}

		rs2::colorizer colorizer;

		std::vector<cv::Vec3d> inputs;
		cv::Vec3d sum(0, 0, 0);

		std::vector<Eigen::Quaterniond> q_inputs;

		int cnt = 0;

		try
		{
			rs2::device device = pipeline_profile.get_device();
			if (!device.is<rs400::advanced_mode>())
			{
				device.as<rs400::advanced_mode>().toggle_advanced_mode(true);
			}

			device.as<rs400::advanced_mode>().load_json(json);
			const auto sensor = rs2::sensor_from_frame(pipeline.wait_for_frames().get_infrared_frame());
			sensor->set_option(rs2_option::RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);
			sensor->set_option(rs2_option::RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, 0);
			sensor->set_option(rs2_option::RS2_OPTION_LASER_POWER, sensor->get_option_range(rs2_option::RS2_OPTION_LASER_POWER).min);
			sensor->set_option(rs2_option::RS2_OPTION_EMITTER_ENABLED, 0);

			// sensor->as<rs2::roi_sensor>().set_region_of_interest(rs2::region_of_interest{wh - d, hh - d, wh + d, hh + d});
			// sensor->set_option(rs2_option::RS2_OPTION_EXPOSURE, 77000);
		}
		catch (rs2::error &e)
		{
			SPDLOG_ERROR("Error: {}", e.what());
			pipeline.stop();
			return EXIT_FAILURE;
		}

		const auto depth_to_disparity = rs2::disparity_transform(true); //!<
		const auto disparity_to_depth = rs2::disparity_transform(false); //!<
																		 // rs2::decimation_filter decimation_filter = rs2::decimation_filter(4.0f); //!<
		const auto spatial_filter = rs2::spatial_filter(0.71f, 25, 5.0f, 0); //!<
		const auto temporal_filter = rs2::temporal_filter(0.05f, 96, 4); //!<
		const auto threshold_filter = rs2::threshold_filter(0.16f, 1.4f); //!<

		rs2::frameset rs2_frameset;

		cv::namedWindow("img_d", 1);

		struct pixel
		{
			int x;
			int y;
		};

		const auto CallBackFunc = [](int event, int x, int y, int flags, void *userdata) {
			struct pixel *pixel = static_cast<struct pixel *>(userdata);

			if (event == cv::EVENT_LBUTTONDOWN)
			{
				std::cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
				pixel->x = x;
				pixel->y = y;
			}
		};

		struct pixel pixel
		{
			width / 2, height / 2
		};

		cv::setMouseCallback("img_d", CallBackFunc, static_cast<void *>(&pixel));

		while (!stop) // Application still alive?
		{
			cv::Ptr<cv::aruco::Dictionary> dictionary = dictionaries[idx]; // A, D = 0 // B, C = 1
			cv::Ptr<cv::aruco::CharucoBoard> board = boards[idx];
			// wait for frames and get frameset
			rs2::frameset frameset = pipeline.wait_for_frames();

			frameset = depth_to_disparity.process(rs2::frame(frameset));
			frameset = spatial_filter.process(rs2::frame(frameset));
			frameset = temporal_filter.process(rs2::frame(frameset));
			frameset = disparity_to_depth.process(rs2::frame(frameset));
			frameset = threshold_filter.process(rs2::frame(frameset));

			// get left and right infrared frames from frameset
			rs2::video_frame ir_frame_left = frameset.get_infrared_frame(1);
			// rs2::video_frame ir_frame_right = frameset.get_infrared_frame(2);

			rs2::depth_frame depth_frame = frameset.get_depth_frame();
			rs2::video_frame colorized_frame = colorizer.colorize(depth_frame);

			cnt++;
			if (cnt < 5)
				continue;

			rs2_intrinsics intrinsics = ir_frame_left.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
			cv::Mat _cv_intrinsics = (cv::Mat_<float>(3, 3) << intrinsics.fx, 0, intrinsics.ppx, 0, intrinsics.fy, intrinsics.ppy, 0, 0, 1);
			cv::Mat _cv_dist_coeffs = (cv::Mat_<float>(5, 1) << intrinsics.coeffs[0], intrinsics.coeffs[1], intrinsics.coeffs[2], intrinsics.coeffs[3], intrinsics.coeffs[4]);

			cv::Mat _drawn_canvas;
			cv::Mat _clean_canvas = cv::Mat(cv::Size(width, height), CV_8UC1, (void *)ir_frame_left.get_data());
			cv::cvtColor(_clean_canvas, _clean_canvas, CV_GRAY2BGR);
			_clean_canvas.copyTo(_drawn_canvas);
			// cv::Mat dMat_right = cv::Mat(cv::Size(width, height), CV_8UC1, (void *)ir_frame_right.get_data());
			cv::Mat dMat_depth = cv::Mat(cv::Size(width, height), CV_8UC3, (void *)colorized_frame.get_data());

			const double center_dist = depth_frame.get_distance(pixel.x, pixel.y);

			const std::string out = fmt::format("{}, {:d}, ", id, static_cast<int>(center_dist * 1000));

			std::cout << out << std::endl;

			cv::imshow("img_d", dMat_depth);
			char c = static_cast<char>(cv::waitKey(1));
			if (c == 'q')
			{
				break;
			}
			else if (c == 'c')
			{
				stop = true;
				break;
			}
		}

		cv::destroyAllWindows();
		pipeline.stop();
	}
}