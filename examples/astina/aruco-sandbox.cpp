#include <librealsense2/rs.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/opencv.hpp>
#include <spdlog/spdlog.h>

int main() try
{
	rs2::context ctx;
	ctx.query_all_sensors().begin()->set_option(RS2_OPTION_EMITTER_ENABLED, 1.0);

	rs2::pipeline pipe;

	rs2::config config;
	const int w = 1280;
	const int h = 720;
	config.enable_stream(RS2_STREAM_COLOR, w, h, RS2_FORMAT_RGB8, 30);
	config.enable_stream(RS2_STREAM_INFRARED, 1, w, h, RS2_FORMAT_Y8, 30);
	// config.enable_stream(RS2_STREAM_DEPTH, 1, w, h, RS2_FORMAT_Y8, 30);
	config.enable_stream(RS2_STREAM_DEPTH, RS2_FORMAT_Z16, 30);

	pipe.start(config);

	auto intrinsics = pipe.get_active_profile().get_stream(RS2_STREAM_INFRARED).as<rs2::video_stream_profile>().get_intrinsics();
	SPDLOG_INFO("fx:{}, fy:{}, w:{}, h:{}, ppx:{}, ppy:{}", intrinsics.fx, intrinsics.fy, intrinsics.width, intrinsics.height, intrinsics.ppx, intrinsics.ppy);
	SPDLOG_INFO("model:{}, c0:{}, c1:{}, c2:{}, c3:{}, c4:{}", static_cast<int>(intrinsics.model), //
				intrinsics.coeffs[0], intrinsics.coeffs[1], intrinsics.coeffs[2], intrinsics.coeffs[3], intrinsics.coeffs[4]);

	cv::Mat cameraMatrix = cv::Mat::eye(cv::Size(3, 3), CV_32FC1);
	cameraMatrix.at<float>(0, 0) = intrinsics.fx;
	cameraMatrix.at<float>(1, 1) = intrinsics.fy;
	cameraMatrix.at<float>(0, 2) = intrinsics.ppx;
	cameraMatrix.at<float>(1, 2) = intrinsics.ppy;
	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
	cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(5, 7, 0.037f, 0.022f, dictionary);

	cv::Mat distCoeffs = cv::Mat::zeros(cv::Size(5, 1), CV_32FC1);

	for (int i = 0; i < 5; i++)
	{
		// TODO check same calibration model
		distCoeffs.at<float>(i, 0) = intrinsics.coeffs[i];
	}

	SPDLOG_INFO("start loop");

	while (true)
	{
		auto frames = pipe.wait_for_frames();
		auto color_frame = frames.get_color_frame();
		auto infrared_frame = frames.get_infrared_frame();
		auto depth_frame = frames.get_depth_frame();

		if (!color_frame || !infrared_frame || !depth_frame)
		{
			SPDLOG_ERROR("drop frame");
			continue;
		}

		cv::Mat color(cv::Size(w, h), CV_8UC3, (void *)color_frame.get_data(), cv::Mat::AUTO_STEP);
		cv::Mat infrared(cv::Size(w, h), CV_8UC1, (void *)infrared_frame.get_data(), cv::Mat::AUTO_STEP);
		cv::Mat depth(cv::Size(w, h), CV_8UC1, (void *)depth_frame.get_data(), cv::Mat::AUTO_STEP);

		cv::cvtColor(color, color, CV_RGB2BGR);
		cv::cvtColor(infrared, infrared, CV_GRAY2BGR);
		cv::cvtColor(depth, depth, CV_GRAY2BGR);

		cv::Mat target;
		cv::Mat canvas;

		// color.copyTo(target);
		// color.copyTo(canvas);

		infrared.copyTo(target);
		infrared.copyTo(canvas);

		std::vector<int> ids;
		std::vector<std::vector<cv::Point2f>> corners;
		cv::aruco::detectMarkers(target, dictionary, corners, ids);
		// if at least one marker detected
		// SPDLOG_INFO("marker: {}", ids.size());

		if (ids.size() > 0)
		{
			// draw each markers
			cv::aruco::drawDetectedMarkers(canvas, corners, ids);
			// draw axis for each marker
			std::vector<cv::Vec3d> rvecs, tvecs;
			cv::aruco::estimatePoseSingleMarkers(corners, 0.05f, cameraMatrix, distCoeffs, rvecs, tvecs);

			for (int i = 0; i < ids.size(); i++)
			{
				cv::aruco::drawAxis(canvas, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1f);
			}

			std::vector<cv::Point2f> charucoCorners;
			std::vector<int> charucoIds;
			cv::aruco::interpolateCornersCharuco(corners, ids, target, board, charucoCorners, charucoIds, cameraMatrix, distCoeffs);

			// if at least one charuco corner detected
			// SPDLOG_INFO("charuco corner: {}", charucoIds.size());

			if (charucoIds.size() > 0)
			{
				cv::aruco::drawDetectedCornersCharuco(canvas, charucoCorners, charucoIds, cv::Scalar(255, 0, 0));

				cv::Vec3d rvec, tvec;
				bool valid = cv::aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, board, cameraMatrix, distCoeffs, rvec, tvec);

				// if charuco pose is valid
				if (valid)
				{
					cv::aruco::drawAxis(canvas, cameraMatrix, distCoeffs, rvec, tvec, 0.1);
				}
			}
		}

		cv::imshow("infrared", canvas);
		// cv::imshow("depth", depth);
		char key = (char)cv::waitKey(30);

		if (key == 27)
		{
			break;
		}
	}
}
catch (const rs2::error &e)
{
	SPDLOG_CRITICAL("RealSense error calling {}({}): {}", e.get_failed_function(), e.get_failed_args(), e.what());
	return EXIT_FAILURE;
}
catch (const std::exception &e)
{
	SPDLOG_CRITICAL("Caught Exception: {}", e.what());
	return EXIT_FAILURE;
}
