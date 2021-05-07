
#include <fstream>
#include <string>
#include <iostream>

#include <fmt/format.h>
#include <spdlog/spdlog.h>

#include <Open3D/Open3D.h>
#include <librealsense2/rs.hpp>

#include "utility.h"
#include "board.h"
#include "camera_info.h"
#include "device_manager.h"
#include "stream_config.h"
#include "streamer.h"
#include "sensor_info.h"
#include "calibration.h"
#include "pointcloud.h"
#include <preset.h>
#include <exception.h>

using namespace symb::scanner;
using namespace error;

void display(const std::string &window_name, Frame &frame)
{
	SPDLOG_INFO("Press Any key to close Window");
	cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
	cv::imshow(window_name, frame.get_image(rs2_stream::RS2_STREAM_INFRARED));
	cv::waitKey(0);
	cv::destroyWindow(window_name);
}

int main() try
{
	SPDLOG_INFO("Start sample app");

	namespace fs = std::experimental::filesystem;

	// ----------------------- ログ出力をUTF-8に設定 ----------------------- //
	SetConsoleOutputCP(65001);

	// ----------------------- 保存パスの設定 ----------------------- //
	const fs::path current_path = fs::current_path();
	const fs::path session_data_dir = current_path / fmt::format("sessions/session_{}", utility::get_date_time());

	// ----------------------- PCI-USBハブのソフトリセット ----------------------- //
	system(fmt::format("{} restart PCI\\VEN_1912", (current_path / "devcon.exe").string()).c_str());

	// ----------------------- キャリブレーションボードの設定 ----------------------- //
	CalibrationBoard calibration_board;
	const fs::path calibration_board_dir = current_path / "board_data";

	if (!calibration_board.load(calibration_board_dir))
	{
		calibration_board.generate(8, 35, 0.05f, 0.03f, 6, 0.03f);
		calibration_board.save(calibration_board_dir);
		calibration_board.save_image(calibration_board_dir);
	}

	// ----------------------- カメラの接続 ----------------------- //
	const CameraInfo info("917622061166", "A1", "RF", 1);

	DeviceManager manager; // デバイスの接続管理クラス	
	manager.reset(); // USBポートに接続中のすべてのデバイスをリセット

	SPDLOG_INFO("Waiting for connection for {}", info.serial());
	// 接続待ちのループ
	while (!manager.is_connected(info.serial()))
	{
		SPDLOG_INFO("Camera Not Connected");
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	}
	SPDLOG_INFO("Camera Connected");

	// ----------------------- プリセットの初期設定 ----------------------- //
	SPDLOG_INFO("Setting Preset");
	{
		std::ifstream j(current_path / "preset.json");
		const std::string json((std::istreambuf_iterator<char>(j)), std::istreambuf_iterator<char>());
		j.close();
		Preset::set_preset(manager.get_rs2_device(info.serial()), json);
	}
	SPDLOG_INFO("Setting Preset Complete");

	// ----------------------- デプスセンサの設定 ----------------------- //
	SPDLOG_INFO("Setting Depth Sensor Options");
	{
		rs2::device device = manager.get_rs2_device(info.serial());
		rs2::sensor sensor = device.first<rs2::depth_sensor>().as<rs2::sensor>();
		SensorInfo sensor_info(sensor);
		sensor_info.set_option(rs2_option::RS2_OPTION_EMITTER_ENABLED, 1.0f);
		sensor_info.set_option(rs2_option::RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1.0f);
		sensor_info.set_option(rs2_option::RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, 0.0f);
	}
	SPDLOG_INFO("Setting Depth Sensor Options Completed");

	// ----------------------- キャリブレーション ----------------------- //
	SPDLOG_INFO("Calibration");
	CalibrationData calibration; // キャリブレーションデータ
	float depth_scale = 0.0001f;
	{
		// キャリブレーション用 Config
		std::vector<StreamConfig> calibration_config{StreamConfig(rs2_stream::RS2_STREAM_INFRARED, rs2_format::RS2_FORMAT_Y8, 6, 1280, 720, 1)};

		// 赤外線エミッタの一時解除
		rs2::device device = manager.get_rs2_device(info.serial());
		rs2::sensor sensor = device.first<rs2::depth_sensor>().as<rs2::sensor>();
		SensorInfo sensor_info(sensor);
		sensor_info.set_option(rs2_option::RS2_OPTION_EMITTER_ENABLED, 0.0f);

		// ストリームを開始してフレームを取得
		std::shared_ptr<Streamer> streamer = std::make_shared<Streamer>(calibration_config, device);
		streamer->start();
		Frameset frameset = streamer->get_frameset(1, 5000); // 5000ms以内に1フレーム取得する
		streamer->stop();
		streamer->close();

		// 赤外線エミッタの復帰
		sensor_info.set_option(rs2_option::RS2_OPTION_EMITTER_ENABLED, 1.0f);

		// キャリブレーションデータの生成
		depth_scale = sensor_info.get_depth_scale();
		const rs2_intrinsics intrinsics = streamer->get_intrinsics(rs2_stream::RS2_STREAM_INFRARED);
		const Frame frame = frameset[0];
		calibration = CalibrationData(calibration_board, depth_scale, intrinsics, frame, info.side());
	}
	SPDLOG_INFO("Calibration Completed");

	// ----------------------- キャプチャ ----------------------- //
	SPDLOG_INFO("Capture");
	Frameset raw_frameset; // ストリームから直接取得したフレーム
	{
		// キャプチャ用 Config
		std::vector<StreamConfig> capture_config;
		capture_config.emplace_back(StreamConfig(rs2_stream::RS2_STREAM_DEPTH, rs2_format::RS2_FORMAT_Z16, 6, 1280, 720));
		capture_config.emplace_back(StreamConfig(rs2_stream::RS2_STREAM_COLOR, rs2_format::RS2_FORMAT_BGR8, 6, 1280, 720));

		// ストリームを開始してフレームを取得
		std::shared_ptr<Streamer> streamer = std::make_shared<Streamer>(capture_config, manager.get_rs2_device(info.serial()));
		streamer->start();
		raw_frameset = streamer->get_frameset(4, 8000); // 8000ms以内に4フレーム取得する
		streamer->stop();
		streamer->close();
	}
	SPDLOG_INFO("Capture Completed");

	// ----------------------- フレームの処理と点群の生成 ----------------------- //
	SPDLOG_INFO("Processing Frames");
	Frame processed_frame; // フィルタをかけられたフレーム
	PointCloud pointcloud; // 点群データ
	{
		// 点群生成する範囲(キャリブレーションマーカー最下部の中心を原点とする)
		const utility::Range3Df range{{-0.5f, 0.5f}, {-0.24f, 1.8f}, {-0.5f, 0.5f}};

		// 複数フレームからフィルタをかけてフレームを取得する
		processed_frame = raw_frameset.get_filtered();

		// フィルタにかけられたフレームから点群を生成する
		pointcloud = PointCloud(calibration, processed_frame, depth_scale, range);

		// Voxelサイズを指定してダウンサンプル (点群絞り)
		pointcloud.downSample(0.003);

		// (ノイズのような)小さい点群を排除
		pointcloud.removeStatisticalOutliers(100, 1.5);

		// 法線の推定 (この時点では方向問題未解決)
		pointcloud.estimateNormals(30);

		// 法線方向の解決 (キャリブレーション時に推定したカメラ座標を使用)
		pointcloud.orientNormals(calibration.get_camera_coordinate());
	}
	SPDLOG_INFO("Processing Frames　Completed");

	// ----------------------- 以下保存関連 ----------------------- //
	// ----------------------- 書き出しディレクトリの生成 ----------------------- //
	const fs::path calibration_path = session_data_dir / "single_calibration";
	const fs::path capture_path = session_data_dir / "single_capture";
	{
		utility::mkdir_if_not_exist(calibration_path);
		utility::mkdir_if_not_exist(capture_path);
	}

	// ----------------------- キャリブレーションデータの保存 ----------------------- //
	{
		const fs::path xml_path = calibration_path / fmt::format("{}_{}.xml", info.id(), info.serial());
		const fs::path bef_path = calibration_path / fmt::format("{}_{}_before.png", info.id(), info.serial());
		const fs::path aft_path = calibration_path / fmt::format("{}_{}_after.png", info.id(), info.serial());

		// キャリブレーションデータ
		calibration.save_xml(xml_path);

		// キャリブレーションの際に使用したフレーム画像の保存
		calibration.save_images(bef_path, aft_path);
	}

	// ----------------------- キャプチャした生フレームの保存 ----------------------- //
	{
		const int frame_count = raw_frameset.size();
		for (int i = 0; i < frame_count; i++)
		{
			Frame frame = raw_frameset[i];
			rs2::frameset rs2_frame = raw_frameset[i]; // rs2::framesetとしての取得も可能

			const cv::Mat depth_image = frame.get_image(rs2_stream::RS2_STREAM_DEPTH); // デプス画像
			const cv::Mat color_image = frame.get_image(rs2_stream::RS2_STREAM_COLOR); // カラー画像

			cv::imwrite((capture_path / fmt::format("{}_depth_raw_{}.png", info.id(), i)).string(), depth_image);
			cv::imwrite((capture_path / fmt::format("{}_color_raw_{}.png", info.id(), i)).string(), color_image);
		}
	}

	// ----------------------- フィルタ後のフレームの保存 ----------------------- //
	{
		const cv::Mat processed_depth = processed_frame.get_image(rs2_stream::RS2_STREAM_DEPTH); // デプス画像
		const cv::Mat processed_color = processed_frame.get_image(rs2_stream::RS2_STREAM_COLOR); // カラー画像

		cv::imwrite((capture_path / fmt::format("{}_aligned_depth_depth_filtered.png", info.id())).string(), processed_depth);
		cv::imwrite((capture_path / fmt::format("{}_aligned_depth_color_filtered.png", info.id())).string(), processed_color);
	}

	// ----------------------- 点群の保存 ----------------------- //
	{
		open3d::io::WritePointCloudToPLY((capture_path / fmt::format("pointcloud_{}.ply", info.id())).string(), pointcloud, false, false);
	}

	SPDLOG_INFO("Stopping sample app");

	return EXIT_SUCCESS;
}
// キャリブレーションデータ生成時にマーカーがみつからなかったとき
catch (const NoMarkersFoundError &e)
{
	SPDLOG_CRITICAL("No Markers found during calibration data generation.");
	Frame frame = e.getFrame();
	if (frame.from_stream(rs2_stream::RS2_STREAM_INFRARED))
		display("NoMarkersFoundError", frame);
	return EXIT_FAILURE;
}
// キャリブレーションデータ生成時のマーカー数が不足しているとき
catch (const InsufficientMarkersError &e)
{
	SPDLOG_CRITICAL("Insufficient Markers found during calibration data generation. {} markers.", e.getMarkersFound());
	Frame frame = e.getFrame();
	if (frame.from_stream(rs2_stream::RS2_STREAM_INFRARED))
		display("InsufficientMarkersError", frame);
	return EXIT_FAILURE;
}
// キャリブレーションデータ生成時に原点推定が失敗したとき
catch (const CornerEstimationError &e)
{
	SPDLOG_CRITICAL("Corner Estimation Failed during calibration data generation. {} markers.", e.getMarkersFound());
	Frame frame = e.getFrame();
	if (frame.from_stream(rs2_stream::RS2_STREAM_INFRARED))
		display("CornerEstimationError", frame);
	return EXIT_FAILURE;
}
// センサオプション設定時の値がレンジ外のとき (bad programming)
catch (const OptionOutOfRangeError &e)
{
	SPDLOG_CRITICAL(e.what());
	char c;
	std::cin >> c;
	return EXIT_FAILURE;
}
// センサオプション設定時が未対応のとき (bad programming)
catch (const OptionUnsupportedError &e)
{
	SPDLOG_CRITICAL(e.what());
	char c;
	std::cin >> c;
	return EXIT_FAILURE;
}
// センサをリセットすれば再実行可能なエラー
catch (const RecoverableError &e)
{
	SPDLOG_CRITICAL(e.what());
	char c;
	std::cin >> c;
	return EXIT_FAILURE;
}
// センサをリセットしても再実行可能ではないエラー(USB抜き差しが必要OR bad programming)
catch (const UnRecoverableError &e) 
{
	SPDLOG_CRITICAL(e.what());
	char c;
	std::cin >> c;
	return EXIT_FAILURE;
}
// その他エラー(原則投げられないはず)
catch (const Error &e)
{
	SPDLOG_CRITICAL(e.what());
	char c;
	std::cin >> c;
	return EXIT_FAILURE;
}
catch (const rs2::error &e)
{
	SPDLOG_CRITICAL("RS2 Error at main [{}({}):{}]", e.get_failed_function(), e.get_failed_args(), e.what());
	char c;
	std::cin >> c;
	return EXIT_FAILURE;
}
catch (const std::exception &e)
{
	SPDLOG_CRITICAL("Excpetion at main [{}]", e.what());
	char c;
	std::cin >> c;
	return EXIT_FAILURE;
}
catch (...)
{
	SPDLOG_CRITICAL("Unknown Excpetion at main");
	return EXIT_FAILURE;
}
