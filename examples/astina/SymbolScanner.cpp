#include "SymbolScanner.h"

#include <fstream>
#include <string>

#include <fmt/format.h>
#include <spdlog/spdlog.h>

#include <librealsense2/rs.hpp>

#include "alignment.h"
#include "board.h"
#include "calibration.h"
#include "device_manager.h"
#include "exception.h"
#include "pointcloud.h"
#include "preset.h"
#include "sensor_info.h"
#include "stream_config.h"
#include "streamer.h"
#include "utility.h"

#include "helper/camera_info_reader.h"
#include "helper/camera_info_with_data.h"
#include "helper/corner_visualizer.h"
#include "helper/multithreader.h"

#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include <conio.h>

using namespace symb::scanner;
using namespace error;
using namespace helper;

static bool isDummy = false;		// ダミーモード
static bool last_dummyMode = false;	// 前回のダミーモード
static bool isCapture_or_Live = false;	// キャプチャかライブで、multithreaderからのタイムアウトでのカメラリセットキャンセル用フラグ

static void SetModeDetail(const CAMERA_MODE new_mode, const unsigned int timeout, const bool &enable_multithread, const bool &enable_test_cap);

// コマンドからパスを取得
static std::experimental::filesystem::path extract_path_in_commands(const std::vector<std::string> &commands, const uint offset = 1)
{
	std::ostringstream oss;
	std::copy(commands.begin() + offset, commands.end(), std::ostream_iterator<std::string>(oss, " "));
	std::string s = oss.str();
	return std::experimental::filesystem::path(s.erase(s.size() - 1));
}

// カメラ群+エラー情報からシリアル番号群を取得
static std::vector<std::string> extract_serial(std::vector<std::shared_ptr<CameraInfoWithData>> &exceptions)
{
	std::vector<std::string> res;
	res.reserve(exceptions.size());
	for (const std::shared_ptr<CameraInfoWithData> &info : exceptions)
	{
		res.emplace_back(info->serial());
	}
	return res;
}

static bool initialized = false;
static std::string global_preset_json_path = "./preset.json";

// 全カメラの情報
static std::vector<std::shared_ptr<CameraInfoWithData>> infos;
static std::vector<std::shared_ptr<CameraInfoWithData>> enabled_infos;

// managerを使って接続待ちを行う
// 永遠に接続されなかったら "q" キーでプログラムが終了
static void wait_for_connections(DeviceManager &manager, std::vector<std::shared_ptr<CameraInfoWithData>> disconnected, const unsigned int timeout_seconds = 90)
{
	const size_t wait_size = disconnected.size();
	bool has_fresh = false;
	unsigned int reset_count = 4;
	bool has_disconnected = false;
	bool first_loop = true;
	std::vector<std::shared_ptr<CameraInfoWithData>> connected;

	// デバイス接続復帰後にPreset設定するデバイスリスト
	std::set<std::shared_ptr<CameraInfoWithData>> fresh_device_list;
	bool force_disconnect_all = false;

	std::chrono::steady_clock::time_point time = std::chrono::steady_clock::now();
	const std::chrono::steady_clock::time_point timeout = time + std::chrono::seconds(timeout_seconds);
	while (!disconnected.empty())
	{
		for (std::vector<std::shared_ptr<CameraInfoWithData>>::iterator itr = disconnected.begin(); itr != disconnected.end();)
		{
			const std::string serial = (*itr)->serial();
			const bool is_fresh = manager.is_fresh(serial);
			if (is_fresh)
			{
				if (first_loop)
					has_fresh = true;
				fresh_device_list.insert(*itr);
			}
			if (manager.is_connected(serial))
			{
				connected.emplace_back(*itr);
				itr = disconnected.erase(itr);
			}
			else
			{
				++itr;
			}
		}

		for (std::vector<std::shared_ptr<CameraInfoWithData>>::iterator itr = connected.begin(); itr != connected.end();)
		{
			if (!manager.is_connected((*itr)->serial()))
			{
				disconnected.emplace_back(*itr);
				itr = connected.erase(itr);
			}
			else
			{
				++itr;
			}
		}

		has_disconnected = !disconnected.empty();

		if (first_loop && has_disconnected)
		{
			SPDLOG_INFO("Waiting for connections");
		}

		if (!first_loop)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(300));
		}

		if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - time).count() > 5)
		{
			const size_t camera_count = disconnected.size();
			for (size_t i = 0; i < camera_count; ++i)
			{
				SPDLOG_INFO("{:>2}/{} Waiting connections for {}", i + 1, camera_count, disconnected[i]->name());
			}

			if (reset_count++ > 5)
			{
				SPDLOG_INFO("Force Resetting ALL Cameras");
				manager.reset();
				reset_count = 0;
				force_disconnect_all = true;
			}
			else
			{
				SPDLOG_INFO("Force Resetting Disconnected Cameras");
				SPDLOG_INFO("disconnected size: {}", disconnected.size());
				manager.reset(extract_serial(disconnected));
			}

			if (manager.get_system_exception_count() >= wait_size)
			{
				std::string str_devices = "";
				if (!disconnected.empty())
				{
					str_devices = "Disconnected(";

					// 切断状態のデバイスのIDとSerialNoを取得
					for (const auto &dev : disconnected)
					{
						symb::scanner::utility::join({dev->id(), "[", dev->location(), "]:", dev->serial(), ","}, str_devices);
					}
					str_devices.replace(str_devices.size() - 1, 1, ")");
				}
				throw SymbolErrors({std::runtime_error(fmt::format("[Unrecoverable] System Exception count times exceeded. {}", str_devices))});
			}

			time = std::chrono::steady_clock::now();
		}
		if (std::chrono::steady_clock::now() > timeout)
		{
			std::string str_devices = "";
			if (!disconnected.empty())
			{
				str_devices = "Disconnected(";

				// 切断状態のデバイスのIDとSerialNoを取得
				for (const auto &dev : disconnected)
				{
					symb::scanner::utility::join({dev->id(), "[", dev->location(), "]:", dev->serial(), ","}, str_devices);
				}
				str_devices.replace(str_devices.size() - 1, 1, ")");
			}
			throw SymbolErrors({std::runtime_error(fmt::format("[Unrecoverable] Camera Waiting for connection timeout {} seconds. {}", timeout_seconds, str_devices))});
		}

		first_loop = false;
	}

	// hardware_resetしたRealSenseは、presetも初期化されるので再度設定
	if (initialized)
	{
		// SPDLOG_INFO("Resetting preset[all:{}, part:{}]", force_disconnect_all, !fresh_device_list.empty());
		if (force_disconnect_all)
		{
			SetPreset(5, global_preset_json_path.c_str());
		}
		else if (!fresh_device_list.empty())
		{
			const size_t fresh_size = fresh_device_list.size();
			const std::vector<std::shared_ptr<CameraInfoWithData>> tmp_enabled_infos = enabled_infos;
			enabled_infos.clear();
			enabled_infos.reserve(fresh_size);
			for (const auto &info : fresh_device_list)
				enabled_infos.emplace_back(info);
			SetPreset(5, global_preset_json_path.c_str());
			enabled_infos = tmp_enabled_infos;
		}
	}

	if (has_disconnected || has_fresh)
	{
		const CAMERA_MODE prev_mode = GetMode();
		if (prev_mode != NONE)
		{
			if (isDummy)
			{
				SetMode(NONE);
				SetMode(prev_mode);
			}
			else
			{
				SetColorMode(NONE);
				SetColorMode(prev_mode);
			}
		}
		SPDLOG_INFO("All Cameras Connected");
	}
}

// wait_for_connectionsのシングルスレッド版
// SetModexxから呼ばれる場合は、call_setmodeはfalseにする
static void wait_for_connections_sequential(DeviceManager &manager, std::vector<std::shared_ptr<CameraInfoWithData>> disconnected, const bool call_setmode = true,
											const unsigned int timeout_seconds = 90)
{
	const size_t wait_size = disconnected.size();
	bool has_fresh = false;
	unsigned int reset_count = 4;
	bool has_disconnected = false;
	bool first_loop = true;
	std::vector<std::shared_ptr<CameraInfoWithData>> connected;
	
	// デバイス接続復帰後にPreset設定するデバイスリスト
	std::set<std::shared_ptr<CameraInfoWithData>> fresh_device_list;
	bool force_disconnect_all = false;

	std::chrono::steady_clock::time_point time = std::chrono::steady_clock::now();
	const std::chrono::steady_clock::time_point timeout = time + std::chrono::seconds(timeout_seconds);
	while (!disconnected.empty())
	{
		for (std::vector<std::shared_ptr<CameraInfoWithData>>::iterator itr = disconnected.begin(); itr != disconnected.end();)
		{
			const std::string serial = (*itr)->serial();
			const bool is_fresh = manager.is_fresh(serial);
			if (is_fresh)
			{
				if (first_loop)
					has_fresh = true;
				fresh_device_list.insert(*itr);
			}
			if (manager.is_connected(serial))
			{
				connected.emplace_back(*itr);
				itr = disconnected.erase(itr);
			}
			else
			{
				++itr;
			}
		}

		for (std::vector<std::shared_ptr<CameraInfoWithData>>::iterator itr = connected.begin(); itr != connected.end();)
		{
			if (!manager.is_connected((*itr)->serial()))
			{
				disconnected.emplace_back(*itr);
				itr = connected.erase(itr);
			}
			else
			{
				++itr;
			}
		}

		has_disconnected = !disconnected.empty();

		if (first_loop && has_disconnected)
		{
			SPDLOG_INFO("Waiting for connections");
		}

		if (!first_loop)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(300));
		}
		if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - time).count() > 5)
		{
			const size_t camera_count = disconnected.size();
			for (size_t i = 0; i < camera_count; ++i)
			{
				SPDLOG_INFO("{:>2}/{} Waiting connections for {}", i + 1, camera_count, disconnected[i]->name());
			}

			if (reset_count++ > 5)
			{
				SPDLOG_INFO("Force Resetting ALL Cameras");
				manager.reset();
				reset_count = 0;
				force_disconnect_all = true;
			}
			else
			{
				SPDLOG_INFO("Force Resetting Disconnected Cameras");
				SPDLOG_INFO("disconnected size: {}", disconnected.size());
				manager.reset(extract_serial(disconnected));
			}

			if (manager.get_system_exception_count() >= wait_size)
			{
				std::string str_devices = "";
				if (!disconnected.empty())
				{
					str_devices = "Disconnected(";

					// 切断状態のデバイスのIDとSerialNoを取得
					for (const auto &dev : disconnected)
					{
						symb::scanner::utility::join({dev->id(), "[", dev->location(), "]:", dev->serial(), ","}, str_devices);
					}
					str_devices.replace(str_devices.size() - 1, 1, ")");
				}
				throw SymbolErrors({std::runtime_error(fmt::format("[Unrecoverable] System Exception count times exceeded. {}", str_devices))});
			}

			time = std::chrono::steady_clock::now();
		}
		if (std::chrono::steady_clock::now() > timeout)
		{
			std::string str_devices = "";
			if (!disconnected.empty())
			{
				str_devices = "Disconnected(";

				// 切断状態のデバイスのIDとSerialNoを取得
				for (const auto &dev : disconnected)
				{
					symb::scanner::utility::join({dev->id(), "[", dev->location(), "]:", dev->serial(), ","}, str_devices);
				}
				str_devices.replace(str_devices.size() - 1, 1, ")");
			}
			throw SymbolErrors({std::runtime_error(fmt::format("[Unrecoverable] Camera Waiting for connection timeout {} seconds. {}", timeout_seconds, str_devices))});
		}

		first_loop = false;
	}

	// hardware_resetしたRealSenseは、presetも初期化されるので再度設定
	if (initialized)
	{
		// SPDLOG_INFO("Resetting preset[all:{}, part:{}]", force_disconnect_all, !fresh_device_list.empty());
		if (force_disconnect_all)
		{
			SetPresetSequential(5, global_preset_json_path.c_str());
		}
		else if (!fresh_device_list.empty())
		{
			const size_t fresh_size = fresh_device_list.size();
			const std::vector<std::shared_ptr<CameraInfoWithData>> tmp_enabled_infos = enabled_infos;
			enabled_infos.clear();
			enabled_infos.reserve(fresh_size);
			for (const auto &info : fresh_device_list)
				enabled_infos.emplace_back(info);
			SetPresetSequential(5, global_preset_json_path.c_str());
			enabled_infos = tmp_enabled_infos;
		}
	}

	if (has_disconnected || has_fresh)
	{
		const CAMERA_MODE prev_mode = GetMode();
		if (prev_mode != NONE && call_setmode == true)
		{
			if (isDummy)
			{
				SetModeWithoutTestCapture(NONE);
				SetModeWithoutTestCapture(prev_mode);
			}
			else
			{
				SetColorModeWithoutTestCapture(NONE);
				SetColorModeWithoutTestCapture(prev_mode);
			}
		}
		SPDLOG_INFO("All Cameras Connected");
	}
}

// Multithreadから投げられた例外の処理を行う
static void handle_exception(DeviceManager &manager, std::vector<std::pair<std::shared_ptr<CameraInfoWithData>, Error>> &errors)
{
	std::vector<std::runtime_error> converted_errors;
	std::vector<std::string> reset;
	bool isTimeout = false;

	for (const std::pair<std::shared_ptr<CameraInfoWithData>, Error> &itr : errors)
	{
		std::string msg;
		const std::shared_ptr<CameraInfoWithData> &info = itr.first; // カメラ情報
		const Error &error = itr.second; // エラー
		switch (error.getErrorType())
		{
		case ErrorType::BAD_STATE:
			msg = fmt::format("{} in bad state (probably due to bad programming) [{}]", info->name(), error.what());
			break;
		case ErrorType::INVALID_PARAMETER:
			msg = fmt::format("{} in threw invalid parameter error [{}]", info->name(), error.what());
			break;
		case ErrorType::CALIBRATION:
		{
			Frame frame;
			switch (error.getCalibrationErrorType())
			{
			case CalibrationErrorType::NO_MARKERS:
			{
				const NoMarkersFoundError extended = error.as<NoMarkersFoundError>();
				frame = extended.getFrame();
				msg = fmt::format("{} found no markers in frame", info->name());
				break;
			}
			case CalibrationErrorType::INSUFFICIENT_MARKERS:
			{
				const InsufficientMarkersError extended = error.as<InsufficientMarkersError>();
				frame = extended.getFrame();
				msg = fmt::format("{} unable to find 5 markers. {} markers found.", info->name(), extended.getMarkersFound());
				break;
			}
			case CalibrationErrorType::FAILED_CORNER_ESTIMATION:
			{
				const CornerEstimationError extended = error.as<CornerEstimationError>();
				frame = extended.getFrame();
				msg = fmt::format("{} unable to estimate corners. {} markers found.", info->name(), extended.getMarkersFound());
				break;
			}
			default:
			case CalibrationErrorType::DEFAULT:
			{
				msg = fmt::format("{} threw [{}]", info->name(), error.what());
				break;
			}
			}
			if (frame)
			{
				// フレームを表示する
				if (frame.from_stream(rs2_stream::RS2_STREAM_INFRARED))
				{
					msg = fmt::format("{} Displaying Failed Frame [{}]", info->serial(), error.what());
					// const std::string window_name = fmt::format("Calibration Failed {} [Press Any key to close window]", info->serial());
					// cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
					// cv::imshow(window_name, frame.get_image(rs2_stream::RS2_STREAM_INFRARED));
					// cv::waitKey(0);
					// cv::destroyWindow(window_name);
				}
			}
			break;
		}
		case ErrorType::SENSOR_INFO:
			switch (error.getOptionType())
			{
			case OptionErrorType::OUT_OF_RANGE:
			{
				const OptionOutOfRangeError extended = error.as<OptionOutOfRangeError>();
				msg = fmt::format("{} threw [{}]", info->name(), extended.what());
				break;
			}
			case OptionErrorType::UNSUPPORTED:
			{
				const OptionUnsupportedError extended = error.as<OptionUnsupportedError>();
				msg = fmt::format("{} threw [{}]", info->name(), extended.what());
				break;
			}
			default:
			case OptionErrorType::DEFAULT:
			{
				msg = fmt::format("{} threw default option error [{}]", info->name(), error.what());
				break;
			}
			}
			break;
		case ErrorType::UNKNOWN:
			msg = fmt::format("{} threw unknown error [{}]", info->name(), error.what());
			break;

		case ErrorType::TIMEOUT:
			msg = fmt::format("{} threw default error [{}]", info->name(), error.what());
			isTimeout = true;
			break;
		default:
		case ErrorType::DEFAULT:
			msg = fmt::format("{} threw default error [{}]", info->name(), error.what());
			break;
		}

		SPDLOG_WARN(msg);

		// リセットが必要なものはリセットを行うようにする
		// キャプチャ及びライブでない場合のみリセットの対象
		if (isCapture_or_Live == false && error.requiresReset())
		{
			reset.emplace_back(info->serial());
		}
		else
		{
			converted_errors.emplace_back(std::runtime_error(msg));
		}
	}

	// リセットする
	if (!reset.empty())
	{
		SPDLOG_WARN("Resetting {} devices", reset.size());
		manager.reset(reset);
	}

	if (!converted_errors.empty())
	{
		// タイムアウトは別エラーとする
		if (isTimeout)
		{
			throw SymbolTimeoutErrors(converted_errors);
		}
		else
		{
			throw SymbolErrors(converted_errors);
		}
	}
}

// 同時実行するカメラの台数
static unsigned int thread_num = 5;

static bool calibrated = false;

static CAMERA_MODE mode = NONE;
static CAMERA_MODE frame_mode = NONE;
static std::mutex streamers_mutex;
static std::map<std::shared_ptr<CameraInfoWithData>, Streamer> streamers;

static CalibrationBoard calibration_board;

// WindowsではDeviceManagerはGlobal領域で使えない
// https://github.com/IntelRealSense/librealsense/issues/3138
static DeviceManager &manager()
{
	static DeviceManager manager_;
	return manager_;
}

static utility::Range3Df range;

// 床オフセット値（土台に乗って撮影した場合などの、土台部分削除用）
static double offset_y;
// 床の強制削除フラグ（土台に乗った場合などに使う。falseの場合でも平面推定により床は除去される）
static bool cut_floor;
// 足とみなす範囲[x方向]（足以外のノイズ除去用）
static double foot_range;

static bool pc_valid = false;
static std::map<std::shared_ptr<CameraInfoWithData>, Frame> f_filter;
static std::map<std::shared_ptr<CameraInfoWithData>, PointCloud> pc_filter;
static PointCloud pc;

//オリジナルサイズの点群(カメラ毎)
static std::map<std::shared_ptr<CameraInfoWithData>, PointCloud> org_pc_filter;

static bool streaming_live;
static std::shared_ptr<std::thread> live_thread;
static std::shared_ptr<SymbolErrors> live_errors;

static std::string calibrate_time = "0000_00_00_00_00_00_000";
static std::string capture_time = "0000_00_00_00_00_00_000";
static std::string generate_time = "0000_00_00_00_00_00_000";

static std::experimental::filesystem::path calibration_path_ = "c:/symb/calibration";

static std::map<CAMERA_MODE, bool> emitter_status;

static const float laser_power = 150;
static const float depth_exposure = 40000.0f; // 40ms
static const std::vector<int> compression_params = {cv::IMWRITE_PNG_COMPRESSION, 0};

static void init_emitter_status()
{
	emitter_status[NONE] = false;
	emitter_status[CALIBRATE] = true;
	emitter_status[CAPTURE] = true;
	emitter_status[LIVE] = true;
}

static void init_logger(std::string log_dir)
{
	std::vector<spdlog::sink_ptr> sinks;
	sinks.push_back(std::make_shared<spdlog::sinks::stdout_color_sink_mt>());

	if (log_dir.length() == 0)
	{
		log_dir = std::string("c:/symb/log");
	}
	SPDLOG_DEBUG("log_dir[{0}]", log_dir);

	const std::experimental::filesystem::path dir(log_dir);

	std::experimental::filesystem::create_directories(dir);
	sinks.push_back(std::make_shared<spdlog::sinks::rotating_file_sink_mt>((dir / fmt::format("api_log_{}.log", utility::get_date_time())).string(), 1048576 * 5, 3));

	const std::shared_ptr<spdlog::logger> combined_logger = std::make_shared<spdlog::logger>("AstinaScanner", begin(sinks), end(sinks));
	combined_logger->flush_on(spdlog::level::info);
	spdlog::register_logger(combined_logger);
	spdlog::set_default_logger(combined_logger);
	spdlog::set_level(spdlog::level::info);
	spdlog::set_pattern("[%6t]%+");
}

static void throwSymbolTimeoutErrors(std::string msg)
{
	SPDLOG_INFO("throwSymbolTimeoutErrors: {}", msg);
	if (initialized)
	{
		Terminate();
	}

	std::vector<std::runtime_error> converted_errors;
	converted_errors.emplace_back(std::runtime_error(msg));

	throw SymbolTimeoutErrors(converted_errors);
}

static void generate_board(const struct _board_params &params)
{
	const bool stat = calibration_board.generate(params.square_count_x, params.square_count_y, params.square_length, params.marker_length, params.marker_size, params.thickness);
	if (!stat)
	{
		throw std::runtime_error("Calibration Board Generation Failed");
	}
}

static void read_config(const std::experimental::filesystem::path &config_path)
{
	infos.clear();
	CameraInfoReader reader(config_path);
	infos = reader.getResult();
	enabled_infos = infos;
}

static void set_calibration_path(std::experimental::filesystem::path calibration_path)
{
	if (calibration_path.string().length() <= 0)
	{
		calibration_path = calibration_path_;
	}
	SPDLOG_DEBUG("calibration_dir[{0}]", calibration_path.string());
	if (std::experimental::filesystem::exists(calibration_path))
	{
		if (!std::experimental::filesystem::is_directory(calibration_path))
		{
			throw std::runtime_error(fmt::format("File Already Exists at Calibration Path({})", calibration_path.string()));
		}
	}
	if (std::experimental::filesystem::create_directories(calibration_path))
	{
		throw std::runtime_error(fmt::format("Unable to create Directory at Calibration Path({})", calibration_path.string()));
	}
	calibration_path_ = calibration_path;
}

static void connect_cameras()
{
	wait_for_connections(manager(), enabled_infos);
	manager().reset();
	wait_for_connections(manager(), enabled_infos);
}

static void connect_cameras_sequential()
{
	wait_for_connections_sequential(manager(), enabled_infos);
	manager().reset();
	wait_for_connections_sequential(manager(), enabled_infos);
}

static void set_range(_f_range3d _range)
{
	range.x.min = _range.x.min;
	range.x.max = _range.x.max;
	range.y.min = _range.y.min;
	range.y.max = _range.y.max;
	range.z.min = _range.z.min;
	range.z.max = _range.z.max;
}

static void do_multithread(std::function<void(std::shared_ptr<CameraInfoWithData>)> &lambda, const unsigned int attempts, const std::string &debug_name, const bool all_or_none = false,
						   const int timeout = 40)
{
	SPDLOG_DEBUG(debug_name);
	const std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
	try
	{
		std::vector<std::shared_ptr<CameraInfoWithData>> success;
		std::vector<std::shared_ptr<CameraInfoWithData>> target = enabled_infos;
		for (unsigned int i = 0u; i < attempts && !target.empty(); i++)
		{
			MultiThreader result(target, lambda, std::chrono::seconds(timeout));
			try
			{
				handle_exception(manager(), result.errors);
			}
			catch (...)
			{
				if (i >= attempts - 1)
				{
					throw;
				}
			}
			wait_for_connections(manager(), enabled_infos);
			if (!all_or_none || result.fail.empty())
			{
				target = std::move(result.fail);
				success.reserve(success.size() + result.success.size());
				success.insert(success.end(), result.success.begin(), result.success.end());
			}
		}
	}
	catch (...)
	{
		SPDLOG_WARN("{} Failed {}ms", debug_name, std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count());
		throw;
	}
	SPDLOG_DEBUG("{} Complete {}ms", debug_name, std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count());
}

// 指定された台数ずつ処理する。（今の形を変えたくないのでとりあえずdo_multithreadをそのまま流用）
// enabled_info.size()
std::vector<std::shared_ptr<CameraInfoWithData>> target_infos;

static void do_singlethread(std::function<void(std::shared_ptr<CameraInfoWithData>)> &lambda, const unsigned int attempts, const std::string &debug_name, const bool all_or_none = false,
							const int timeout = 40)
{
	SPDLOG_DEBUG(debug_name);
	const std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
	try
	{
		std::vector<std::shared_ptr<CameraInfoWithData>> success;
		std::vector<std::shared_ptr<CameraInfoWithData>> target = target_infos;
		for (unsigned int i = 0u; i < attempts && !target.empty(); i++)
		{
			MultiThreader result(target, lambda, std::chrono::seconds(timeout));
			try
			{
				handle_exception(manager(), result.errors);
			}
			catch (...)
			{
				if (i >= attempts - 1)
				{
					throw;
				}
			}
			wait_for_connections_sequential(manager(), target);
			if (!all_or_none || result.fail.empty())
			{
				target = std::move(result.fail);
				success.reserve(success.size() + result.success.size());
				success.insert(success.end(), result.success.begin(), result.success.end());
			}
		}
	}
	catch (...)
	{
		SPDLOG_WARN("{} Failed {}ms", debug_name, std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count());
		throw;
	}
	SPDLOG_DEBUG("{} Complete {}ms", debug_name, std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count());
}

/**
 * @brief
 *
 * @param lambda 繰り返し処理するラムダ式
 * @param attempts  試行回数
 * @param debug_name  デバッグ名(ログに使用)
 * @param all_or_none
 * @param timeout
 */
static void repeat_lamda(std::function<void(std::shared_ptr<CameraInfoWithData>)> &lambda, const unsigned int attempts, const std::string &debug_name, const bool all_or_none = false,
						 const int timeout = 40)
{
	std::chrono::system_clock::time_point start = std::chrono::system_clock::now();

	// 指定された台数ずつモード変更
	for (unsigned int i = 0; i < enabled_infos.size();)
	{
		target_infos.clear();
		target_infos.shrink_to_fit();
		for (unsigned int j = 0; j < thread_num; j++)
		{
			if (i >= enabled_infos.size())
				break;

			target_infos.push_back(enabled_infos.at(i));
			i++;
		}

		if (timeout > 0)
		{
			// タイムアウトチェック

			std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
			int elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start).count();
			// タイムアウトを決定
			int timeout_remain = timeout - elapsed;

			// ここで残り時間がなければタイムアウトにする。
			if (timeout_remain <= 0)
			{
				std::string msg;
				msg = fmt::format("{} Timeout timeout={}", debug_name, timeout);
				throwSymbolTimeoutErrors(msg);
			}
		}

		do_singlethread(lambda, attempts, debug_name, all_or_none, timeout);
	}
}

enum sensor_type
{
	depth,
	color
};

static void set_depth_table(const unsigned int retry_count, const unsigned int depth_unit = 30, const unsigned int disparity_shift = 40);

// static void set_depth_table_sequential(const unsigned int retry_count, const unsigned int depth_unit = 30, const unsigned int disparity_shift = 40);
static void set_depth_table_sequential(const unsigned int retry_count, const unsigned int depth_unit = 30, const unsigned int disparity_shift = 40, const int timeout = 0);

static void set_sensor_value(const sensor_type sensor_type, const std::vector<std::pair<rs2_option, float>> &option_pairs);

// static void set_sensor_value_sequential(const sensor_type sensor_type, const std::vector<std::pair<rs2_option, float>> &option_pairs);
static void set_sensor_value_sequential(const sensor_type sensor_type, const std::vector<std::pair<rs2_option, float>> &option_pairs, const int timeout);

static void set_laser_power(const float value);
static void set_emitter(const bool enable);

// static void set_emitter_sequential(const bool enable);
static void set_emitter_sequential(const bool enable, const int timeout);

static void set_exposure(const float value);
static void set_auto_exposure(const bool enable);
static void set_auto_white_balance(const bool enable);

#define CAMERA_INFO_NAME "D415"

// デバイス名およびUSBバージョンのチェック
// Initializeから呼び出される
static void check_device_info()
{
	if (!initialized)
	{
		throw UnInitializedError("CheckDeviceInfo");
	}
	if (StreamingLive())
	{
		throw StreamingSafeGuardError("CheckDeviceInfo");
	}

	std::string error_detail;
	bool check_error = false;

	std::function<void(std::shared_ptr<CameraInfoWithData>)> print_device_info_lambda = [&](const std::shared_ptr<CameraInfoWithData> &info) -> void {
		SPDLOG_INFO("{} CheckDeviceInfo", info->name());
		const rs2::device device = manager().get_rs2_device(info->serial());
		std::string camera_info_name = device.get_info(RS2_CAMERA_INFO_NAME);
		double usb_version = atof(device.get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR));
		SPDLOG_INFO("    Using device    : {}", camera_info_name);
		SPDLOG_INFO("    USB version     : {}", usb_version);
		SPDLOG_INFO("    Firmware version: {}", device.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION));

		if (camera_info_name.find(CAMERA_INFO_NAME) == std::string::npos)
		{
			error_detail += fmt::format("device info name error[{}] device[{}]", info->name(), camera_info_name);
			check_error = true;
		}
		if (usb_version < 3.0f)
		{
			error_detail += fmt::format("usb version error[{}] version[{}]", info->name(), usb_version);
			check_error = true;
		}
	};
	// 指定された台数ずつ確認する
	for (unsigned int i = 0; i < enabled_infos.size();)
	{
		target_infos.clear();
		target_infos.shrink_to_fit();
		for (unsigned int j = 0; j < thread_num; j++)
		{
			if (i >= enabled_infos.size())
				break;

			target_infos.push_back(enabled_infos.at(i));
			i++;
		}
		do_singlethread(print_device_info_lambda, 1, "PrintDeviceInfo", false, 90);
	}

	// デバイス名またはUSBバージョンのエラーがあればExceptionをthrowする
	if (check_error)
	{
		SPDLOG_ERROR("CheckDeviceError [{}]", error_detail);
		std::string msg;
		msg = fmt::format("CheckDeviceError [{}]", error_detail);
		std::vector<std::runtime_error> converted_errors;
		converted_errors.emplace_back(std::runtime_error(msg));
		throw SymbolErrors(converted_errors);
	}
	return;
}

SYMBOLSCANNER_API void Initialize(const struct InitializeParams params)
try
{
	if (initialized)
	{
		return;
	}

	SPDLOG_DEBUG("config_path[{0}]", params.config_file_path);
	SPDLOG_DEBUG("preset_path[{0}]", params.preset_json_path);
	SPDLOG_DEBUG("calibration_dir[{0}]", params.calibration_dir);
	SPDLOG_DEBUG("cut_floor[{0}]", params.cut_floor);
	SPDLOG_DEBUG("offset_y[{0}]", params.offset_y);
	SPDLOG_DEBUG("foot_range[{0}]", params.foot_range);
	SPDLOG_DEBUG("enable_set_preset[{0}]", params.enable_set_preset);
	SPDLOG_DEBUG("thread_num[{0}]", params.thread_num);

	thread_num = params.thread_num;
	SetPriorityClass(GetCurrentProcess(), ABOVE_NORMAL_PRIORITY_CLASS);
	manager();
	init_emitter_status();
	init_logger(params.log_dir);
	generate_board(params.board_params);
	set_range(params.range);
	cut_floor = params.cut_floor;
	offset_y = params.offset_y;
	foot_range = params.foot_range;
	global_preset_json_path = params.preset_json_path;
	
	read_config(std::experimental::filesystem::path(params.config_file_path));
	set_calibration_path(std::experimental::filesystem::path(params.calibration_dir));

	connect_cameras();

	initialized = true;

	// デバイス名およびUSBバージョンのチェック
	try
	{
		check_device_info();
	}
	catch (...)
	{
		if (params.enable_set_preset == true)
		{
			Terminate();
		}
		else
		{
			TerminateSequential();
		}

		throw;
	}

	try
	{
		if (params.enable_set_preset == true)
		{
			SetPreset(5, params.preset_json_path.c_str());
		}
	}
	catch (...)
	{
		if (params.enable_set_preset == true)
		{
			Terminate();
		}
		else
		{
			TerminateSequential();
		}
		throw;
	}
}
// Nomura
// Exceptionを追加（read_configからのExceptionを返すようにした）
catch (const InvalidParameterError &e)
{
	initialized = false;
	SPDLOG_ERROR("InvalidParameterError Error [{0}]", e.what());
	throw std::runtime_error(e.what());
}
catch (const BadStateError &e)
{
	initialized = false;
	SPDLOG_ERROR("BadStateError Error [{0}]", e.what());
	throw std::runtime_error(e.what());
}
catch (const std::exception &e)
{
	initialized = false;
	SPDLOG_ERROR("std::exception Exception [{0}]", e.what());
	throw;
}
catch (...)
{
	initialized = false;
	SPDLOG_DEBUG("###### Initialize exception");
	throw std::runtime_error("Initialize exception");
}

// 床の雄セットパラメータを設定（更新）
SYMBOLSCANNER_API void SetFloorOffsetParam(const bool _cut_floor, const double _offset_y, const double _foot_range)
{
	if (!initialized)
	{
		throw UnInitializedError("SetFloorOffsetParam");
	}

	SPDLOG_DEBUG("cut_floor[{0}]", _cut_floor);
	SPDLOG_DEBUG("offset_y[{0}]", _offset_y);
	SPDLOG_DEBUG("foot_range[{0}]", _foot_range);
	cut_floor = _cut_floor;
	offset_y = _offset_y;
	foot_range = _foot_range;
}

SYMBOLSCANNER_API CAMERA_MODE GetMode(void) { return mode; }

SYMBOLSCANNER_API unsigned int GetEnabledDeviceCount(void) { return static_cast<unsigned int>(enabled_infos.size()); }

static void close_stream_before_setmode(const CAMERA_MODE &last_mode, const CAMERA_MODE &new_mode, const bool &enable_multithread, int &timeout_remain)
{
	const bool enable_close = (last_mode != NONE);
	const bool enable_udpate_emitter = (emitter_status[last_mode] != emitter_status[new_mode]);

	const std::chrono::system_clock::time_point start = std::chrono::system_clock::now();
	const int timeout = timeout_remain;
	if (enable_close != NONE)
	{
		// ストリームがオープン状態であればMode切替前にクローズする
		std::function<void(std::shared_ptr<CameraInfoWithData>)> close_lambda = [&](const std::shared_ptr<CameraInfoWithData> &info) -> void {
			Streamer streamer = streamers[info];
			SPDLOG_INFO("{} closing", info->name());

			const std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
			streamer.stop();
			streamer.close();
			{
				std::lock_guard<std::mutex> _(streamers_mutex);
				streamers.erase(streamers.find(info));
			}
			SPDLOG_INFO("{} closing Complete. took {}ms.", info->name(), std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count());
		};

		if (enable_udpate_emitter)
		{
			if (enable_multithread)
			{
				set_emitter(emitter_status[new_mode]);
				// set_laser_power(emitter_status[new_mode] ? laser_power : 0);
			}
			else
			{
				if (new_mode == CAPTURE || new_mode == LIVE)
				{
					// Capture mode or Live mode に移行するとき、タイムアウト
					if (timeout > 0)
					{
						const std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
						const int elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
						timeout_remain = timeout - elapsed;

						// ここで残り時間がなければタイムアウトにする。
						if (timeout_remain <= 0)
						{
							SPDLOG_ERROR("SetModeWithoutTestCapture Timeout");
							std::string msg;
							msg = fmt::format("SetModeWithoutTestCapture Timeout timeout={}", timeout);
							throwSymbolTimeoutErrors(msg);
						}
						set_emitter_sequential(emitter_status[new_mode], timeout_remain);
					}
					else
					{
						set_emitter_sequential(emitter_status[new_mode], 0);
					}
				}
				else
				{
					// Calibration modeに移行するときは、タイムアウト設定しない
					set_emitter_sequential(emitter_status[new_mode], 0);
				}
			}
		}

		try
		{
			if (enable_multithread)
			{
				do_multithread(close_lambda, 5, "Closing");
			}
			else
			{
				repeat_lamda(close_lambda, 5, "Closing", timeout_remain);
			}
		}
		catch (...)
		{
			if (emitter_status[last_mode] != emitter_status[new_mode])
			{
				if (enable_multithread)
				{
					set_emitter(emitter_status[last_mode]);
					// set_laser_power(emitter_status[last_mode] ? laser_power : 0);
				}
				else
				{
					if (new_mode == CAPTURE || new_mode == LIVE)
					{
						// Capture mode or Live mode に移行するとき、タイムアウト
						if (timeout > 0)
						{
							const std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
							const int elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
							timeout_remain = timeout - elapsed;

							// ここで残り時間がなければタイムアウトにする。
							if (timeout_remain <= 0)
							{
								SPDLOG_ERROR("SetModeWithoutTestCapture Timeout");
								std::string msg;
								msg = fmt::format("SetModeWithoutTestCapture Timeout timeout={}", timeout);
								throwSymbolTimeoutErrors(msg);
							}
							set_emitter_sequential(emitter_status[new_mode], timeout_remain);
						}
						else
						{
							set_emitter_sequential(emitter_status[new_mode], 0);
						}
					}
					else
					{
						// Calibration modeに移行するときは、タイムアウト設定しない
						set_emitter_sequential(emitter_status[new_mode], 0);
					}
				}
			}
			throw;
		}

		mode = NONE;
		if (last_mode == LIVE)
		{
			if (enable_multithread)
			{
				set_depth_table(2, 30, 40);
			}
			else
			{
				if (timeout > 0)
				{
					const std::chrono::system_clock::time_point now = std::chrono::system_clock::now(); // 現在時刻
					const int elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count(); //経過時間(ミリ秒)
					// タイムアウトを決定
					timeout_remain = timeout - elapsed;

					// ここで残り時間がなければタイムアウトにする。
					if (timeout_remain <= 0)
					{
						SPDLOG_ERROR("SetModeWithoutTestCapture Timeout");
						std::string msg;
						msg = fmt::format("SetModeWithoutTestCapture Timeout timeout={}", timeout);
						throwSymbolTimeoutErrors(msg);
					}
					set_depth_table_sequential(2, 30, 40, timeout_remain);
				}
				else
				{
					set_depth_table_sequential(2, 30, 40, 0);
				}
			}
		}
	}
}

static void open_stream_after_setmode(std::vector<StreamConfig> &capture_config, const CAMERA_MODE &new_mode, const bool &enable_multithread, int &timeout_remain, const bool &enable_test_cap=true) {

	const int timeout = timeout_remain;
	std::chrono::system_clock::time_point start = std::chrono::system_clock::now();

	std::function<void(std::shared_ptr<CameraInfoWithData>)> open_lambda = [&](const std::shared_ptr<CameraInfoWithData> &info) -> void {
		info->depth_scale = SensorInfo(manager().get_rs2_device(info->serial()).first<rs2::depth_sensor>()).get_depth_scale(); // 撮影時の撮影スケールを取得

		SPDLOG_INFO("{} Opening", info->name());
		std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
		Streamer streamer(capture_config, manager().get_rs2_device(info->serial())); // ストリーム開始
		if (new_mode == CALIBRATE)
		{
#if 0 // get_instrinsicsのdepth or color判断用のログ
			SPDLOG_INFO("=====================================");
			SPDLOG_INFO("intrinsics[color] width:{}", streamer.get_intrinsics(rs2_stream::RS2_STREAM_COLOR).width);
			SPDLOG_INFO("intrinsics[color] height:{}", streamer.get_intrinsics(rs2_stream::RS2_STREAM_COLOR).height);
			SPDLOG_INFO("intrinsics[color] ppx:{}", streamer.get_intrinsics(rs2_stream::RS2_STREAM_COLOR).ppx);
			SPDLOG_INFO("intrinsics[color] ppy:{}", streamer.get_intrinsics(rs2_stream::RS2_STREAM_COLOR).ppy);
			SPDLOG_INFO("intrinsics[color] fx:{}", streamer.get_intrinsics(rs2_stream::RS2_STREAM_COLOR).fx);
			SPDLOG_INFO("intrinsics[color] fy:{}", streamer.get_intrinsics(rs2_stream::RS2_STREAM_COLOR).fy);
			SPDLOG_INFO("intrinsics[color] model:{}", streamer.get_intrinsics(rs2_stream::RS2_STREAM_COLOR).model);			
			for(auto i: streamer.get_intrinsics(rs2_stream::RS2_STREAM_COLOR).coeffs) {
				SPDLOG_INFO("intrinsics[color] coeffs:{}", i);
			}
			SPDLOG_INFO("-----------------------------------");
			SPDLOG_INFO("intrinsics[depth] width:{}", streamer.get_intrinsics(rs2_stream::RS2_STREAM_DEPTH).width);
			SPDLOG_INFO("intrinsics[depth] height:{}", streamer.get_intrinsics(rs2_stream::RS2_STREAM_DEPTH).height);
			SPDLOG_INFO("intrinsics[depth] ppx:{}", streamer.get_intrinsics(rs2_stream::RS2_STREAM_DEPTH).ppx);
			SPDLOG_INFO("intrinsics[depth] ppy:{}", streamer.get_intrinsics(rs2_stream::RS2_STREAM_DEPTH).ppy);
			SPDLOG_INFO("intrinsics[depth] fx:{}", streamer.get_intrinsics(rs2_stream::RS2_STREAM_DEPTH).fx);
			SPDLOG_INFO("intrinsics[depth] fy:{}", streamer.get_intrinsics(rs2_stream::RS2_STREAM_DEPTH).fy);
			SPDLOG_INFO("intrinsics[depth] model:{}", streamer.get_intrinsics(rs2_stream::RS2_STREAM_DEPTH).model);			
			for(auto i: streamer.get_intrinsics(rs2_stream::RS2_STREAM_DEPTH).coeffs) {
				SPDLOG_INFO("intrinsics[depth] coeffs:{}", i);
			}
			SPDLOG_INFO("=====================================");
#endif
			// COLORとどちらを使うべきか・・
			// info->intrinsics = streamer.get_intrinsics(rs2_stream::RS2_STREAM_COLOR);
			info->intrinsics = streamer.get_intrinsics(rs2_stream::RS2_STREAM_DEPTH);
		}
		streamer.start();
		SPDLOG_INFO("{} Opening Complete. took {}ms.", info->name(), std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count());

		{
			std::lock_guard<std::mutex> _(streamers_mutex);
			streamers[info] = streamer;
		}

		if (enable_test_cap)
		{
			const long long timeout = 8000;
			SPDLOG_INFO("{} test capturing", info->name());
			start = std::chrono::steady_clock::now();
			while (true)
			{
				try
				{
					// ストリーム開始後のテスト撮影(念のためここでは2枚撮っておく)
					streamer.get_frameset(2, timeout);
					break;
				}
				catch (const std::exception &e)
				{
					SPDLOG_WARN("{} Get Frameset Failed {}", info->name(), e.what());
					if (utility::contains(e.what(), "Streamer not started") || std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count() > timeout)
					{
						try
						{
							streamer.stop();
							streamer.close();
							{
								std::lock_guard<std::mutex> _(streamers_mutex);
								streamers.erase(streamers.find(info));
							}
						}
						catch (...)
						{
						}
						throw;
					}
				}
			}
			SPDLOG_INFO("{} test capturing Complete. took {}ms.", info->name(), std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count());
		}
	};

	if (new_mode == LIVE)
	{
		if (enable_multithread)
		{
			set_depth_table(2, 1000, 0);
		}
		else
		{
			if (timeout > 0)
			{
				std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
				int elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
				// タイムアウトを決定
				timeout_remain = timeout - elapsed;

				// ここで残り時間がなければタイムアウトにする。
				if (timeout_remain <= 0)
				{
					SPDLOG_ERROR("SetModeWithoutTestCapture Timeout");
					std::string msg;
					msg = fmt::format("SetModeWithoutTestCapture Timeout timeout={}", timeout);
					throwSymbolTimeoutErrors(msg);
				}
				set_depth_table_sequential(2, 1000, 0, timeout_remain);
			}
			else
			{
				set_depth_table_sequential(2, 1000, 0, 0);
			}
		}
	}

	if (emitter_status[NONE] != emitter_status[new_mode])
	{
		if (enable_multithread)
		{
			set_emitter(emitter_status[new_mode]);
			// set_laser_power(emitter_status[new_mode] ? laser_power : 0);
		}
		else
		{
			if (emitter_status[NONE] != emitter_status[new_mode])
			{
				if (new_mode == CAPTURE || new_mode == LIVE)
				{
					if (timeout > 0)
					{
						const std::chrono::system_clock::time_point now = std::chrono::system_clock::now(); // 現在時刻
						const int elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count(); //経過時間(ミリ秒)
						// タイムアウトを決定
						timeout_remain = timeout - elapsed;

						// ここで残り時間がなければタイムアウトにする。
						if (timeout_remain <= 0)
						{
							SPDLOG_ERROR("SetModeWithoutTestCapture Timeout");
							std::string msg;
							msg = fmt::format("SetModeWithoutTestCapture Timeout timeout={}", timeout);
							throwSymbolTimeoutErrors(msg);
						}
						set_emitter_sequential(emitter_status[new_mode], timeout_remain);
					}
					else
					{
						set_emitter_sequential(emitter_status[new_mode], 0);
					}
				}
				else
				{
					set_emitter_sequential(emitter_status[new_mode], 0);
				}
			}
		}
	}
	try
	{
		if (enable_multithread)
		{
			do_multithread(open_lambda, 5, "Opening");
		}
		else
		{
			repeat_lamda(open_lambda, 5, "Opening");
		}
	}
	catch (...)
	{
		if (new_mode == LIVE)
		{
			if (enable_multithread)
			{
				set_depth_table(2, 30, 40);
			}
			else
			{
				set_depth_table_sequential(2, 1000, 0, 0);
			}
		}
		if (emitter_status[NONE] != emitter_status[new_mode])
		{
			if (enable_multithread)
			{
				set_emitter(emitter_status[NONE]);
				// set_laser_power(emitter_status[new_mode] ? laser_power : 0);
			}
			else
			{
				set_emitter_sequential(emitter_status[NONE], 0);
				// set_laser_power(emitter_status[new_mode] ? laser_power : 0);
			}
		}
		throw;
	}
}

static void SetModeDetail(const CAMERA_MODE new_mode, const unsigned int timeout, const bool &enable_multithread, const bool &enable_test_cap)
{
	if (!initialized)
	{
		throw UnInitializedError("SetModeDetail");
	}

	if (mode == new_mode && isDummy == last_dummyMode)
	{
		return;
	}

	if (StreamingLive())
	{
		throw StreamingSafeGuardError("SetModeDetail");
	}

	// timeout設定・判定用
	std::chrono::system_clock::time_point start = std::chrono::system_clock::now();
	int timeout_remain = timeout;

	const CAMERA_MODE last_mode = mode;
	last_dummyMode = isDummy;

	if (enable_multithread)
	{
		wait_for_connections(manager(), enabled_infos);
	}
	else
	{
		wait_for_connections_sequential(manager(), enabled_infos, false);
	}

	close_stream_before_setmode(last_mode, new_mode, enable_multithread, timeout_remain);

	std::vector<StreamConfig> capture_config;

	// Modeに応じて解像度、フレームレートを変更
	// 各パラメータは、精度、動作の安定性に影響するため変更時は注意すること
	switch (new_mode)
	{
	case CAPTURE:
		// 使える解像度は以下URLを参照
		// https://www.mouser.com/pdfdocs/Intel_D400_Series_Datasheet.pdf
		capture_config.emplace_back(rs2_stream::RS2_STREAM_DEPTH, rs2_format::RS2_FORMAT_Z16, 6, 1280, 720);
		if (isDummy)
		{
			capture_config.emplace_back(rs2_stream::RS2_STREAM_COLOR, rs2_format::RS2_FORMAT_BGR8, 6, 1280, 720, 0, isDummy);
		}
		else
		{
			capture_config.emplace_back(rs2_stream::RS2_STREAM_COLOR, rs2_format::RS2_FORMAT_BGR8, 6, 1280, 720);
		}
		break;
	case LIVE:
		// 安定性対策（フレームレートを15fpsに設定）
		capture_config.emplace_back(rs2_stream::RS2_STREAM_DEPTH, rs2_format::RS2_FORMAT_Z16, 15, 424, 240);
		if (isDummy)
		{
			capture_config.emplace_back(rs2_stream::RS2_STREAM_COLOR, rs2_format::RS2_FORMAT_BGR8, 15, 424, 240, 0, isDummy);
		}
		else
		{
			capture_config.emplace_back(rs2_stream::RS2_STREAM_COLOR, rs2_format::RS2_FORMAT_BGR8, 15, 424, 240);
		}
		break;
	case CALIBRATE:
		capture_config.emplace_back(rs2_stream::RS2_STREAM_DEPTH, rs2_format::RS2_FORMAT_Z16, 6, 1280, 720);
		capture_config.emplace_back(rs2_stream::RS2_STREAM_COLOR, rs2_format::RS2_FORMAT_BGR8, 6, 1280, 720);
		break;
	case NONE:
	default:
		break;
	}

	if (capture_config.empty())
		return;

	if (enable_multithread)
	{
		wait_for_connections(manager(), enabled_infos);
	}
	else
	{
		wait_for_connections_sequential(manager(), enabled_infos, false);
	}

	open_stream_after_setmode(capture_config, new_mode, enable_multithread, timeout_remain, enable_test_cap);

	mode = new_mode;

	if (new_mode == CALIBRATE) // キャリブレーションは繊細なのでフレームが落ち着くまで待つ
	{
		std::this_thread::sleep_for(std::chrono::seconds(2));
	}
}

SYMBOLSCANNER_API void SetMode(const CAMERA_MODE mode, const unsigned int timeout, const bool &enable_multithread, const bool &enable_test_cap)
{
	// キャリブレーションモードの場合はカラーモードにする
	if (mode == CALIBRATE)
	{
		isDummy = false;
	}
	else
	{
		isDummy = true;
	}
	SetModeDetail(mode, timeout, enable_multithread, enable_test_cap);
}

SYMBOLSCANNER_API void SetModeWithoutTestCapture(const CAMERA_MODE mode, const int timeout)
{
	// キャリブレーションモードの場合はカラーモードにする
	if (mode == CALIBRATE)
	{
		isDummy = false;
	}
	else
	{
		isDummy = true;
	}
	const bool enable_multithread = false;
	const bool enable_test_cap = false;
	SetModeDetail(mode, timeout, enable_multithread, enable_test_cap);
}

SYMBOLSCANNER_API void SetModeSequential(const CAMERA_MODE mode)
{
	// キャリブレーションモードの場合はカラーモードにする
	if (mode == CALIBRATE)
	{
		isDummy = false;
	}
	else
	{
		isDummy = true;
	}
	const unsigned int timeout = 0;
	const bool enable_multithread = false;
	const bool enable_test_cap = true;
	SetModeDetail(mode, timeout, enable_multithread, enable_test_cap);
}

SYMBOLSCANNER_API void SetColorMode(const CAMERA_MODE mode)
{
	// カラーモードにする
	isDummy = false;
	const unsigned int timeout = 0;
	const bool enable_multithread = true;
	const bool enable_test_cap = true;
	SetModeDetail(mode, timeout, enable_multithread, enable_test_cap);
}

SYMBOLSCANNER_API void SetColorModeSequential(const CAMERA_MODE mode)
{
	// カラーモードにする
	isDummy = false;
	const unsigned int timeout = 0;
	const bool enable_multithread = false;
	const bool enable_test_cap = true;
	SetModeDetail(mode, timeout, enable_multithread, enable_test_cap);
}

SYMBOLSCANNER_API void SetColorModeWithoutTestCapture(const CAMERA_MODE mode, const int timeout)
{
	// カラーモードにする
	isDummy = false;
	const bool enable_multithread = false;
	const bool enable_test_cap = false;
	SetModeDetail(mode, timeout, enable_multithread, enable_test_cap);
}


SYMBOLSCANNER_API void SetMaskID(const std::vector<std::string> &enabled, const bool &enable_multithread)
{
	if (!initialized)
	{
		throw UnInitializedError("SetMaskID");
	}

	if (enable_multithread)
	{
		wait_for_connections(manager(), enabled_infos);
	}
	else
	{
		wait_for_connections_sequential(manager(), enabled_infos);
	}

	const CAMERA_MODE prev_mode = GetMode();

	std::vector<std::string> all_ids, before_enabled, before_disabled;
	for (const std::shared_ptr<CameraInfoWithData> &info : enabled_infos)
	{
		before_enabled.emplace_back(info->id());
	}
	for (const std::shared_ptr<CameraInfoWithData> &info : infos)
	{
		const std::vector<std::string>::iterator itr = std::find(before_enabled.begin(), before_enabled.end(), info->id());
		if (itr == before_enabled.end())
		{
			before_disabled.emplace_back(info->id());
		}
		all_ids.emplace_back(info->id());
	}

	std::vector<std::string> after_enabled, after_disabled;
	if (enabled.empty())
	{
		after_enabled = all_ids;
	}
	else
	{
		for (const std::string &id : all_ids)
		{
			const auto itr = std::find(enabled.begin(), enabled.end(), id);
			if (itr != enabled.end())
			{
				after_enabled.emplace_back(id);
			}
			else
			{
				after_disabled.emplace_back(id);
			}
		}
	}

	std::vector<std::string> newly_enabling, newly_disabling;
	for (const std::string &e : after_enabled)
	{
		const std::vector<std::string>::iterator itr = std::find(before_enabled.begin(), before_enabled.end(), e);
		if (itr == before_enabled.end())
		{
			newly_enabling.emplace_back(e);
		}
	}
	for (const std::string &d : after_disabled)
	{
		const std::vector<std::string>::iterator itr = std::find(before_disabled.begin(), before_disabled.end(), d);
		if (itr == before_disabled.end())
		{
			newly_disabling.emplace_back(d);
		}
	}

	if (newly_enabling.empty() && newly_disabling.empty())
	{
		return;
	}

	if (enable_multithread)
	{
		if (isDummy)
		{
			SetMode(NONE);
		}
		else
		{
			SetColorMode(NONE);
		}
	}
	else
	{
		if (isDummy)
		{
			SetModeWithoutTestCapture(NONE);
		}
		else
		{
			SetColorModeWithoutTestCapture(NONE);
		}
	}

	enabled_infos.clear();
	for (const std::string &id : after_enabled)
	{
		const std::vector<std::shared_ptr<CameraInfoWithData>>::iterator itr =
			std::find_if(infos.begin(), infos.end(), [id](const std::shared_ptr<CameraInfoWithData> &other) { return other->id() == id; });
		if (itr != infos.end())
		{
			enabled_infos.emplace_back(*itr);
		}
	}

	if (enable_multithread)
	{
		if (isDummy)
		{
			SetMode(prev_mode);
		}
		else
		{
			SetColorMode(prev_mode);
		}
	}
	else
	{
		if (isDummy)
		{
			SetModeWithoutTestCapture(prev_mode);
		}
		else
		{
			SetColorModeWithoutTestCapture(prev_mode);
		}
	}
}

SYMBOLSCANNER_API void SetMaskIDSequential(const std::vector<std::string> &enabled)
{
	const bool enable_multithread = false;
	SetMaskID(enabled, enable_multithread);
}

static void set_depth_table(const unsigned int retry_count, const unsigned int depth_unit, const unsigned int disparity_shift)
{
	if (!initialized)
	{
		throw UnInitializedError("set_depth_table");
	}
	if (StreamingLive())
	{
		throw StreamingSafeGuardError("set_depth_table");
	}
	wait_for_connections(manager(), enabled_infos);
	const CAMERA_MODE prev_mode = GetMode();
	if (isDummy)
	{
		SetMode(NONE);
	}
	else
	{
		SetColorMode(NONE);
	}

	SPDLOG_DEBUG("set_depth_table[depth_scale={0} disparity_shift={1}]", depth_unit, disparity_shift);

	std::function<void(std::shared_ptr<CameraInfoWithData>)> set_preset_lambda = [&](const std::shared_ptr<CameraInfoWithData> &info) -> void {
		SPDLOG_INFO("{0} setting depth table depth_scale={1} disparity_shift={2}", info->name(), depth_unit, disparity_shift);
		const rs2::device device = manager().get_rs2_device(info->serial());
		Preset::set_depth_table(device, depth_unit, disparity_shift);
		SPDLOG_INFO("{0} setting depth table depth_scale={1} disparity_shift={2} Complete", info->name(), depth_unit, disparity_shift);
	};
	set_sensor_value(sensor_type::depth, {{rs2_option::RS2_OPTION_DEPTH_UNITS, static_cast<float>(depth_unit) / 1000000}});
	do_multithread(set_preset_lambda, retry_count, "Set Preset", false, 90);
	if (isDummy)
	{
		SetMode(prev_mode);
	}
	else
	{
		SetColorMode(prev_mode);
	}
}

// static void set_depth_table_sequential(const unsigned int retry_count, const unsigned int depth_unit, const unsigned int disparity_shift)
static void set_depth_table_sequential(const unsigned int retry_count, const unsigned int depth_unit, const unsigned int disparity_shift, const int timeout)
{
	if (!initialized)
	{
		throw UnInitializedError("set_depth_table_sequential");
	}
	if (StreamingLive())
	{
		throw StreamingSafeGuardError("set_depth_table_sequential");
	}
	// timeout設定・判定用
	std::chrono::system_clock::time_point start, now;
	start = std::chrono::system_clock::now(); // 計測開始時間
	int timeout_remain = timeout;

	wait_for_connections_sequential(manager(), enabled_infos, false);
	const CAMERA_MODE prev_mode = GetMode();

	if (timeout > 0)
	{
		now = std::chrono::system_clock::now(); // 現在時刻
		int elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count(); //経過時間(ミリ秒)
		// タイムアウトを決定
		timeout_remain = timeout - elapsed;

		// ここで残り時間がなければタイムアウトにする
		if (timeout_remain <= 0)
		{
			SPDLOG_ERROR("set_depth_table_sequential Timeout");
			std::string msg;
			msg = fmt::format("set_depth_table_sequential Timeout timeout={}", timeout);
			throwSymbolTimeoutErrors(msg);
		}
		if (isDummy)
		{
			SetModeWithoutTestCapture(NONE, timeout_remain);
		}
		else
		{
			SetColorModeWithoutTestCapture(NONE, timeout_remain);
		}
	}
	else
	{
		if (isDummy)
		{
			SetModeWithoutTestCapture(NONE);
		}
		else
		{
			SetColorModeWithoutTestCapture(NONE);
		}
	}

	SPDLOG_DEBUG("set_depth_table[depth_scale={0} disparity_shift={1}]", depth_unit, disparity_shift);

	std::function<void(std::shared_ptr<CameraInfoWithData>)> set_preset_lambda = [&](const std::shared_ptr<CameraInfoWithData> &info) -> void {
		SPDLOG_INFO("{0} setting depth table depth_scale={1} disparity_shift={2}", info->name(), depth_unit, disparity_shift);
		const rs2::device device = manager().get_rs2_device(info->serial());
		Preset::set_depth_table(device, depth_unit, disparity_shift);
		SPDLOG_INFO("{0} setting depth table depth_scale={1} disparity_shift={2} Complete", info->name(), depth_unit, disparity_shift);
	};
	set_sensor_value_sequential(sensor_type::depth, {{rs2_option::RS2_OPTION_DEPTH_UNITS, static_cast<float>(depth_unit) / 1000000}}, timeout);
	// 指定された台数ずつモード変更
	for (unsigned int i = 0; i < enabled_infos.size();)
	{
		target_infos.clear();
		target_infos.shrink_to_fit();
		for (unsigned int j = 0; j < thread_num; j++)
		{
			if (i >= enabled_infos.size())
				break;

			target_infos.push_back(enabled_infos.at(i));
			i++;
		}
		if (timeout > 0)
		{
			now = std::chrono::system_clock::now(); // 現在時刻
			int elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count(); //経過時間(ミリ秒)
			// タイムアウトを決定
			timeout_remain = timeout - elapsed;

			// ここで残り時間がなければタイムアウトにする
			if (timeout_remain <= 0)
			{
				SPDLOG_ERROR("set_depth_table_sequential Timeout");
				std::string msg;
				msg = fmt::format("set_depth_table_sequential Timeout timeout={}", timeout);
				throwSymbolTimeoutErrors(msg);
			}
			do_singlethread(set_preset_lambda, retry_count, "Set Preset", false, timeout_remain / 1000);
		}
		else
		{
			do_singlethread(set_preset_lambda, retry_count, "Set Preset", false, 90);
		}
	}

	if (timeout > 0)
	{
		now = std::chrono::system_clock::now(); // 現在時刻
		int elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count(); //経過時間(ミリ秒)
		// タイムアウトを決定
		timeout_remain = timeout - elapsed;

		// ここで残り時間がなければタイムアウトにする
		if (timeout_remain <= 0)
		{
			SPDLOG_ERROR("set_depth_table_sequential Timeout");
			std::string msg;
			msg = fmt::format("set_depth_table_sequential Timeout timeout={}", timeout);
			throwSymbolTimeoutErrors(msg);
		}
		if (isDummy)
		{
			SetModeWithoutTestCapture(prev_mode, timeout_remain);
		}
		else
		{
			SetColorModeWithoutTestCapture(prev_mode, timeout_remain);
		}
	}
	else
	{
		if (isDummy)
		{
			SetModeWithoutTestCapture(prev_mode);
		}
		else
		{
			SetColorModeWithoutTestCapture(prev_mode);
		}
	}
}

static void set_sensor_value(const sensor_type sensor_type, const std::vector<std::pair<rs2_option, float>> &option_pairs)
{
	if (!initialized)
	{
		throw UnInitializedError("set_sensor_value");
	}
	if (StreamingLive())
	{
		throw StreamingSafeGuardError("set_sensor_value");
	}
	wait_for_connections(manager(), enabled_infos);
	const unsigned int max_threads = 20;
	std::mutex m;
	int dev_count = 0;
	std::function<void(std::shared_ptr<CameraInfoWithData>)> set_options_lambda = [&](const std::shared_ptr<CameraInfoWithData> &info) -> void {
		while (true)
		{
			{
				std::lock_guard<std::mutex> _(m);
				if (static_cast<unsigned>(dev_count) < max_threads)
				{
					dev_count += 1;
					break;
				}
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
		}
		try
		{
			const std::string sensor_type_name = sensor_type == sensor_type::depth ? "DEPTH" : "COLOR";
			rs2::sensor sensor;
			if (sensor_type::color == sensor_type)
			{
				sensor = utility::get_color_sensor(manager().get_rs2_device(info->serial()));
			}
			else if (sensor_type::depth == sensor_type)
			{
				sensor = manager().get_rs2_device(info->serial()).first<rs2::depth_sensor>().as<rs2::sensor>();
			}

			for (const auto &pair : option_pairs)
			{
				const rs2_option option = pair.first;
				const std::string option_name(rs2_option_to_string(pair.first));
				const float value = pair.second;
				SPDLOG_INFO("{} setting {} sensor options {} to {}", info->name(), sensor_type_name, option_name, value);
				const rs2::device device = manager().get_rs2_device(info->serial());
				unsigned int attempts = 0;
				while (true)
				{
					try
					{
						SensorInfo sensor_info(sensor);
						sensor_info.set_option(option, value);
					}
					catch (...)
					{
						attempts++;
						if (attempts > 3)
						{
							throw;
						}
						continue;
					}
					break;
				}
				SPDLOG_INFO("{} setting {} sensor options {} to {} Complete", info->name(), sensor_type_name, option_name, value);
			}
		}
		catch (...)
		{
			std::lock_guard<std::mutex> _(m);
			dev_count -= 1;
			throw;
		}
		{
			std::lock_guard<std::mutex> _(m);
			dev_count -= 1;
		}
	};
	do_multithread(set_options_lambda, 5, fmt::format("Set Sensor Option"));
}

static void set_sensor_value_sequential(const sensor_type sensor_type, const std::vector<std::pair<rs2_option, float>> &option_pairs, const int timeout)
{
	if (!initialized)
	{
		throw UnInitializedError("set_sensor_value_sequential");
	}
	if (StreamingLive())
	{
		throw StreamingSafeGuardError("set_sensor_value_sequential");
	}
	// timeout設定・判定用
	std::chrono::system_clock::time_point start, now;
	start = std::chrono::system_clock::now(); // 計測開始時間
	int timeout_remain = timeout;

	wait_for_connections_sequential(manager(), enabled_infos, false);
	const unsigned int max_threads = 20;
	std::mutex m;
	int dev_count = 0;
	std::function<void(std::shared_ptr<CameraInfoWithData>)> set_options_lambda = [&](const std::shared_ptr<CameraInfoWithData> &info) -> void {
		while (true)
		{
			{
				std::lock_guard<std::mutex> _(m);
				if (static_cast<unsigned>(dev_count) < max_threads)
				{
					dev_count += 1;
					break;
				}
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
		}
		try
		{
			const std::string sensor_type_name = sensor_type == sensor_type::depth ? "DEPTH" : "COLOR";
			rs2::sensor sensor;
			if (sensor_type::color == sensor_type)
			{
				sensor = utility::get_color_sensor(manager().get_rs2_device(info->serial()));
			}
			else if (sensor_type::depth == sensor_type)
			{
				sensor = manager().get_rs2_device(info->serial()).first<rs2::depth_sensor>().as<rs2::sensor>();
			}

			for (const auto &pair : option_pairs)
			{
				const rs2_option option = pair.first;
				const std::string option_name(rs2_option_to_string(pair.first));
				const float value = pair.second;
				SPDLOG_INFO("{} setting {} sensor options {} to {}", info->name(), sensor_type_name, option_name, value);
				const rs2::device device = manager().get_rs2_device(info->serial());
				unsigned int attempts = 0;
				while (true)
				{
					try
					{
						SensorInfo sensor_info(sensor);
						sensor_info.set_option(option, value);
					}
					catch (...)
					{
						attempts++;
						if (attempts > 3)
						{
							throw;
						}
						continue;
					}
					break;
				}
				SPDLOG_INFO("{} setting {} sensor options {} to {} Complete", info->name(), sensor_type_name, option_name, value);
			}
		}
		catch (...)
		{
			std::lock_guard<std::mutex> _(m);
			dev_count -= 1;
			throw;
		}
		{
			std::lock_guard<std::mutex> _(m);
			dev_count -= 1;
		}
	};
	// 指定された台数ずつモード変更
	for (unsigned int i = 0; i < enabled_infos.size();)
	{
		target_infos.clear();
		target_infos.shrink_to_fit();
		for (unsigned int j = 0; j < thread_num; j++)
		{
			if (i >= enabled_infos.size())
				break;

			target_infos.push_back(enabled_infos.at(i));
			i++;
		}
		if (timeout > 0)
		{
			now = std::chrono::system_clock::now(); // 現在時刻
			int elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count(); //経過時間(秒)
			// タイムアウトを決定
			timeout_remain = timeout - elapsed;

			// ここで残り時間がなければタイムアウトにする
			if (timeout_remain <= 0)
			{
				SPDLOG_ERROR("set_sensor_value_sequential Timeout");
				std::string msg;
				msg = fmt::format("set_sensor_value_sequential Timeout timeout={}", timeout);
				throwSymbolTimeoutErrors(msg);
			}

			do_singlethread(set_options_lambda, 5, fmt::format("Set Sensor Option, timeout"), false, timeout_remain / 1000);
		}
		else
		{
			do_singlethread(set_options_lambda, 5, fmt::format("Set Sensor Option, timeout"));
		}
	}
}

static void set_laser_power(const float value) { set_sensor_value(sensor_type::depth, {std::make_pair(rs2_option::RS2_OPTION_LASER_POWER, value)}); }

static void set_emitter(const bool enable) { set_sensor_value(sensor_type::depth, {std::make_pair(rs2_option::RS2_OPTION_EMITTER_ENABLED, enable ? 1.f : 0.f)}); }

// static void set_emitter_sequential(const bool enable) { set_sensor_value_sequential(sensor_type::depth, {std::make_pair(rs2_option::RS2_OPTION_EMITTER_ENABLED, enable ? 1.f : 0.f)}); }
static void set_emitter_sequential(const bool enable, const int timeout)
{
	set_sensor_value_sequential(sensor_type::depth, {std::make_pair(rs2_option::RS2_OPTION_EMITTER_ENABLED, enable ? 1.f : 0.f)}, timeout);
}

static void set_exposure(const float value) { set_sensor_value(sensor_type::depth, {std::make_pair(rs2_option::RS2_OPTION_EXPOSURE, value)}); }

static void set_auto_exposure(const bool enable) { set_sensor_value(sensor_type::depth, {std::make_pair(rs2_option::RS2_OPTION_ENABLE_AUTO_EXPOSURE, enable ? 1.f : 0.f)}); }

static void set_auto_white_balance(const bool enable) { set_sensor_value(sensor_type::depth, {std::make_pair(rs2_option::RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, enable ? 1.f : 0.f)}); }

SYMBOLSCANNER_API void SetPreset(const unsigned int retry_count, const char *preset_json_path, const bool &enable_multithread)
{
	if (!initialized)
	{
		throw UnInitializedError("SetPreset");
	}
	if (StreamingLive())
	{
		throw StreamingSafeGuardError("SetPreset");
	}
	if (enable_multithread)
	{
		wait_for_connections(manager(), enabled_infos);
	}
	else
	{
		wait_for_connections_sequential(manager(), enabled_infos);
	}

	const CAMERA_MODE prev_mode = GetMode();

	if (enable_multithread)
	{
		if (isDummy)
		{
			SetMode(NONE);
		}
		else
		{
			SetColorMode(NONE);
		}
	}
	else
	{
		if (isDummy)
		{
			SetModeWithoutTestCapture(NONE);
		}
		else
		{
			SetColorModeWithoutTestCapture(NONE);
		}
	}

	const std::string preset_path = preset_json_path;
	SPDLOG_DEBUG("preset_path[{0}]", preset_path);

	const std::experimental::filesystem::path path(preset_path);

	if (path.extension().string() == ".json" && std::experimental::filesystem::exists(path) && !std::experimental::filesystem::is_directory(path))
	{
		std::ifstream j(path);
		const std::string json((std::istreambuf_iterator<char>(j)), std::istreambuf_iterator<char>());
		j.close();
		std::function<void(std::shared_ptr<CameraInfoWithData>)> set_preset_lambda = [&](const std::shared_ptr<CameraInfoWithData> &info) -> void {
			SPDLOG_INFO("{} setting preset", info->name());
			const rs2::device device = manager().get_rs2_device(info->serial());
			Preset::set_preset(device, json);
			SPDLOG_INFO("{} setting preset Complete", info->name());
		};

		if (enable_multithread)
		{
			do_multithread(set_preset_lambda, retry_count, "Set Preset", false, 90);
			if (isDummy)
			{
				SetMode(prev_mode);
			}
			else
			{
				SetColorMode(prev_mode);
			}
		}
		else
		{
			repeat_lamda(set_preset_lambda, retry_count, "Set Preset", false, 90);
			if (isDummy)
			{
				SetModeWithoutTestCapture(prev_mode);
			}
			else
			{
				SetColorModeWithoutTestCapture(prev_mode);
			}
		}
	}
	else
	{
		if (enable_multithread)
		{
			if (isDummy)
			{
				SetMode(prev_mode);
			}
			else
			{
				SetColorMode(prev_mode);
			}
		}
		else
		{
			if (isDummy)
			{
				SetModeWithoutTestCapture(prev_mode);
			}
			else
			{
				SetColorModeWithoutTestCapture(prev_mode);
			}
		}
		throw std::runtime_error("Invalid Json Path at API Call SetPreset");
	}
}

SYMBOLSCANNER_API void SetPresetSequential(const unsigned int retry_count, const char *preset_json_path)
{
	const bool enable_multithread = false;
	SetPreset(retry_count, preset_json_path, enable_multithread);
}

SYMBOLSCANNER_API const std::string GetPreset(const unsigned int retry_count)
{
	if (!initialized)
	{
		throw UnInitializedError("GetPreset");
	}
	if (StreamingLive())
	{
		throw StreamingSafeGuardError("GetPreset");
	}
	wait_for_connections_sequential(manager(), enabled_infos);
	const CAMERA_MODE prev_mode = GetMode();
	if (isDummy)
	{
		SetModeWithoutTestCapture(NONE);
	}
	else
	{
		SetColorModeWithoutTestCapture(NONE);
	}

	std::ostringstream ret_preset_data_oss;
	std::function<void(std::shared_ptr<CameraInfoWithData>)> get_preset_lambda = [&](const std::shared_ptr<CameraInfoWithData> &info) -> void {
		SPDLOG_INFO("{} getting preset", info->name());
		const rs2::device device = manager().get_rs2_device(info->serial());
		std::string preset_data = Preset::get_preset(device);
		ret_preset_data_oss << info->name() << '\n' + preset_data << '\n';
		SPDLOG_INFO("{} getting preset Complete", info->name());
	};

	// 1台数ずつプリセット取得
	for (size_t i = 0; i < enabled_infos.size(); i++)
	{
		target_infos.clear();
		target_infos.shrink_to_fit();
		target_infos.push_back(enabled_infos.at(i));
		do_singlethread(get_preset_lambda, retry_count, "Get Preset", false, 90);
	}

	if (isDummy)
	{
		SetModeWithoutTestCapture(prev_mode);
	}
	else
	{
		SetColorModeWithoutTestCapture(prev_mode);
	}
	ret_preset_data_oss << "=========";
	return ret_preset_data_oss.str();
}

#define rad2deg(a) ((a) / M_PI * 180.0) /* rad を deg に換算するマクロ関数 */

// GeneratePointCloudから呼び出される
// 法線方向による点群除去
std::shared_ptr<open3d::geometry::PointCloud> CutFloor(std::shared_ptr<open3d::geometry::PointCloud> floor_pc)
{
	Eigen::Vector3d v(0, 1, 0);
	std::shared_ptr<open3d::geometry::PointCloud> pc(new open3d::geometry::PointCloud());;

	// 法線をチェック
	for (size_t i = 0; i < floor_pc->normals_.size(); i++)
	{
		Eigen::Vector3d normal = floor_pc->normals_.at(i);
		double deg_y = rad2deg(std::acos(normal.dot(v)));

		// 採用する法線の角度は30度以下
		if (deg_y < 30.0f)
		{
			continue;
		}
		pc->points_.push_back(floor_pc->points_.at(i));
		pc->normals_.push_back(floor_pc->normals_.at(i));

		if (floor_pc->HasColors())
		{
			pc->colors_.push_back(floor_pc->colors_.at(i));
		}
	}
	return pc;
}

SYMBOLSCANNER_API void GenerateCalibration(void)
{
	if (!initialized)
	{
		throw UnInitializedError("GenerateCalibration");
	}
	if (frame_mode != CALIBRATE)
	{
		throw InvalidFrameModeError("GenerateCalibration", {CALIBRATE});
	}
	calibrated = false;

	const utility::Range3Df board_range{{0, 0}, {0, 0}, {0, 0}};

	std::mutex data_mutex_corner;
	std::map<std::string, std::pair<std::vector<int>, open3d::geometry::PointCloud>> corner_pointclouds;
	std::mutex data_mutex_raw_pc;
	std::map<std::string, PointCloud> raw_pointclouds;
	std::mutex data_mutex;
	std::map<std::shared_ptr<CameraInfoWithData>, CalibrationData> calibration_data;
	std::function<void(std::shared_ptr<CameraInfoWithData>)> generate_calibration_lambda = [&](const std::shared_ptr<CameraInfoWithData> &info) -> void {
		CalibrationFrame frame(info->frameset.get_filtered(), calibration_board.get_dictionary(info->side()), calibration_board.get_board(info->side()));
		const std::tuple<double, std::pair<std::vector<int>, open3d::geometry::PointCloud>> result = frame.get_corner_pointcloud();
		const double plane_error = std::get<0>(result);
		const std::pair<std::vector<int>, open3d::geometry::PointCloud> &corner_pointcloud = std::get<1>(result);
		const open3d::geometry::PointCloud &pc = corner_pointcloud.second;
		const size_t corner_count = pc.points_.size();
		SPDLOG_DEBUG("{} Corner Pointcloud Plane Error: {:< 10.6}[mm], Corner Count: {:<}", info->name(), plane_error, corner_count);
		{
			std::lock_guard<std::mutex> _(data_mutex_corner);
			corner_pointclouds[info->id()] = corner_pointcloud;
		}
		const CalibrationData calibration(calibration_board, info->side(), info->frameset);
		{
			std::lock_guard<std::mutex> _(data_mutex);
			calibration_data[info] = calibration;
		}

		if (info->level() == 1) // use floor data for floor alignment
		{
			const PointCloud untransformed(CalibrationData::Identity(frame), frame, info->depth_scale, board_range);
			{
				std::lock_guard<std::mutex> _(data_mutex_raw_pc);
				raw_pointclouds[info->id()] = untransformed;
			}
		}
	};

	do_multithread(generate_calibration_lambda, 1, "Generate Calibration");
	if (calibration_data.size() != enabled_infos.size())
	{
		throw std::runtime_error("Generate Calibration Failed");
	}

	Alignment::dump_evaluation(calibration_board, corner_pointclouds, calibration_data);

	// const CornerVisualizer v(corner_pointclouds);
	// v.show(calibration_data, "No align");

	Alignment::align_to_each_left(corner_pointclouds, calibration_data);
	Alignment::dump_evaluation(calibration_board, corner_pointclouds, calibration_data);
	// v.show(calibration_data, "align_to_each_left");

	Alignment::align_to_front_side_each_level(calibration_board, corner_pointclouds, calibration_data);
	Alignment::dump_evaluation(calibration_board, corner_pointclouds, calibration_data);
	// v.show(calibration_data, "align_to_front_side_each_level");

	// skip height align because this code accumulte height error which from camera distortion
	// Alignment::align_to_center_front_back_together(corner_pointclouds, calibration_data);
	// Alignment::dump_evaluation(calibration_board, corner_pointclouds, calibration_data);
	// // v.show(calibration_data, "align_to_center_front_back_together");

	Alignment::realign_all(calibration_board, corner_pointclouds, calibration_data);
	Alignment::dump_evaluation(calibration_board, corner_pointclouds, calibration_data);
	// v.show(calibration_data, "realign_all");

	Alignment::align_to_floor(calibration_board, corner_pointclouds, calibration_data, raw_pointclouds);
	Alignment::dump_evaluation(calibration_board, corner_pointclouds, calibration_data);
	// v.show(calibration_data, "align_to_floor");

	for (auto &&itr : calibration_data)
	{
		itr.first->calibration = itr.second;
	}

	calibrated = true;
	calibrate_time = utility::get_date_time();
	if (pc_valid)
	{
		capture_time = calibrate_time;
		pc_valid = false;
	}

	corner_pointclouds.clear();
	raw_pointclouds.clear();
	calibration_data.clear();
}

SYMBOLSCANNER_API void SaveCalibrationToDir(const char *calibration_data_dir)
{
	if (!initialized)
	{
		throw UnInitializedError("SaveCalibration");
	}
	if (!IsCalibrated())
	{
		throw UnCalibratedError("SaveCalibration");
	}

	const std::string calibration_dir = calibration_data_dir;
	SPDLOG_DEBUG("calibration_dir[{0}]", calibration_dir);

	const std::experimental::filesystem::path parent_dir(calibration_dir);

	const std::vector<std::experimental::filesystem::path> save_paths({parent_dir / fmt::format("calibration_{}", calibrate_time), parent_dir / "master"});
	for (const std::experimental::filesystem::path &path : save_paths)
	{
		if (std::experimental::filesystem::exists(path) && std::experimental::filesystem::is_directory(path))
		{
			std::experimental::filesystem::remove_all(path);
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
			std::experimental::filesystem::create_directories(path);
		}
	}
	std::function<void(std::shared_ptr<CameraInfoWithData>)> calibrate_saver_lambda = [&](const std::shared_ptr<CameraInfoWithData> &info) -> void {
		// キャリブレーションデータを保存する
		const std::string xml_path_child = fmt::format("{}_{}.xml", info->id(), info->serial());
		const std::string bef_path_child = fmt::format("{}_{}_color.png", info->id(), info->serial());
		const std::string aft_path_child = fmt::format("{}_{}_after.png", info->id(), info->serial());
		const std::string dpt_path_child = fmt::format("{}_{}_depth.png", info->id(), info->serial());
		for (const std::experimental::filesystem::path &path : save_paths)
		{
			const std::experimental::filesystem::path xml_path = path / xml_path_child;
			const std::experimental::filesystem::path bef_path = path / bef_path_child;
			const std::experimental::filesystem::path aft_path = path / aft_path_child;
			const std::experimental::filesystem::path dpt_path = path / dpt_path_child;
			info->calibration.save_xml(xml_path);
			info->calibration.save_images(bef_path, aft_path, dpt_path);
		}
	};
	do_multithread(calibrate_saver_lambda, 2, "Calibration Save");
}

SYMBOLSCANNER_API void SaveCalibration(void) { SaveCalibrationToDir(calibration_path_.string().c_str()); }

SYMBOLSCANNER_API void LoadCalibrationFromDir(const char *calibration_data_dir)
{
	if (!initialized)
	{
		throw UnInitializedError("LoadCalibration");
	}
	calibrated = false;

	std::mutex m1;
	std::map<std::shared_ptr<CameraInfoWithData>, CalibrationData> data;

	const std::string calibration_dir = calibration_data_dir;
	SPDLOG_DEBUG("calibration_dir[{0}]", calibration_dir);

	const std::experimental::filesystem::path dir = std::experimental::filesystem::path(calibration_dir);

	if (std::experimental::filesystem::is_directory(dir))
	{
		std::function<void(std::shared_ptr<CameraInfoWithData>)> loader_lambda = [&](const std::shared_ptr<CameraInfoWithData> &info) -> void {
			const std::experimental::filesystem::path xml_path = dir / fmt::format("{}_{}.xml", info->id(), info->serial());
			const std::experimental::filesystem::path bef_path = dir / fmt::format("{}_{}_before.png", info->id(), info->serial());
			const std::experimental::filesystem::path aft_path = dir / fmt::format("{}_{}_after.png", info->id(), info->serial());
			const std::experimental::filesystem::path dpt_path = dir / fmt::format("{}_{}_depth.png", info->id(), info->serial());
			if (exists(bef_path) && exists(aft_path) && exists(dpt_path))
			{
				std::lock_guard<std::mutex> _(m1);
				data[info] = CalibrationData(xml_path, bef_path, aft_path, dpt_path);
			}
			else
			{
				std::lock_guard<std::mutex> _(m1);
				data[info] = CalibrationData(xml_path);
			}
		};
		do_multithread(loader_lambda, 1, "Load Calibration");
		if (data.size() == enabled_infos.size())
		{
			for (std::map<std::shared_ptr<CameraInfoWithData>, CalibrationData>::iterator itr = data.begin(); itr != data.end(); ++itr)
			{
				itr->first->calibration = itr->second;
			}
			calibrated = true;
			calibrate_time = utility::get_date_time();
			pc_valid = false;
		}
	}
	else
	{
		throw std::runtime_error("Invalid Calibration Data Path at API Call LoadCalibration");
	}
}

SYMBOLSCANNER_API void LoadCalibration(void) { LoadCalibrationFromDir((calibration_path_ / "master").string().c_str()); }

SYMBOLSCANNER_API bool IsCalibrated(void) { return calibrated; }

SYMBOLSCANNER_API void AssertConnection(const bool &enable_multithread)
{
	if (!initialized)
	{
		throw UnInitializedError("AssertConnection");
	}
	SetPriorityClass(GetCurrentProcess(), ABOVE_NORMAL_PRIORITY_CLASS);
	if (enable_multithread)
	{
		wait_for_connections(manager(), enabled_infos);
	}
	else
	{
		wait_for_connections_sequential(manager(), enabled_infos);
	}
}

SYMBOLSCANNER_API void AssertConnectionSequential()
{
	const bool enable_multithread = false;
	AssertConnection(enable_multithread);
}

SYMBOLSCANNER_API void Capture(const unsigned int retry_count, const CaptureParams params)
{
	if (!initialized)
	{
		throw UnInitializedError("Capture");
	}
	if (!(mode == CAPTURE || mode == CALIBRATE))
	{
		throw InvalidModeError("Capture", {CAPTURE, CALIBRATE});
	}

	SetPriorityClass(GetCurrentProcess(), HIGH_PRIORITY_CLASS);
	try
	{
		wait_for_connections(manager(), enabled_infos);
		const unsigned int max_threads = 20;
		std::mutex m;
		int dev_count = 0;
		std::mutex framesets_mutex;
		std::map<std::shared_ptr<CameraInfoWithData>, Frameset> framesets;

		std::function<void(std::shared_ptr<CameraInfoWithData>)> capture_lambda = [&](const std::shared_ptr<CameraInfoWithData> &info) -> void {
			while (true)
			{
				{
					std::lock_guard<std::mutex> _(m);
					if (static_cast<unsigned>(dev_count) < max_threads)
					{
						dev_count += 1;
						break;
					}
				}
				std::this_thread::sleep_for(std::chrono::milliseconds(500));
			}
			try
			{
				SPDLOG_INFO("{} capturing", info->name());
				const std::chrono::time_point<std::chrono::steady_clock> start = std::chrono::steady_clock::now();
				Streamer streamer = streamers[info];
				while (streamer)
				{
					try
					{
						const Frameset frameset = streamer.get_capture_frameset(static_cast<int>(params.frame_count), static_cast<int>(params.timeout));
						{
							std::lock_guard<std::mutex> _(framesets_mutex);
							framesets[info] = frameset;
						}
						break;
					}
					catch (const std::exception &e)
					{
						SPDLOG_WARN("{} Get Frameset Failed [{}]", info->name(), e.what());
						if (utility::contains(e.what(), "Streamer not started") ||
							static_cast<unsigned long long>(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count()) > params.timeout)
						{
							try
							{
								streamer.stop();
								streamer.close();
							}
							catch (...)
							{
							}
							throw;
						}
					}
					catch (...)
					{
						SPDLOG_WARN("{} Get Frameset Failed Unknown Failure", info->name());
						throw;
					}
				}
				SPDLOG_INFO("{} capturing Complete. took {}ms.", info->name(), std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count());
			}
			catch (...)
			{
				std::lock_guard<std::mutex> _(m);
				dev_count -= 1;
				throw;
			}
			{
				std::lock_guard<std::mutex> _(m);
				dev_count -= 1;
			}
		};
		// multithreaderでのタイムアウトによるカメラリセットをやめる
		isCapture_or_Live = true;

		// タイムアウトは渡されたパラメータのみを使う
		do_multithread(capture_lambda, retry_count, "Capture Frames", true, params.timeout / 1000);

		// フラグを戻す
		isCapture_or_Live = false;

		// Copy to global framesets only when all successful
		if (framesets.size() != enabled_infos.size())
		{
			SPDLOG_ERROR("Capture Frames Failed");
			throw std::runtime_error("Capture Frames Failed"); // Probably not going to be throw due to being thrown in do_multithread
		}
		frame_mode = mode;
		pc_valid = false;
		for (std::map<std::shared_ptr<CameraInfoWithData>, Frameset>::iterator itr = framesets.begin(); itr != framesets.end(); ++itr)
		{
			itr->first->frameset = itr->second;
		}
		capture_time = utility::get_date_time();
	}
	catch (...)
	{
		SetPriorityClass(GetCurrentProcess(), ABOVE_NORMAL_PRIORITY_CLASS);
		throw;
	}
	SetPriorityClass(GetCurrentProcess(), ABOVE_NORMAL_PRIORITY_CLASS);
}

// キャリブレーション用（1台ずつ最高解像度で撮影）
SYMBOLSCANNER_API void CaptureSequential(const unsigned int retry_count, const CaptureParams params)
{
	if (!initialized)
	{
		throw UnInitializedError("CaptureSequential");
	}
	if (!(mode == CAPTURE || mode == CALIBRATE))
	{
		throw InvalidModeError("CaptureSequential", {CAPTURE, CALIBRATE});
	}

	SetPriorityClass(GetCurrentProcess(), HIGH_PRIORITY_CLASS);
	try
	{
		wait_for_connections_sequential(manager(), enabled_infos);
		const unsigned int max_threads = 20;
		std::mutex m;
		int dev_count = 0;
		std::mutex framesets_mutex;
		std::map<std::shared_ptr<CameraInfoWithData>, Frameset> framesets;
		std::function<void(std::shared_ptr<CameraInfoWithData>)> capture_lambda = [&](const std::shared_ptr<CameraInfoWithData> &info) -> void {
			while (true)
			{
				{
					std::lock_guard<std::mutex> _(m);
					if (static_cast<unsigned>(dev_count) < max_threads)
					{
						dev_count += 1;
						break;
					}
				}
				std::this_thread::sleep_for(std::chrono::milliseconds(500));
			}
			try
			{
				SPDLOG_INFO("{} capturing", info->name());
				const std::chrono::time_point<std::chrono::steady_clock> start = std::chrono::steady_clock::now();
				Streamer streamer = streamers[info];
				while (streamer)
				{
					try
					{
						const Frameset frameset = streamer.get_capture_frameset(static_cast<int>(params.frame_count), static_cast<int>(params.timeout));
						{
							std::lock_guard<std::mutex> _(framesets_mutex);
							framesets[info] = frameset;
						}
						break;
					}
					catch (const std::exception &e)
					{
						SPDLOG_WARN("{} Get Frameset Failed [{}]", info->name(), e.what());
						if (utility::contains(e.what(), "Streamer not started") ||
							static_cast<unsigned long long>(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count()) > params.timeout)
						{
							try
							{
								streamer.stop();
								streamer.close();
							}
							catch (...)
							{
							}
							throw;
						}
					}
					catch (...)
					{
						SPDLOG_WARN("{} Get Frameset Failed Unknown Failure", info->name());
						throw;
					}
				}
				SPDLOG_INFO("{} capturing Complete. took {}ms.", info->name(), std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count());
			}
			catch (...)
			{
				std::lock_guard<std::mutex> _(m);
				dev_count -= 1;
				throw;
			}
			{
				std::lock_guard<std::mutex> _(m);
				dev_count -= 1;
			}
		};

		// multithreaderでのタイムアウトによるカメラリセットをやめる
		isCapture_or_Live = true;
		
		// 1台ずつ撮影
		for (int i = 0; i < enabled_infos.size(); i++)
		{
			target_infos.clear();
			target_infos.shrink_to_fit();
			target_infos.push_back(enabled_infos.at(i));
			do_singlethread(capture_lambda, retry_count, "Capture Frames", true, params.timeout / 1000);
		}

		// フラグを戻す
		isCapture_or_Live = false;

		// Copy to global framesets only when all successful
		if (framesets.size() != enabled_infos.size())
		{
			SPDLOG_ERROR("Capture Frames Failed");
			throw std::runtime_error("Capture Frames Failed"); // Probably not going to be throw due to being thrown in do_multithread
		}
		frame_mode = mode;
		pc_valid = false;
		for (std::map<std::shared_ptr<CameraInfoWithData>, Frameset>::iterator itr = framesets.begin(); itr != framesets.end(); ++itr)
		{
			itr->first->frameset = itr->second;
		}
		capture_time = utility::get_date_time();
	}
	catch (...)
	{
		SetPriorityClass(GetCurrentProcess(), ABOVE_NORMAL_PRIORITY_CLASS);
		throw;
	}
	SetPriorityClass(GetCurrentProcess(), ABOVE_NORMAL_PRIORITY_CLASS);
}

SYMBOLSCANNER_API void SaveRawCapture(const char *save_raw_dir)
{
	if (!initialized)
	{
		throw UnInitializedError("SaveRawCapture");
	}
	if (!(frame_mode == CAPTURE || frame_mode == CALIBRATE))
	{
		throw InvalidFrameModeError("SaveRawCapture", {CAPTURE, CALIBRATE});
	}
	SetPriorityClass(GetCurrentProcess(), ABOVE_NORMAL_PRIORITY_CLASS);

	const unsigned int max_threads = 6;
	std::mutex m;
	unsigned int dev_count = 0;

	const std::string raw_dir = save_raw_dir;
	SPDLOG_DEBUG("raw_dir[{0}]", raw_dir);

	const std::experimental::filesystem::path dir = std::experimental::filesystem::path(raw_dir);

	SPDLOG_INFO("save_raw_dir[{0}", dir.string());

	std::experimental::filesystem::create_directories(dir);
	std::function<void(std::shared_ptr<CameraInfoWithData>)> capture_saver_lambda = [&](const std::shared_ptr<CameraInfoWithData> &info) -> void {
		while (true)
		{
			{
				std::lock_guard<std::mutex> _(m);
				if (static_cast<unsigned>(dev_count) < max_threads)
				{
					dev_count += 1;
					break;
				}
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
		}
		try
		{
			SPDLOG_INFO("{} saving raw frames", info->name());
			const size_t size = info->frameset.size();
			for (size_t i = 0; i < size; ++i)
			{
				const Frame frame = info->frameset[static_cast<int>(i)]; //!< 撮影で取得したi番目のフレーム
				if (frame_mode == CALIBRATE || frame_mode == CAPTURE)
				{
					cv::imwrite((dir / fmt::format("{}_depth_raw_{}.png", info->id(), i)).string(), frame.get_image(rs2_stream::RS2_STREAM_DEPTH), compression_params);
					cv::imwrite((dir / fmt::format("{}_color_raw_{}.png", info->id(), i)).string(), frame.get_image(rs2_stream::RS2_STREAM_COLOR), compression_params);
					// depthのモノクロ画像を保存する
					cv::imwrite((dir / fmt::format("{}_depth_data_raw_{}.png", info->id(), i)).string(), frame.get_image(rs2_stream::RS2_STREAM_DEPTH, false, true), compression_params);
				}
			}
			SPDLOG_INFO("{} saving raw frames Complete", info->name());
		}
		catch (...)
		{
			std::lock_guard<std::mutex> _(m);
			dev_count -= 1;
			throw;
		}
		{
			std::lock_guard<std::mutex> _(m);
			dev_count -= 1;
		}
	};
	do_multithread(capture_saver_lambda, 2, "Saving Raw Frames");
}

SYMBOLSCANNER_API std::shared_ptr<open3d::geometry::PointCloud> GeneratePointCloud(const struct FilterParams params)
{
	if (pc_valid)
	{
		return pc;
	}
	if (!initialized)
	{
		throw UnInitializedError("GeneratePointCloud");
	}
	if (frame_mode != CAPTURE)
	{
		throw InvalidFrameModeError("GeneratePointCloud", {CAPTURE});
	}
	if (!IsCalibrated())
	{
		throw UnCalibratedError("GeneratePointCloud");
	}

	SPDLOG_DEBUG("cut_floor[{0}]", cut_floor);
	SPDLOG_DEBUG("offset_y[{0}]", offset_y);
	SPDLOG_DEBUG("foot_range[{0}]", foot_range);

	SetPriorityClass(GetCurrentProcess(), ABOVE_NORMAL_PRIORITY_CLASS);
	const unsigned int max_threads = 6;
	std::mutex m;
	unsigned int dev_count = 0;

	std::mutex point_clouds_mutex;
	std::map<std::shared_ptr<CameraInfoWithData>, Frame> filtered_frames;
	std::map<std::shared_ptr<CameraInfoWithData>, PointCloud> filtered_pointcloud;
	std::map<std::shared_ptr<CameraInfoWithData>, PointCloud> filtered_org_pointcloud; // オリジナルの点群
	std::vector<PointCloud> point_clouds;
	std::function<void(std::shared_ptr<CameraInfoWithData>)> pc_generate_lambda = [&](const std::shared_ptr<CameraInfoWithData> &info) -> void {
		while (true)
		{
			{
				std::lock_guard<std::mutex> _(m);
				if (static_cast<unsigned>(dev_count) < max_threads)
				{
					dev_count += 1;
					break;
				}
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
		}
		try
		{
			SPDLOG_INFO("{} filtering frames", info->name());
			const Frame filtered = info->frameset.get_filtered();
			SPDLOG_INFO("{} filtering frames Complete", info->name());

			SPDLOG_INFO("{} generating pointcloud", info->name());
			PointCloud pointcloud = PointCloud(info->calibration, filtered, info->depth_scale, range);

			// オリジナルサイズの点群を残す（ファイル保存時にオリジナルサイズのものを保存する場合への対応）
			PointCloud org_pointcloud;
			org_pointcloud.ptr()->points_ = pointcloud.ptr()->points_;
			org_pointcloud.ptr()->colors_ = pointcloud.ptr()->colors_;
			org_pointcloud.ptr()->normals_ = pointcloud.ptr()->normals_;

			for (const _noise_filter_params &noise_filter_param : params.noise_filter_params)
			{
				SPDLOG_INFO("{0} generating pointcloud noise_nb_neighbors={1}", info->name(), noise_filter_param.noise_nb_neighbors);
				SPDLOG_INFO("{0} generating pointcloud noist_std_ratio={1}", info->name(), noise_filter_param.noist_std_ratio);

				pointcloud.removeStatisticalOutliers(noise_filter_param.noise_nb_neighbors != 0, noise_filter_param.noist_std_ratio == 0.0f);
			}
			// Nomura
			// ダウンサンプリングをノイズ除去後に移動
			SPDLOG_INFO("{0} generating pointcloud down_sample_voxel_size={1}", info->name(), params.down_sample_voxel_size);
			if (params.down_sample_voxel_size > 0.00000001f)
			{
				pointcloud.downSample(params.down_sample_voxel_size); // ダウンサンプル(パラメータは適当)
			}

			SPDLOG_INFO("{0} generating pointcloud normal_knn={1}", info->name(), params.normal_knn);
			if (params.normal_knn > 0)
			{
				pointcloud.estimateNormals(params.normal_knn); // 法線推定(パラメータは適当)
				pointcloud.orientNormals(info->calibration.get_camera_coordinate());
			}

			// 床位置の強制オフセット(指定された位置をy=0にする)
			if (offset_y != 0.0f)
			{
				const Eigen::Vector3d offset{0, -offset_y, 0};
				for (auto &&point : pointcloud.ptr()->points_)
				{
					point += offset;
				}
				// オリジナルサイズの点群にもオフセット処理を行う
				for (auto &&point : org_pointcloud.ptr()->points_)
				{
					point += offset;
				}
			}

			pointcloud.scaleUp(1000); // m -> mm
			org_pointcloud.scaleUp(1000); // m -> mm(オリジナルの点群)

			SPDLOG_INFO("{} generating pointcloud Complete", info->name());

			{
				std::lock_guard<std::mutex> _(point_clouds_mutex);
				filtered_frames[info] = filtered;
				filtered_pointcloud[info] = pointcloud;
				filtered_org_pointcloud[info] = org_pointcloud; // 生点群を保存する時のためにvectorに追加
				point_clouds.emplace_back(pointcloud); // 後でカメラ全台で合成するために点群をvectorに追加
			}
		}
		catch (...)
		{
			std::lock_guard<std::mutex> _(m);
			dev_count -= 1;
			throw;
		}
		{
			std::lock_guard<std::mutex> _(m);
			dev_count -= 1;
		}
	};
	do_multithread(pc_generate_lambda, 2, "Generating PointCloud", false, 90);

	if (point_clouds.size() != enabled_infos.size())
	{
		SPDLOG_ERROR("Generating PointCloud Failed");
		throw std::runtime_error("Generating PointCloud Failed"); // Probably not going to be throw due to being thrown in do_multithread
	}
	pc = PointCloud(point_clouds);

	// 法線方向による床ノイズ除去
	if (cut_floor)
	{
		std::shared_ptr<open3d::geometry::PointCloud> o3d_pc = pc;

		// Crop（床付近だけを残す）
		std::shared_ptr<open3d::geometry::PointCloud> cropped_cloud(new open3d::geometry::PointCloud());
		std::shared_ptr<open3d::geometry::PointCloud> fixed_cloud (new open3d::geometry::PointCloud());

		// 法線による判定範囲上限(mm) 0～max_yまでを判定範囲とする。
		double max_y = 8.0f;

		// 足付近のノイズ除去範囲(y方向5cm)
		double foot_range_y = 50.0f;
		double foot_range_x_max = foot_range * 1000;
		double foot_range_x_min = foot_range_x_max * -1;
		
		// 床付近のみ残す
		for (size_t i = 0; i < o3d_pc->points_.size(); i++)
		{
			if (o3d_pc->points_.at(i)[1] >= 0 && o3d_pc->points_.at(i)[1] <= max_y)	// 床付近
			{ // y=0～8mm
				// 足付近のfoot_range外の点群は無視する(範囲は +-foot_range)
				if (foot_range > 0)
				{
					if (o3d_pc->points_.at(i)[0] > foot_range_x_max || o3d_pc->points_.at(i)[0] < foot_range_x_min)
					{
						continue;
					}
				}
				cropped_cloud->points_.push_back(o3d_pc->points_.at(i));
				cropped_cloud->normals_.push_back(o3d_pc->normals_.at(i));
				if (o3d_pc->HasColors())
				{
					cropped_cloud->colors_.push_back(o3d_pc->colors_.at(i));
				}
			}
			else if (o3d_pc->points_.at(i)[1] > max_y)	// 床付近以外
			{
				// 床上5cm以下の足付近のfoot_range外の点群は無視する(範囲は +-foot_range)
				if (foot_range > 0)
				{
					if (o3d_pc->points_.at(i)[1] <= foot_range_y)
					{
						if (o3d_pc->points_.at(i)[0] > foot_range_x_max || o3d_pc->points_.at(i)[0] < foot_range_x_min)
						{
							continue;
						}
					}
				}
				fixed_cloud->points_.push_back(o3d_pc->points_.at(i));
				fixed_cloud->normals_.push_back(o3d_pc->normals_.at(i));
				if (o3d_pc->HasColors())
				{
					fixed_cloud->colors_.push_back(o3d_pc->colors_.at(i));
				}
			}
		}

		// 法線方向による点群除去
		std::shared_ptr<open3d::geometry::PointCloud> valid_cloud = CutFloor(cropped_cloud);

		// 補正後のpoitcloudを作成
		*fixed_cloud += *valid_cloud;	// 床付近以外

		// PointCloudを再生成
		pc = PointCloud(fixed_cloud);
	}

	// DBSCANによる最大クラスタの抽出
	if (params.extract_radius >= DBL_EPSILON || params.extract_knn > 0)
	{
		// m->mm スケールアップ済みなので範囲は1000倍
		pc.extractLargestConnectedComponent(params.extract_radius * 1000, params.extract_knn);
	}
	f_filter = filtered_frames;
	pc_filter = filtered_pointcloud;
	org_pc_filter = filtered_org_pointcloud; // 生点群を保存する時のためにvectorに追加
	pc_valid = true;
	generate_time = capture_time;
	return pc;
}

SYMBOLSCANNER_API void SavePointCloud(const char *save_pc_dir, const bool save_color_data)
{
	if (!initialized)
	{
		throw UnInitializedError("SavePointCloud");
	}
	if (!pc_valid)
	{
		throw InvalidPointCloudError("SavePointCloud");
	}
	SetPriorityClass(GetCurrentProcess(), ABOVE_NORMAL_PRIORITY_CLASS);
	const unsigned int max_threads = 6;
	std::mutex mm;
	unsigned int dev_count = 0;

	const std::string pc_dir = save_pc_dir;
	SPDLOG_DEBUG("pc_dir[{0}]", pc_dir);

	std::mutex m;
	const std::experimental::filesystem::path dir = std::experimental::filesystem::path(pc_dir);

	SPDLOG_DEBUG("pc_dir[{0}]", dir.string());

	std::experimental::filesystem::create_directories(dir);
	std::function<void(std::shared_ptr<CameraInfoWithData>)> filtered_saver_lambda = [&](const std::shared_ptr<CameraInfoWithData> &info) -> void {
		while (true)
		{
			{
				std::lock_guard<std::mutex> _(mm);
				if (static_cast<unsigned>(dev_count) < max_threads)
				{
					dev_count += 1;
					break;
				}
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
		}
		try
		{
			const Frame filtered = f_filter[info];
			SPDLOG_INFO("{} saving filtered frames", info->name());
			cv::imwrite((dir / fmt::format("{}_aligned_depth_depth_filtered.png", info->id())).string(), filtered.get_image(rs2_stream::RS2_STREAM_DEPTH), compression_params);
			cv::imwrite((dir / fmt::format("{}_aligned_depth_color_filtered.png", info->id())).string(), filtered.get_image(rs2_stream::RS2_STREAM_COLOR), compression_params);
			// depthのモノクロ画像を保存する
			cv::imwrite((dir / fmt::format("{}_aligned_depth_data_filtered.png", info->id())).string(), filtered.get_image(rs2_stream::RS2_STREAM_DEPTH, false, true), compression_params);
			SPDLOG_INFO("{} saving filtered frames Complete", info->name());
		}
		catch (...)
		{
			std::lock_guard<std::mutex> _(mm);
			dev_count -= 1;
			throw;
		}
		{
			std::lock_guard<std::mutex> _(mm);
			dev_count -= 1;
		}
		const PointCloud pointcloud = pc_filter[info];
		{
			std::lock_guard<std::mutex> _(m);
			open3d::geometry::PointCloud output_pc = *pointcloud.ptr();
			if (!save_color_data)
			{
				output_pc.colors_.clear();
			}
			SPDLOG_INFO("{} saving pointcloud with {} points", info->name(), output_pc.points_.size());
			open3d::io::WritePointCloudToPLY((dir / fmt::format("pointcloud_{}.ply", info->id())).string(), output_pc, false, false);
			SPDLOG_INFO("{} saving pointcloud Complete", info->name());
		}
	};
	do_multithread(filtered_saver_lambda, 2, "Saving Filtered Frames and Pointcloud", false, 90);

#if 0
    {
        // ASTINAデバッグ用
        open3d::utility::SetVerbosityLevel(open3d::utility::VerbosityLevel::VerboseError);
        SPDLOG_INFO("Saving Combined Pointcloud");
        open3d::io::WritePointCloudToPLY((dir / "pointcloud_combined.ply").string(), pc, false, false);
        SPDLOG_INFO("Saving Combined Pointcloud Complete");
    }
#endif

	SPDLOG_INFO("Generating+Saving Mesh from Pointcloud");

	open3d::utility::SetVerbosityLevel(open3d::utility::VerbosityLevel::VerboseInfo);
}

SYMBOLSCANNER_API void SaveOrgPointCloud(const char *save_pc_dir, const bool save_color_data, const int normal_knn)
{
	if (!initialized)
	{
		throw UnInitializedError("SavePointCloud");
	}
	if (!pc_valid)
	{
		throw InvalidPointCloudError("SavePointCloud");
	}
	SetPriorityClass(GetCurrentProcess(), ABOVE_NORMAL_PRIORITY_CLASS);
	const unsigned int max_threads = 6;
	std::mutex mm;
	unsigned int dev_count = 0;

	const std::string pc_dir = save_pc_dir;
	SPDLOG_DEBUG("pc_dir[{0}]", pc_dir);

	std::mutex m;
	const std::experimental::filesystem::path dir = std::experimental::filesystem::path(pc_dir);

	SPDLOG_DEBUG("pc_dir[{0}]", dir.string());

	std::experimental::filesystem::create_directories(dir);
	std::function<void(std::shared_ptr<CameraInfoWithData>)> filtered_saver_lambda = [&](const std::shared_ptr<CameraInfoWithData> &info) -> void {
		while (true)
		{
			{
				std::lock_guard<std::mutex> _(mm);
				if (static_cast<unsigned>(dev_count) < max_threads)
				{
					dev_count += 1;
					break;
				}
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
		}
		try
		{
			const Frame filtered = f_filter[info];
			SPDLOG_INFO("{} saving filtered frames", info->name());
			cv::imwrite((dir / fmt::format("{}_aligned_depth_depth_filtered.png", info->id())).string(), filtered.get_image(rs2_stream::RS2_STREAM_DEPTH), compression_params);
			cv::imwrite((dir / fmt::format("{}_aligned_depth_color_filtered.png", info->id())).string(), filtered.get_image(rs2_stream::RS2_STREAM_COLOR), compression_params);
			// depthのモノクロ画像を保存する
			cv::imwrite((dir / fmt::format("{}_aligned_depth_data_filtered.png", info->id())).string(), filtered.get_image(rs2_stream::RS2_STREAM_DEPTH, false, true), compression_params);
			SPDLOG_INFO("{} saving filtered frames Complete", info->name());
		}
		catch (...)
		{
			std::lock_guard<std::mutex> _(mm);
			dev_count -= 1;
			throw;
		}
		{
			std::lock_guard<std::mutex> _(mm);
			dev_count -= 1;
		}
		PointCloud pointcloud = org_pc_filter[info];
		{
			std::lock_guard<std::mutex> _(m);

			// オリジナルサイズの点群はここで法線を付ける
			SPDLOG_INFO("{0} saving original_pointcloud normal_knn={1}", info->name(), normal_knn);
			if (normal_knn > 0 && !pointcloud.ptr()->HasNormals())
			{
				pointcloud.estimateNormals(normal_knn); // 法線推定(パラメータは適当)
				pointcloud.orientNormals(info->calibration.get_camera_coordinate());
			}

			open3d::geometry::PointCloud output_pc = *pointcloud.ptr();
			if (!save_color_data)
			{
				output_pc.colors_.clear();
			}
			SPDLOG_INFO("{} saving pointcloud with {} points", info->name(), output_pc.points_.size());
			open3d::io::WritePointCloudToPLY((dir / fmt::format("pointcloud_{}.ply", info->id())).string(), output_pc, false, false);
			SPDLOG_INFO("{} saving pointcloud Complete", info->name());
		}
	};
	do_multithread(filtered_saver_lambda, 2, "Saving Filtered Frames and Pointcloud", false, 90);
	SPDLOG_INFO("Generating+Saving Mesh from Pointcloud");

	open3d::utility::SetVerbosityLevel(open3d::utility::VerbosityLevel::VerboseInfo);
}

// StartLiveSeauenceからのFilterParamがlive_lambdaから読めないのでグローバルに持っておく
static std::vector<_noise_filter_params> live_noise_filter_params;
static double live_down_sample_voxel_size = 0.0;
static int live_normal_knn = 0;

SYMBOLSCANNER_API void StartLive(const struct FilterParams filter_params, std::function<void(const char *, const std::shared_ptr<open3d::geometry::PointCloud>)> &callback,
								 const bool &enable_multithread)
{
	if (!initialized)
	{
		throw UnInitializedError("StartLive");
	}
	if (mode != LIVE)
	{
		throw InvalidModeError("StartLive", {LIVE});
	}
	if (!IsCalibrated())
	{
		throw UnCalibratedError("StartLive");
	}
	SetPriorityClass(GetCurrentProcess(), ABOVE_NORMAL_PRIORITY_CLASS);

	if (enable_multithread)
	{
		wait_for_connections(manager(), enabled_infos);
	}
	else
	{
		wait_for_connections_sequential(manager(), enabled_infos);
	}

	// filter_paramsをグローバルに保存（現状使っていないが・・）
	live_noise_filter_params.clear();
	live_noise_filter_params.shrink_to_fit();

	for (size_t i = 0; i < filter_params.noise_filter_params.size(); i++)
	{
		_noise_filter_params nf_params;
		nf_params.noise_nb_neighbors = filter_params.noise_filter_params.at(i).noise_nb_neighbors;
		nf_params.noist_std_ratio = filter_params.noise_filter_params.at(i).noist_std_ratio;
		live_noise_filter_params.push_back(nf_params);
	}
	live_down_sample_voxel_size = filter_params.down_sample_voxel_size;
	live_normal_knn = filter_params.normal_knn;
	
	streaming_live = true;
	live_errors.reset();
	live_errors = nullptr;
	try
	{
		if (live_thread != nullptr)
		{
			if (live_thread->joinable())
			{
				live_thread->join();
			}
		}
	}
	catch (...)
	{
		SPDLOG_ERROR("Error at live _thread->join()");
	}
	live_thread = std::make_shared<std::thread>([&]() {
		std::function<void(std::shared_ptr<CameraInfoWithData>)> live_lambda = [&](const std::shared_ptr<CameraInfoWithData> &info) -> void {
			try
			{
				Streamer streamer = streamers[info];
				Frameset live_frameset;
				const auto &capture = [&](const int count, const long long timeout) {
					const std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
					// Live中は成功するかStopLiveまで繰り返す
					while (streaming_live)
					{
						try
						{
							live_frameset = streamer.get_capture_frameset(count, timeout);
							break;
						}
						catch (const std::runtime_error& e)
						{
							SPDLOG_ERROR("[{}] Get Frameset runtime_error [{}]", info->name(), e.what());
							throw e;	// Exceptionを発生させる
						}
						catch (const std::exception &e)
						{
							SPDLOG_WARN("[{}] Get Frameset Failed [{}]", info->name(), e.what());
							throw e;	// Exceptionを発生させる
						}
						catch(...)
						{
							// その他のエラーもExceptionを発生させる。
							SPDLOG_ERROR("[{}] Get Frameset threw exceptionr [???]", info->name());
							throw std::runtime_error(fmt::format("Get Frameset threw exception [???]").c_str());
						}
					}
				};
				while (streaming_live)
				{
					try	// capture及びlive_frameset.get_alignedのExceptionでも処理を継続する
					{
						capture(1, 8000); // 画像を一枚とる
						const Frame frame = live_frameset.get_aligned();
						{
							PointCloud pointcloud(info->calibration, frame, info->depth_scale, range); // 点群を生成する
							
							if (!live_noise_filter_params.empty())
							{
								for (const _noise_filter_params &noise_filter_param : live_noise_filter_params)
								{
									SPDLOG_DEBUG("{0} noise_nb_neighbors={1}", info->name(), noise_filter_param.noise_nb_neighbors);
									SPDLOG_DEBUG("{0} noist_std_ratio={1}", info->name(), noise_filter_param.noist_std_ratio);
									pointcloud.removeStatisticalOutliers(noise_filter_param.noise_nb_neighbors != 0, noise_filter_param.noist_std_ratio == 0.0f);
								}
							}

							// Nomura
							// ダウンサンプリングをノイズ除去後に移動
							if (live_down_sample_voxel_size > 0.00000001)
							{
								SPDLOG_DEBUG("{0} down_sample_voxel_size={1}", info->name(), live_down_sample_voxel_size);
								pointcloud.downSample(live_down_sample_voxel_size); // ダウンサンプル(パラメータは適当)
							}

							if (live_normal_knn > 0)
							{
								pointcloud.estimateNormals(live_normal_knn); // 法線推定(パラメータは適当)
								pointcloud.orientNormals(info->calibration.get_camera_coordinate());
							}

							try
							{
								callback(info->id().c_str(), pointcloud.ptr());
							}
							catch (const std::runtime_error& e)
							{
								SPDLOG_ERROR("[{}] Live sreaming runtime_error [{}] -> Retry", info->name(), e.what());
								// 15fps分の待ちを入れる
								std::this_thread::sleep_for(std::chrono::milliseconds(66));
							}
							catch (const std::exception &e)
							{
								// Exceptionはthrowせずに処理を継続する
								//throw std::runtime_error(fmt::format("Callback threw exception [{}]", e.what()).c_str());
								SPDLOG_ERROR(fmt::format("[{}] Error at live sreaming [{}] -> Retry", info->name(), e.what()));
								// 15fps分の待ちを入れる
								std::this_thread::sleep_for(std::chrono::milliseconds(66));
							}
							catch (...)
							{
								// Exceptionはthrowせずに処理を継続する
								//throw std::runtime_error(fmt::format("Callback threw exception [???]").c_str());
								SPDLOG_ERROR("[{}] Error at live sreaming [???] -> Retry", info->name());
								SPDLOG_ERROR(fmt::format("Error at live sreaming [{}] -> Retry", info->name()));
								// 15fps分の待ちを入れる
								std::this_thread::sleep_for(std::chrono::milliseconds(66));
							}
						}
					}
					catch (const error::UnRecoverableTimeoutError &e)	// capture->streamer.get_capture_frameset()で発生する
					{
						SPDLOG_ERROR("[{}] UnRecoverableTimeoutError Error [{0}] -> Retry", info->name(), e.what());
						// 15fps分の待ちを入れる
						std::this_thread::sleep_for(std::chrono::milliseconds(66));
					}
					catch (const BadStateError &e)	// live_frameset.get_aligned()で発生する
					{
						SPDLOG_ERROR("[{}] BadStateError Error [{}] -> Retry", info->name(), e.what());
						// 15fps分の待ちを入れる
						std::this_thread::sleep_for(std::chrono::milliseconds(66));
					}
					catch (const std::runtime_error &e)
					{
						SPDLOG_ERROR("[{}] runtime_error [{}] -> Retry", info->name(), e.what());
						// 15fps分の待ちを入れる
						std::this_thread::sleep_for(std::chrono::milliseconds(66));
					}
					catch (const std::exception& e)	// capture->streamer.get_capture_frameset()で発生する
					{
						SPDLOG_ERROR("[{}] Error at live sreaming [{}] -> Retry", info->name(), e.what());
						// 15fps分の待ちを入れる
						std::this_thread::sleep_for(std::chrono::milliseconds(66));
					}
					catch(...)
					{
						SPDLOG_ERROR("[{}] Error at live sreaming[Other error] -> Retry", info->name());
						// 15fps分の待ちを入れる
						std::this_thread::sleep_for(std::chrono::milliseconds(66));
					}
				}
			}
			catch (const std::runtime_error& e)
			{
				// Exceptionはthrowしなくなったので、ここは通らないはず
				SPDLOG_ERROR("[{}] runtime_error [{}]", info->name(), e.what());
			}
			catch (const std::exception &e)
			{
				// Exceptionはthrowしなくなったので、ここは通らないはず
				// Live中は成功するまで繰り返す
				// SPDLOG_ERROR(fmt::format("Error at live sreaming [{}]", e.what()));
				SPDLOG_ERROR(fmt::format("[{}] Error at live sreaming [{}] -> Retry", info->name(), e.what()));
				// 以下をコメントアウト
				/*
				streaming_live = false;
				throw;
				*/
			}
			catch (...)
			{
				SPDLOG_ERROR("[{}] Error at live sreaming[Other error]", info->name());
				// Exceptionはthrowしなくなったので、ここは通らないはず
				// Nomura:Live中に画像取得に失敗した場合？
				// この場合でも次回には取得可能(カメラの異常ではない)ようなので、ここではthrowしない
				// 実機ベースで10分程度連続してLive表示しているとこの現象が発生し、Liveが停止するが、
				// 以下をコメントアウトすると継続してLive表示が可能となる。
				// streaming_live = false;
				// throw;
			}
		};
		SetPriorityClass(GetCurrentProcess(), NORMAL_PRIORITY_CLASS);

		// multithreaderでのタイムアウトによるカメラリセットをやめる
		isCapture_or_Live = true;

		MultiThreader result(enabled_infos, live_lambda, std::chrono::seconds(0)); // Disable Timeout

		// フラグを戻す
		isCapture_or_Live = false;

		try
		{
			handle_exception(manager(), result.errors);
		}
		catch (const SymbolErrors &e)
		{
			live_errors = std::make_shared<SymbolErrors>(e.errors());
			SPDLOG_ERROR("MultiThreader SymbolErrors [{}]", GetSymbolErrors());
		}
		catch (...)
		{
			SPDLOG_ERROR("MultiThreader Error [???]");
		}
	});
}

SYMBOLSCANNER_API void StartLiveSequential(const struct FilterParams filter_params, std::function<void(const char *, const std::shared_ptr<open3d::geometry::PointCloud>)> &callback)
{
	const bool enable_multithread = false;
	StartLive(filter_params, callback, enable_multithread);
}

SYMBOLSCANNER_API void StopLive(const bool &enable_multithread)
{
	if (!initialized)
	{
		throw UnInitializedError("StopLive");
	}
	if (StreamingLive())
	{
		SetPriorityClass(GetCurrentProcess(), ABOVE_NORMAL_PRIORITY_CLASS);
		if (enable_multithread)
		{
			wait_for_connections(manager(), enabled_infos);
		}
		else
		{
			wait_for_connections_sequential(manager(), enabled_infos);
		}

		streaming_live = false;
		if (live_errors != nullptr)
		{
			throw SymbolErrors(live_errors->errors());
		}
	}
	try
	{
		if (live_thread != nullptr)
		{
			if (live_thread->joinable())
			{
				live_thread->join();
				live_thread.reset();
			}
		}
	}
	catch (...)
	{
	}
}

SYMBOLSCANNER_API void StopLiveSequential(void)
{
	const bool enable_multithread = false;
	StopLive(enable_multithread);
}

SYMBOLSCANNER_API bool StreamingLive(void) { return streaming_live; }

SYMBOLSCANNER_API void Terminate(const bool &enable_multithread)
{
	if (initialized)
	{
		if (enable_multithread)
		{
			StopLive();
			if (isDummy)
			{
				SetMode(NONE);
			}
			else
			{
				SetColorMode(NONE);
			}
			manager().stop();
		}
		else
		{
			StopLiveSequential();
			if (isDummy)
			{
				SetModeWithoutTestCapture(NONE);
			}
			else
			{
				SetColorModeWithoutTestCapture(NONE);
			}
			manager().stop();
		}
	}
}

SYMBOLSCANNER_API void TerminateSequential(void)
{
	const bool enable_multithread = false;
	Terminate(enable_multithread);
}

SYMBOLSCANNER_API void ForceResetCameras(const bool &enable_multithread)
{
	if (!initialized)
	{
		throw UnInitializedError("ForceResetCameras");
	}
	if (StreamingLive())
	{
		try
		{
			if (enable_multithread)
			{
				StopLive();
			}
			else
			{
				StopLiveSequential();
			}
		}
		catch (...)
		{
		}
	}
	try
	{
		if (enable_multithread)
		{
			if (isDummy)
			{
				SetMode(NONE);
			}
			else
			{
				SetColorMode(NONE);
			}
		} else {
			if (isDummy)
			{
				SetModeWithoutTestCapture(NONE);
			}
			else
			{
				SetColorModeWithoutTestCapture(NONE);
			}
		}
	}
	catch (...)
	{
	}
	mode = NONE;
	manager().reset();
}

SYMBOLSCANNER_API void ForceResetCamerasSequential(void)
{
	const bool enable_multithread = false;
	ForceResetCameras(enable_multithread);
}

std::string get_mode_names(const std::vector<CAMERA_MODE> &valid_modes)
{
	std::ostringstream msg;
	bool comma = false;
	for (const CAMERA_MODE &mode : valid_modes)
	{
		if (comma)
		{
			msg << ", ";
		}
		switch (mode)
		{
		case CAMERA_MODE::NONE:
			msg << "NONE";
			break;
		case CAMERA_MODE::CALIBRATE:
			msg << "CALIBRATE";
			break;
		case CAMERA_MODE::CAPTURE:
			msg << "CAPTURE";
			break;
		case CAMERA_MODE::LIVE:
			msg << "LIVE";
			break;
		}
		comma = true;
	}
	return msg.str();
}

SYMBOLSCANNER_API UnInitializedError::UnInitializedError(const std::string &routine_name) : runtime_error(fmt::format("API Uninitialized at API Call {}", routine_name)) {}

SYMBOLSCANNER_API UnCalibratedError::UnCalibratedError(const std::string &routine_name) : std::runtime_error(fmt::format("Uncalibrated at API Call {}", routine_name)) {}

SYMBOLSCANNER_API InvalidModeError::InvalidModeError(const std::string &routine_name, const std::vector<CAMERA_MODE> &valid_modes)
	: std::runtime_error(fmt::format("Invalid \"Mode\" [{}] at API Call {}; Valid modes are [{}]", get_mode_names({mode}), routine_name, get_mode_names(valid_modes)).c_str())
{
}

SYMBOLSCANNER_API InvalidFrameModeError::InvalidFrameModeError(const std::string &routine_name, const std::vector<CAMERA_MODE> &valid_modes)
	: std::runtime_error(fmt::format("Invalid \"Frame Mode\" [{}] at API Call {}; Valid modes are [{}]", get_mode_names({frame_mode}), routine_name, get_mode_names(valid_modes)).c_str())
{
}

SYMBOLSCANNER_API StreamingSafeGuardError::StreamingSafeGuardError(const std::string &routine_name) : std::runtime_error(fmt::format("Unsafe API Call {}; Live Frame Streaming is on", routine_name)) {}

SYMBOLSCANNER_API InvalidPointCloudError::InvalidPointCloudError(const std::string &routine_name)
	: std::runtime_error(fmt::format("Un-Generated or Invalid Point Cloud data at API Call {}", routine_name))
{
}

SYMBOLSCANNER_API SymbolErrors::SymbolErrors(const std::vector<std::runtime_error> &errors) : runtime_error(fmt::format("{} errors", errors.size()))
{
	errors_ = errors;

	// Electron Wrapper用にメッセージをcharとして保存しておく
	memset(symbol_error_message, 0, sizeof(symbol_error_message));
	std::ostringstream error_string_stream;
	for (const std::runtime_error &cam_error: errors)
	{
		error_string_stream << cam_error.what() << "\n";
	}
	sprintf_s(symbol_error_message, "%s", error_string_stream.str().c_str());
}

std::vector<std::runtime_error> SYMBOLSCANNER_API SymbolErrors::errors(void) const { return errors_; }

// タイムアウトをSymbolErrorsから分離
SYMBOLSCANNER_API SymbolTimeoutErrors::SymbolTimeoutErrors(const std::vector<std::runtime_error> &errors) : runtime_error(fmt::format("{} errors", errors.size()))
{
	errors_ = errors;

	// Electron Wrapper用にメッセージをcharとして保存しておく
	memset(symbol_error_message, 0, sizeof(symbol_error_message));
	std::ostringstream error_string_stream;
	for (const std::runtime_error& cam_error : errors)
	{
		error_string_stream << cam_error.what() << "\n";
	}
	sprintf_s(symbol_error_message, "%s", error_string_stream.str().c_str());
}

std::vector<std::runtime_error> SYMBOLSCANNER_API SymbolTimeoutErrors::errors(void) const { return errors_; }

// Nomura 追加
// symbolErrors.errorsの取得
SYMBOLSCANNER_API char *GetSymbolErrors() { return symbol_error_message; }
