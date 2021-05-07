#include <filesystem>
#include <functional>
#include <iostream>
#include <string>

#include <fmt/format.h>
#include <spdlog/spdlog.h>

#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"

#include <Open3D/Open3D.h>
#include <opencv2/opencv.hpp>

#undef SYMBOLSCANNER_EXPORTS
#include <SymbolScanner.h>
#include <conio.h>

// #define ENABLE_SYMBOL_WRAPPER

void printCommandHelp();
std::vector<std::string> split_string(const std::string &string, const std::string &delimeter);
std::string get_date_time();

int main()
{
	// ログ出力をUTF-8に設定
	SetConsoleOutputCP(65001);

	if (!std::experimental::filesystem::exists("./log"))
	{
		std::experimental::filesystem::create_directories("./log");
	}

	std::vector<spdlog::sink_ptr> sinks;
	sinks.push_back(std::make_shared<spdlog::sinks::stdout_color_sink_mt>());
	sinks.push_back(std::make_shared<spdlog::sinks::rotating_file_sink_mt>(fmt::format("./log/sdk_test_log_{}.txt", get_date_time()), 1048576 * 5, 3));
	const std::shared_ptr<spdlog::logger> combined_logger = std::make_shared<spdlog::logger>("SDK_TEST", begin(sinks), end(sinks));

	spdlog::register_logger(combined_logger);
	spdlog::set_default_logger(combined_logger);
	spdlog::set_level(spdlog::level::trace);
	spdlog::set_pattern("[%5t]%+");

	SPDLOG_INFO("Start sdk_test app");

	const struct InitializeParams init_params
	{
		{
			8, // square_count_x
			35, // square_count_y
			0.05f, // square_length
			0.03f, // marker_length
			6, // marker_size
			0.003f // thickness
		},
			"./config.json", // config path
			"./preset.json", // preset path
			{
				{-0.5f, 0.5f}, {FLT_EPSILON, 2.00f}, {-0.6f, 0.4f} // range x, y, z
			},
			false, // cut_floor
			0.0f, // offset_y
			0.3f, // foot_range
			"c:/symb/calibration",  // calibration_dir
			"c:/symb/log", // log_dir
			true,  // enable_set_preset
			5 // thread_num
	};

	const FilterParams filter_params = {0.0f, {{400, 2.5f}, {200, 2.5f}, {100, 2.5f}}, 30, 0.02, 20};

	try
	{
#ifdef ENABLE_SYMBOL_WRAPPER
		InitializeWithoutPreset(init_params);
		SetModeSequential(CAMERA_MODE::CAPTURE);
		SetPresetSequential(5, init_params.preset_json_path.c_str());
		SetModeSequential(CAMERA_MODE::CAPTURE);
#else
		Initialize(init_params);
#endif
	}
	catch (...)
	{
		SPDLOG_ERROR("Initialization Failed. Exiting.");
		return EXIT_FAILURE;
	}

	bool quit = false;

	while (!quit)
	{
		try
		{
			printCommandHelp();
			std::cout << "> ";
			std::string input;
			std::getline(std::cin, input);
			const std::vector<std::string> command = split_string(input, " ");
			if (command.empty())
				continue;
			if (command[0] == "mode" && command.size() >= 2)
			{
				if (command[1] == "calibrate")
				{
					SetMaskID({});
#ifdef ENABLE_SYMBOL_WRAPPER
					SetModeWithoutTestCapture(CALIBRATE, 30);
#else
					SetMode(CALIBRATE);
#endif
				}
				else if (command[1] == "capture")
				{
					SetMaskID({});
#ifdef ENABLE_SYMBOL_WRAPPER
					SetModeWithoutTestCapture(CAPTURE, 30);
#else
					SetMode(CAPTURE);
#endif
				}
				else if (command[1] == "live")
				{
					SetMaskID({"A1", "A2", "A3", "A4", "A5", "D1", "D2", "D3", "D4", "D5"});
#ifdef ENABLE_SYMBOL_WRAPPER
					SetModeWithoutTestCapture(LIVE, 30);
#else
					SetMode(LIVE);
#endif
				}
				else if (command[1] == "none")
				{
					SetMaskID({});
#ifdef ENABLE_SYMBOL_WRAPPER
					SetModeWithoutTestCapture(NONE, 20);
#else
					SetMode(NONE);
#endif
				}
				else
				{
					SPDLOG_INFO("Invalid Command");
				}
			}
			else if (command[0] == "cc")
			{
				CAMERA_MODE prev_mode = GetMode();
				SetMaskID({});
#ifdef ENABLE_SYMBOL_WRAPPER
				SetModeWithoutTestCapture(CALIBRATE, 30);
				CaptureSequential(5, CaptureParams{4, 8000});
				GenerateCalibration();
				SetModeWithoutTestCapture(prev_mode, 30);
#else
				SetMode(CALIBRATE);
				Capture(5, CaptureParams{4, 8000});
				GenerateCalibration();
				SetMode(prev_mode);
#endif
			}
			else if (command[0] == "cs" && command.size() == 1)
			{
				SaveCalibration();
			}
			else if (command[0] == "cs" && command.size() >= 2)
			{
				const std::experimental::filesystem::path dir(command[1]);
				// SaveCalibration(dir.string().c_str());
				SaveCalibrationToDir(dir.string().c_str());
			}
			else if (command[0] == "cl" && command.size() == 1)
			{
				LoadCalibration();
			}
			else if (command[0] == "cl" && command.size() >= 2)
			{
				const std::experimental::filesystem::path dir(command[1]);
				// LoadCalibration(dir.string().c_str());
				LoadCalibrationFromDir(dir.string().c_str());
			}
			else if (command[0] == "ic")
			{
				if (IsCalibrated())
				{
					SPDLOG_INFO("Calibrated!");
				}
				else
				{
					SPDLOG_INFO("Not Calibrated!");
				}
			}
			else if (command[0] == "c")
			{
				SetMaskID({});
#ifdef ENABLE_SYMBOL_WRAPPER
				SetModeWithoutTestCapture(CAPTURE,30);
				AssertConnection();
				Capture(5, {4, 8000}); // retry 5, frame_count 4, timeout 8000ms
#else
				SetMode(CAPTURE);
				AssertConnection();
				Capture(5, {4, 8000}); // retry 5, frame_count 4, timeout 8000ms
#endif
			}
			else if (command[0] == "rs")
			{
				std::string path = "c:/symb/data/" + get_date_time();

				SaveRawCapture(path.c_str());
			}
			else if (command[0] == "s")
			{
				std::shared_ptr<open3d::geometry::PointCloud> pointcloud = GeneratePointCloud(filter_params);
				std::string path = "c:/symb/data/" + get_date_time();
				SavePointCloud(path.c_str(),true);
			}
			else if (command[0] == "v")
			{
				std::shared_ptr<open3d::geometry::PointCloud> pointcloud = GeneratePointCloud(filter_params);
				open3d::visualization::Visualizer visualizer;
				visualizer.CreateVisualizerWindow("PointCloud", 1600, 900);
				visualizer.AddGeometry(pointcloud);
				visualizer.Run();
				visualizer.DestroyVisualizerWindow();
			}
			else if (command[0] == "pw" && command.size() >= 2)
			{
				const std::experimental::filesystem::path dir(command[1]);
				SetPreset(5, dir.string().c_str());
			}
			else if (command[0] == "pr" && command.size() == 1)
			{
				std::string preset_str = GetPreset(5);
				SPDLOG_INFO("Preset: {}", preset_str);
			}
			else if (command[0] == "hwrst" && command.size() == 1)
			{
				ForceResetCameras();
			}
			else if ((command[0] == "loop" || command[0] == "loop-save") && command.size() >= 2)
			{
				while (_kbhit())
				{
					_getch();
				}
				if (!IsCalibrated())
				{
					LoadCalibration();
				}

				const unsigned long count = std::stoul(command[1]);
				const unsigned long sleep = (command.size() >= 3) ? std::stoul(command[2]) : 0ul;
				const bool enable_save = command[0] == "loop-save";

				for (unsigned long i = 0ul; i < count; ++i)
				{
					SPDLOG_INFO("Loop #{}/{}; Press \"q\" to stop loop", i + 1, count);
					SetMaskID({});
					SetMode(CAPTURE);
					AssertConnection();
					Capture(5, {4, 8000}); // retry 5, frame_count 4, timeout 8000ms
					std::string save_path = "C:/SYMBOL/shooting_data/5/" + get_date_time();
					SaveRawCapture(save_path.c_str());
					std::shared_ptr<open3d::geometry::PointCloud> pointcloud = GeneratePointCloud(filter_params);
					SavePointCloud(save_path.c_str(),true);

					SetMode(NONE);

					if (!enable_save)
					{
						std::experimental::filesystem::remove_all(save_path);
					}

					char c;
					bool stop = false;
					while (_kbhit())
					{
						c = static_cast<char>(_getch());
						if (c == 'q' || c == 'Q')
							stop = true;
					}
					if (stop)
						break;
					if (sleep > 0)
					{
						SPDLOG_INFO("Sleep {}min", sleep);
						std::this_thread::sleep_for(std::chrono::minutes(sleep));						
					}
				}
			}
			else if (command[0] == "l")
			{
				const std::vector<std::string> mask = {"A1", "A2", "A3", "A4", "A5", "D1", "D2", "D3", "D4", "D5"};
				const int width = 450, height = 800;
				const int size = width * height;
				std::map<std::string, bool> empty;
				std::map<std::string, std::chrono::steady_clock::time_point> timestamp;
				std::map<std::string, std::shared_ptr<std::vector<uint8_t>>> sources;
				std::map<std::string, std::unique_ptr<std::mutex>> mutes;
				for (const auto &id : mask)
				{
					empty[id] = true;
					timestamp[id] = std::chrono::steady_clock::now();
					sources[id] = std::make_shared<std::vector<uint8_t>>(size, 255);
					mutes[id] = std::make_unique<std::mutex>();
				}
				bool showing = true;
				const auto max = [](float a, float b) { return a > b ? a : b; };
				const auto min = [](float a, float b) { return a < b ? a : b; };
				std::function<void(const char *, const std::shared_ptr<open3d::geometry::PointCloud>)> cbf = [&](const char *id, const std::shared_ptr<open3d::geometry::PointCloud> pc) -> void {
					const std::string str_id = std::string(id);
					std::shared_ptr<std::vector<uint8_t>> &image = sources[str_id];
					std::lock_guard<std::mutex> _(*mutes[str_id]);
					if (!empty[str_id])
					{
						std::fill(image->begin(), image->end(), 255);
						empty[str_id] = true;
					}
					if (!pc->points_.empty())
					{
						timestamp[str_id] = std::chrono::steady_clock::now();
						const std::shared_ptr<open3d::geometry::PointCloud> down_sampled = VoxelDownSample(*pc, 0.01);
						const _f_range3d &range = init_params.range;
						for (const auto &point : down_sampled->points_) // 各点を2次元投影する
						{
							const int x_coordinate = width - static_cast<int>((min(max(static_cast<float>(point[0]), range.x.min), range.x.max) - range.x.min) / (range.x.max - range.x.min) * width);
							const int y_coordinate = height - static_cast<int>((min(max(static_cast<float>(point[1]), range.y.min), range.y.max) - range.y.min) / (range.y.max - range.y.min) * height);
							const int index = y_coordinate * width + x_coordinate;
							if (index < image->size())
							{
								image->at(index) = 0;
							}
						}
						empty[str_id] = false;
					}
				};
				CAMERA_MODE prev_mode = GetMode();
				SetMaskID(mask);
#ifdef ENABLE_SYMBOL_WRAPPER
				SetModeWithoutTestCapture(LIVE, 30);
#else
				SetMode(LIVE);
#endif
				StartLive({0.0f, {}, 0, 0, 0}, cbf);
				std::vector<uint8_t> data(size, 255);

				while (showing && StreamingLive())
				{
					std::fill(data.begin(), data.end(), 255);
					try
					{
						const std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
						for (const auto &source : sources) //各カメラのソース
						{
							std::lock_guard<std::mutex> _(*mutes[source.first]);
							if (!empty[source.first] && timestamp[source.first] + std::chrono::seconds(1) > now)
							{
								for (int i = 0; i < size; i++)
								{
									data[i] &= source.second->at(i); // 合成を行う
								}
							}
						}
						cv::Mat image(height, width, CV_8UC1, data.data());
						cv::imshow("image", image); // ウィンドウに画像表示
						const auto key = cv::waitKey(33); // 30fps
						if (key != -1)
							showing = false; // キー入力したら止める
					}
					catch (...)
					{
						showing = false; // エラーが発生したら止める
					}
				}
				cv::destroyAllWindows();

				StopLive();
#ifdef ENABLE_SYMBOL_WRAPPER
				SetModeWithoutTestCapture(prev_mode, 30);
#else
				SetMode(prev_mode);
#endif
			}
			else if (command[0] == "occ")
			{
				for (const char &pole : std::vector<char>{'A', 'B', 'C', 'D'})
				{
					for (int i = 1; i <= 5; i++)
					{
						const std::string id = fmt::format("{}{}", pole, i);
						SetMode(NONE);
						SetMaskID(std::vector<std::string>{id});
						SetMode(CALIBRATE);
						AssertConnection();
						Capture(5, CaptureParams{1, 8000});
						GenerateCalibration();
						SaveCalibration();
					}
				}
				SetMode(NONE);
				SetMaskID(std::vector<std::string>{});
			}
			else if (command[0] == "oc")
			{
				for (const char &pole : std::vector<char>{'A', 'B', 'C', 'D'})
				{
					for (int i = 1; i <= 5; i++)
					{
						const std::string id = fmt::format("{}{}", pole, i);
						SetMode(NONE);
						SetMaskID(std::vector<std::string>{id});
						SetMode(CAPTURE);
						AssertConnection();
						Capture(5, {4, 8000});
						std::shared_ptr<open3d::geometry::PointCloud> pointcloud = GeneratePointCloud(filter_params);
						std::string path = "c:/symb/data/" + get_date_time();
						SavePointCloud(path.c_str(),true);
					}
				}
				SetMode(NONE);
				SetMaskID(std::vector<std::string>{});
			}
			else if (command[0] == "q")
			{
				quit = true;
			}
			else
			{
				SPDLOG_INFO("Invalid Command");
			}
		}
		catch (const SymbolErrors &e)
		{
			for (const std::runtime_error &camera_error : e.errors())
			{
				SPDLOG_ERROR("Camera Error [{}]", camera_error.what());
			}
			try
			{
				std::string path = "c:/symb/error/" + get_date_time();
				SaveRawCapture(path.c_str());
			}
			catch (...)
			{
			}
		}
		catch (const std::runtime_error &e)
		{
			SPDLOG_ERROR("Error [{}]", e.what());
			try
			{
				std::string path = "c:/symb/error/" + get_date_time();
				SaveRawCapture(path.c_str());
			}
			catch (...)
			{
			}
		}
	}

	SPDLOG_INFO("Stopping sdk_test app");

	try
	{
		Terminate();
	}
	catch (...)
	{
		SPDLOG_ERROR("Termination Failed. Exiting.");
		TerminateProcess(GetCurrentProcess(), 1);
		return EXIT_FAILURE;
	}
	return EXIT_SUCCESS;
};

void printCommandHelp()
{
	SPDLOG_INFO("mode capture:\t\tSet to CAPTURE mode");
	SPDLOG_INFO("mode calibrate:\t\tSet to CALIBRATE mode");
	SPDLOG_INFO("mode live:\t\tSet to LIVE mode");
	SPDLOG_INFO("mode none:\t\tSet to NONE mode (Streams Closed)");
	SPDLOG_INFO("cc:\t\t\tCalibrate all cameras");
	SPDLOG_INFO("cl:\t\t\tLoad latest calibration data from master");
	SPDLOG_INFO("cl <directory>:\t\tLoad calibration data from directory");
	SPDLOG_INFO("cs:\t\t\tSave calibration data to master");
	SPDLOG_INFO("cs <directory>:\t\tSave current calibration data to directory");
	SPDLOG_INFO("pw <json filepath>:\tLoad camera preset");
	SPDLOG_INFO("pr\t\t\tDump camera preset");
	SPDLOG_INFO("c:\t\t\tCapture");
	SPDLOG_INFO("rs:\t\t\tSave Raw Frames");
	SPDLOG_INFO("s:\t\t\tSave current capture data pointcloud");
	SPDLOG_INFO("v:\t\t\tView current capture data pointcloud");
	SPDLOG_INFO("l:\t\t\tLive View");
	SPDLOG_INFO("q:\t\t\tQuit");
	SPDLOG_INFO("");
	SPDLOG_INFO("loop <count>:\t\tRepeat [c->rs->s] <count> times");
}

std::vector<std::string> split_string(const std::string &string, const std::string &delimeter)
{
	std::vector<std::string> result;
	std::size_t previous = 0;
	std::size_t current = string.find(delimeter);
	while (current != std::string::npos)
	{
		const std::string &seg = string.substr(previous, current - previous);
		if (!seg.empty())
		{
			result.emplace_back(seg);
		}
		previous = current + 1;
		current = string.find(delimeter, previous);
	}
	const std::string &seg = string.substr(previous, current - previous);
	if (!seg.empty())
	{
		result.emplace_back(seg);
	}
	return result;
}

std::string get_date_time()
{
	const auto p = std::chrono::system_clock::now();
	const auto t = std::chrono::system_clock::to_time_t(p);
	struct tm tt = {};
	localtime_s(&tt, &t);

	return fmt::format("{:04}_{:02}_{:02}_{:02}_{:02}_{:02}_{:03}", //
					   tt.tm_year + 1900, tt.tm_mon + 1, tt.tm_mday, tt.tm_hour, tt.tm_min, tt.tm_sec, //
					   std::chrono::duration_cast<std::chrono::milliseconds>(p.time_since_epoch()).count() % 1000);
}