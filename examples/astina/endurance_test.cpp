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

#include <cxxopts.hpp>

std::string get_date_time();

void milestone_log(const std::string &key, const std::string &message = "")
{
	SPDLOG_INFO("<{}>{}{}", key, message.empty() ? "" : ": ", message);
}

void unhandling_exception_log(const std::string &key, const std::string &message = "")
{
	SPDLOG_INFO("<{}>!{}{}", key, message.empty() ? "" : ": ", message);
}

class AstinaSDK {
public:
	explicit AstinaSDK(const bool symbol_mode) : _symbol_mode(symbol_mode){
	}

	void myInitialize(const InitializeParams init_params) {
		if (_symbol_mode) {
			milestone_log("will_initialize_without_preset");
			Initialize(init_params);
			milestone_log("done_initialize_without_preset");
			//
			milestone_log("will_set_preset_sequential");
			SetPresetSequential(5, init_params.preset_json_path.c_str());
			milestone_log("done_set_preset_sequential");
		} else {
			Initialize(init_params);
		}
	}

	void mySetMaskID(const std::vector<std::string> &enabled) {
		if (_symbol_mode) {
			SetMaskIDSequential(enabled);
		} else {
			SetMaskID(enabled);
		}
	}

	void mySetMode(CAMERA_MODE mode, bool on_first = false) {
		if (_symbol_mode) {
			if (on_first) {
				SetModeSequential(mode);
			} else {
				SetModeWithoutTestCapture(mode, 30000);
			}
		} else {
			SetMode(mode);
		}
	}

	void myCapture(unsigned int retry_count, CaptureParams params, CAMERA_MODE mode) {
		assert(mode == CALIBRATE || mode == CAPTURE);
		if (_symbol_mode) {
			if (mode == CALIBRATE) {
				CaptureSequential(retry_count, params);
			} else {
				Capture(retry_count, params);
			}
		} else {
			Capture(retry_count, params);
		}
	}

	// WARNING: StartLiveを別階層から呼ぶと何故かフレームが取得できなくなる。threadのcontextとか？ 謎すぎるが面倒なので直接呼ぶ
	// void myStartLive(FilterParams filter_params, std::function<void (const char *, std::shared_ptr<open3d::geometry::PointCloud>)> &callback) {
	// 	if (_symbol_mode) {
	// 		StartLiveSequential(filter_params, callback);
	// 	} else {
	// 		StartLive(filter_params, callback);
	// 	}
	// }

	void myStopLive() {
		if (_symbol_mode) {
			StopLiveSequential();
		} else {
			StopLive();
		}
	}

	void myAssertConnection(){
		if (_symbol_mode) {
			AssertConnectionSequential();
		} else {
			AssertConnection();
		}
	}

	void myTerminate(){
		if (_symbol_mode) {
			TerminateSequential();
		} else {
			Terminate();
		}
	}

	void myTerminateProcess(HANDLE hProcess, UINT uExitCode){
		TerminateProcess(hProcess, uExitCode);
	}

private:
	bool _symbol_mode;
};

// シルエットキャプチャ時のFPS計算用
typedef struct period_counter_t
{
	long long elapsed;
	long long counter;
}period_counter;

int main(int argc, char **argv)
{
	int LIVE_MODE_FPS = 15; // TODO should sync with embeded one on SetMode
	cxxopts::Options options("endurance test", "try again and again for real usecase");
	int loop_count;
	int loop_wait;
	unsigned int capture_attempts;
	unsigned long long capture_timeout;
	bool symbol_mode = false;
	bool do_calibration_on_loop = false, do_live_mode_on_loop = false, disable_capture_on_loop = false;
	bool do_output = false;
	bool help = false;
	int live_mode_time;
	std::string log_dir, config_path;

	cxxopts::OptionAdder builder = options.add_options();
	builder("l,loop", "loop count", cxxopts::value<int>(loop_count)->default_value("100"));
	builder("a,symbol_mode", "symbol mode for stability (sequential process)", cxxopts::value<bool>(symbol_mode)->default_value("true"));
	builder("c,calibration", "add calibration test on loop", cxxopts::value<bool>(do_calibration_on_loop)->default_value("false"));
	builder("d,livemode", "add live mode test on loop", cxxopts::value<bool>(do_live_mode_on_loop)->default_value("true"));
	builder("y,livemode_time", "how long live mode keep on loop", cxxopts::value<int>(live_mode_time)->default_value("60000"));
	builder("e,disable_capture", "disable capture on loop", cxxopts::value<bool>(disable_capture_on_loop)->default_value("false"));
	builder("o,output", "output capture result on loop", cxxopts::value<bool>(do_output)->default_value("false"));
	builder("t,capture_attempts", "how many try capture", cxxopts::value<unsigned int>(capture_attempts)->default_value("1"));
	builder("s,capture_timeout", "how long capture wait(ms)", cxxopts::value<unsigned long long>(capture_timeout)->default_value("8000"));
	builder("w,wait", "wait time(ms) on per loop", cxxopts::value<int>(loop_wait)->default_value("3000"));
	builder("log_dir", "api/app log directory", cxxopts::value<std::string>(log_dir)->default_value("./log"));
	builder("config_path", "config path(json)", cxxopts::value<std::string>(config_path)->default_value("config.json"));

	builder("h,help", "Print usage", cxxopts::value<bool>(help)->default_value("false"));

	for (int ai = 0; ai < argc; ai++)
	{
		SPDLOG_DEBUG("Arg[{}/{}] : {}", ai, argc, argv[ai]);
	}

	try
	{
		options.parse(argc, argv);
	}
	catch (cxxopts::OptionException &e)
	{
		std::cout << "parse error:" << e.what() << std::endl;
		return -1;
	}

	if (help) {
		std::cout << options.help() << std::endl;
		return 0;
	}

	// ログ出力をUTF-8に設定
	SetConsoleOutputCP(65001);

	std::string log_file_label = fmt::format("endurance_test_log_{}", get_date_time());
	if (!std::experimental::filesystem::exists(log_dir))
	{
		std::experimental::filesystem::create_directories(log_dir);
	}

	std::string output_dir = fmt::format("{}/{}_output", log_dir, log_file_label);
	if (do_output && !std::experimental::filesystem::exists(output_dir))
	{
		std::experimental::filesystem::create_directories(output_dir);
	}

	std::vector<spdlog::sink_ptr> sinks;
	sinks.push_back(std::make_shared<spdlog::sinks::stdout_color_sink_mt>());
	sinks.push_back(std::make_shared<spdlog::sinks::rotating_file_sink_mt>(fmt::format("{}/{}.txt", log_dir, log_file_label), 1048576 * 5, 3));
	const std::shared_ptr<spdlog::logger> combined_logger = std::make_shared<spdlog::logger>("ENDURANCE_TEST", begin(sinks), end(sinks));

	spdlog::register_logger(combined_logger);
	spdlog::set_default_logger(combined_logger);
	spdlog::set_level(spdlog::level::trace);
	spdlog::flush_on(spdlog::level::info);
	spdlog::set_pattern("[%5t]%+");

	SPDLOG_INFO("loop_count:{}", loop_count);
	SPDLOG_INFO("_symbol_mode:{}", symbol_mode ? "o" : "x");
	SPDLOG_INFO("calibration:{}", do_calibration_on_loop ? "o" : "x");
	SPDLOG_INFO("live_mode:{}", do_live_mode_on_loop ? "o" : "x");
	SPDLOG_INFO("capture:{}", !disable_capture_on_loop ? "o" : "x");
	SPDLOG_INFO("capture_attemp:{}", capture_attempts);
	SPDLOG_INFO("capture_timeout:{}", capture_timeout);
	SPDLOG_INFO("wait per loop:{}", loop_wait);

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
			// "./config.json", // config path
			config_path,
			"./preset.json", // preset path
			{
				{-0.5f, 0.5f}, {FLT_EPSILON, 2.00f}, {-0.6f, 0.4f} // range x, y, z
			},
			false, // cut_floor
			0.0, // offset_y
			0.3, // foot_range
			"c:/symb/calibration", // calibration_dir
			log_dir,  // log_dir
			false, // enable_set_preset
			5 // thread_num
	};

	AstinaSDK sdk(symbol_mode);

	// 点群生成とLiveでフィルターパラメータを分ける
	//const FilterParams filter_params = {0.0f, {{400, 2.5f}, {200, 2.5f}, {100, 2.5f}}, 30, 0.02, 20};
	const FilterParams generate_pointcloud_filter_params = {0.0003f, {}, 30, 0.02, 20};
	const FilterParams live_filter_params = { 0.0f, {}, 0, 0, 0 };

	const std::chrono::high_resolution_clock::time_point init_tic = std::chrono::high_resolution_clock::now();
	milestone_log("will_initialize");
	try
	{
		sdk.myInitialize(init_params);
	}
	catch (...)
	{
		// SPDLOG_ERROR("Initialization Failed. Exiting.");
		unhandling_exception_log("initialize", "failed");
		return EXIT_FAILURE;
	}
	const double init_elapsed = static_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - init_tic).count()) / 1000.0;
	milestone_log("done_initialize", fmt::format("elapsed:{} sec", init_elapsed));
	bool something_failed = false;
	// global try for every time do terminate.
	try {
		// for error handling for force terminate on any case
		const std::chrono::high_resolution_clock::time_point first_set_mode_tic = std::chrono::high_resolution_clock::now();

		milestone_log("will_first_set_mode");
		try
		{
			sdk.mySetMaskID({});
			sdk.mySetMode(CAPTURE, true);
		}
		catch (...)
		{
			unhandling_exception_log("first_set_mode", "failed");
			throw;
		}
		milestone_log("done_first_set_mode");
		//
		milestone_log("will_first_reset_mode");
		try
		{
			sdk.mySetMode(NONE, true);
		}
		catch (...)
		{
			unhandling_exception_log("first_reset_mode", "failed");
			throw;
		}
		const double first_set_mode_elapsed = static_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - first_set_mode_tic).count()) / 1000.0;
		milestone_log("done_first_reset_mode", fmt::format("elapsed: {} sec", first_set_mode_elapsed));

		// load calibration data from master
		milestone_log("will_calibration");
		LoadCalibration();
		milestone_log("done_calibration");

		bool do_loop = do_calibration_on_loop || do_live_mode_on_loop || !disable_capture_on_loop;
		if (do_loop){
			milestone_log("loop_start", fmt::format("total loop:{}", loop_count));
			for (int i = 0; i < loop_count; i++)
			{
				if (do_calibration_on_loop) {

					// room time between set_mode/capture
					std::this_thread::sleep_for(std::chrono::milliseconds(3000));

					milestone_log("will_calibration_on_loop", fmt::format("loop:{}", i+1));
					milestone_log("will_set_mode_for_calibration_on_loop", fmt::format("loop:{}", i+1));
					try {
						sdk.mySetMaskID({});
						sdk.mySetMode(CALIBRATE);
					} catch (...) {
						unhandling_exception_log("set_mode_for_calibration_on_loop", fmt::format("failed on loop:{}", i + 1));
						throw;
					}
					milestone_log("done_set_mode_for_calibration_on_loop", fmt::format("loop:{}", i+1));
					milestone_log("will_capture_for_calibration_on_loop", fmt::format("loop:{}", i+1));
					try {
						//キャリブレーションの設定でキャプチャするだけ
						sdk.myCapture(1, { 4, 8000 }, CALIBRATE);
					} catch (...) {
						// 再撮影の可能性を信じてループは継続する
						something_failed = true;
						unhandling_exception_log("capture_for_calibration_on_loop", fmt::format("failed on loop:{}", i + 1));
					}
					if (!something_failed)
					{
						milestone_log("done_capture_for_calibration_on_loop", fmt::format("loop:{}", i + 1));

						milestone_log("will_reset_mode_for_calibration_on_loop", fmt::format("loop:{}", i + 1));
						try
						{
							sdk.mySetMode(NONE);
						}
						catch (...)
						{
							unhandling_exception_log("reset_mode_for_calibration_on_loop", fmt::format("failed on loop:{}", i + 1));
							throw;
						}
						milestone_log("done_reset_mode_for_calibration_on_loop", fmt::format("loop:{}", i + 1));
						milestone_log("done_calibration_on_loop", fmt::format("loop:{}", i + 1));
						std::this_thread::sleep_for(std::chrono::milliseconds(3000));
					}	
				}

				if (do_live_mode_on_loop) {
					// room time between set_mode/capture
					std::this_thread::sleep_for(std::chrono::milliseconds(3000));

					milestone_log("will_live_mode_on_loop", fmt::format("loop:{}", i+1));
					std::unordered_map<std::string, int> counter;
					std::unordered_map<std::string, std::vector<period_counter>> period_counter_map; // 個別のカメラカウンタ
					std::unordered_map<std::string, int> period_index; // period_counter_map.secondのインデックス
					std::unordered_map<std::string, std::chrono::high_resolution_clock::time_point> period_start_time; // 各カメラごとのperiod開始時刻
					std::mutex mtx;

					std::map<std::string, std::chrono::high_resolution_clock::time_point> time_points;
					std::map<std::string, std::unique_ptr<std::mutex>> mutes;
					std::function<void(const char *, const std::shared_ptr<open3d::geometry::PointCloud>)> cbf = [&](const char *c_id, const std::shared_ptr<open3d::geometry::PointCloud> &pc) -> void {
						const std::string id(c_id);

						if (mutes.find(id) == mutes.end())
						{
							mutes[id] = std::make_unique<std::mutex>();
						}
						std::lock_guard<std::mutex> _(*mutes[id]);

						if (counter.find(id) == counter.end()) {
							counter[id] = 0;
							// 以下10秒ごとのFPS計算用
							period_index[id] = 0;
							period_counter period;
							period.elapsed = -1;
							period.counter = 0;
							period_counter_map[id] = std::vector<period_counter>(); // vector
							period_counter_map[id].push_back(period); // period_counter
							period_start_time[id] = std::chrono::high_resolution_clock::now(); // 計測開始時刻;
						}
						counter[id] += 1;
						
						// 以下10秒ごとのFPS計算用
						const long long elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - period_start_time[id]).count();
						period_counter_map[id].at(period_index[id]).elapsed = elapsed;
						period_counter_map[id].at(period_index[id]).counter += 1;
						
						// 10秒経過していたら新たなperiod_counterを作成
						if(elapsed >= 10000){
							period_counter period;
							period.elapsed = -1;
							period.counter = 0;
							period_counter_map[id].push_back(period);
							period_index[id] += 1;
							SPDLOG_INFO("{}: Heart Beat", id);
							period_start_time[id] = std::chrono::high_resolution_clock::now();  // 開始時刻の更新
						}
					};
					milestone_log("will_start_live_on_loop", fmt::format("loop:{}", i+1));
					try {
						sdk.mySetMaskID({});
						sdk.mySetMode(LIVE);

						// room time between set_mode/capture
						std::this_thread::sleep_for(std::chrono::milliseconds(3000));

						//StartLiveはなぜか直接呼ばないと駄目（frameの取得ができなくなる）。 謎すぎるので要確認
						if (symbol_mode) {
							StartLiveSequential(live_filter_params, cbf);
						} else {
							StartLive(live_filter_params, cbf);
						}
					} catch(...) {
						unhandling_exception_log("start_live_on_loop", fmt::format("failed on loop:{}", i+1));
						throw;
					}
					milestone_log("done_start_live_on_loop", fmt::format("loop:{}", i+1));

					{
						const std::chrono::high_resolution_clock::time_point start_live_mode_at = std::chrono::high_resolution_clock::now();
						const std::chrono::high_resolution_clock::time_point end_live_mode_at = start_live_mode_at + std::chrono::milliseconds(live_mode_time);
						for (;;)
						{
							const std::chrono::high_resolution_clock::time_point now = std::chrono::high_resolution_clock::now();
							const long long duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_live_mode_at - now).count();
							if (duration < 0)
							{
								break;
							}
							SPDLOG_INFO("Remaining Time; {}/{}ms", duration, live_mode_time);
							if (duration > 1000)
							{
								std::this_thread::sleep_for(std::chrono::milliseconds(1000));
							}
							else
							{
								std::this_thread::sleep_until(end_live_mode_at);		
							}
						}
					}
					
					milestone_log("will_stop_live_on_loop", fmt::format("loop:{}", i+1));
					try {
						sdk.myStopLive();
						sdk.mySetMode(NONE);
					} catch (...){
						unhandling_exception_log("stop_live_on_loop", fmt::format("failed on loop:{}", i+1));
						throw;
					}
					milestone_log("done_stop_live_on_loop", fmt::format("loop:{}", i+1));

					// フレームレート確認
					SPDLOG_INFO("live_mode time: {} ms", live_mode_time);
					int not_enough_count = 0;
					for (std::unordered_map<std::string, std::vector<period_counter>>::iterator itr = period_counter_map.begin(); itr != period_counter_map.end(); ++itr)
					{
						std::vector<period_counter> period_counter_list = itr->second;
						bool is_fail = false;
						for (const period_counter& period: period_counter_list)
						{
							if (period.elapsed <= 0)
							{
								continue;
							}
							const double period_fps = static_cast<double>(period.counter) / (static_cast<double>(period.elapsed) / 1000.0);
							if (period_fps < LIVE_MODE_FPS * 0.5) {
								SPDLOG_WARN("\tframe : {}=>elapsed={} count={} fps={} (not enough. warning..)", itr->first, period.elapsed, period.counter, period_fps);
								is_fail = true;
							} else{
								SPDLOG_INFO("\tframe : {}=>elapsed={} count={} fps={}", itr->first, period.elapsed, period.counter, period_fps);
							}
						}
						if (is_fail)
						{
								not_enough_count += 1;
						}
					}

					if (counter.size() < GetEnabledDeviceCount()) {
						something_failed = true;
						SPDLOG_ERROR("lack frame info: only {}/{}", counter.size(), GetEnabledDeviceCount());
						unhandling_exception_log("live_mode_on_loop", fmt::format("failed on loop:{}, ", i + 1));
					} else if (not_enough_count > 0) { // 1台でもfpsの閾値を満たさなければエラーとする。
						something_failed = true;
						SPDLOG_ERROR("not enough frame cameras : {}", not_enough_count);
						unhandling_exception_log("live_mode_on_loop", fmt::format("failed on loop:{}", i+1));
					}
					if (!something_failed)
					{
						milestone_log("done_live_mode_on_loop", fmt::format("loop:{}", i + 1));
					}
				}

				if (!disable_capture_on_loop) {

					// room time between set_mode/capture
					std::this_thread::sleep_for(std::chrono::milliseconds(3000));

					milestone_log("will_capture_on_loop", fmt::format("loop:{}", i+1));
					milestone_log("will_set_mode_for_capture_on_loop", fmt::format("loop:{}", i + 1));
					const std::chrono::high_resolution_clock::time_point loop_set_mode_tic = std::chrono::high_resolution_clock::now();
					try
					{
						sdk.mySetMaskID({});
						sdk.mySetMode(CAPTURE);
						sdk.myAssertConnection();
					}
					catch (...)
					{
						unhandling_exception_log("set_mode_for_capture_on_loop", fmt::format("failed on loop:{}", i + 1));
						throw;
					}

					const double loop_set_mode_elapsed = static_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - loop_set_mode_tic).count()) / 1000.0;
					milestone_log("done_set_mode_for_capture_on_loop", fmt::format("loop:{}, elapsed:{} sec", i + 1, loop_set_mode_elapsed));

					// room time between set_mode/capture
					std::this_thread::sleep_for(std::chrono::milliseconds(3000));


					milestone_log("will_capture_process_on_loop", fmt::format("loop:{}", i + 1));
					const std::chrono::high_resolution_clock::time_point loop_capture_tic = std::chrono::high_resolution_clock::now();
					try
					{
						sdk.myCapture(capture_attempts, {4, capture_timeout}, CAPTURE);
					}
					catch (...)
					{
						unhandling_exception_log("capture_process_on_loop", fmt::format("failed on loop:{}", i + 1));

						//キャプチャの失敗の場合、そのままループを続行する（再撮影を模倣）。失敗扱いにはする。
						something_failed = true;

						// 一度Noneに戻す
						try
						{
							sdk.mySetMode(NONE);
						}
						catch (...)
						{
							unhandling_exception_log("reset_mode_on_loop_after_capture_failed", fmt::format("failed on loop {}", i + 1));
							//この場合もスルー（そのまま失敗する可能性が高いが・・。）
						}
					}
					if (!something_failed)
					{
						const double loop_capture_elapsed = static_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - loop_capture_tic).count()) / 1000.0;
						milestone_log("done_capture_process_on_loop", fmt::format("loop:{}, elapsed:{} sec", i + 1, loop_capture_elapsed));

						if (do_output)
						{
							milestone_log("will_output_capture_result");

							std::string save_path = fmt::format("{}/{}", output_dir, i);

							std::shared_ptr<open3d::geometry::PointCloud> pointcloud = GeneratePointCloud(generate_pointcloud_filter_params);
							SavePointCloud(save_path.c_str(), true);

							milestone_log("done_output_capture_result");
						}

						milestone_log("will_reset_mode_for_capture_on_loop", fmt::format("loop {}", i + 1));
						try
						{
							sdk.mySetMode(NONE);
						}
						catch (...)
						{
							unhandling_exception_log("reset_mode_for_capture_on_loop", fmt::format("failed on loop {}", i + 1));
							throw;
						}
						milestone_log("done_reset_mode_for_capture_on_loop", fmt::format("loop {}", i + 1));
						milestone_log("done_capture_on_loop", fmt::format("loop:{}", i + 1));
					}
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
				{
					milestone_log("stop_by_keyboard");
					break;
				}
				// sleep if next
				if (i < loop_count - 1) {
					std::cout << "sleep " << loop_wait << " msec for loop interval." << std::endl;
					std::this_thread::sleep_for(std::chrono::milliseconds(loop_wait));
				}
			}
		}
	}
	catch (...)
	{
		// TODO try-catchを仕込んだところ以外からのexceptionを拾う？
		something_failed = true;
	}

	milestone_log("will_terminate");
	try
	{
		sdk.myTerminate();
	}
	catch (...)
	{
		unhandling_exception_log("terminate", "failed");
		milestone_log("will_terminate_process");
		sdk.myTerminateProcess(GetCurrentProcess(), 1);
		milestone_log("done_terminate_process");
		return EXIT_FAILURE;
	}
	milestone_log("done_terminate");

	if (something_failed)
	{
		return EXIT_FAILURE;
	}
	else
	{
		return EXIT_SUCCESS;
	}
}

std::string get_date_time()
{
	const std::chrono::system_clock::time_point p = std::chrono::system_clock::now();
	const __time64_t t = std::chrono::system_clock::to_time_t(p);
	struct tm tt = {};
	localtime_s(&tt, &t);

	return fmt::format("{:04}_{:02}_{:02}_{:02}_{:02}_{:02}_{:03}", //
					   tt.tm_year + 1900, tt.tm_mon + 1, tt.tm_mday, tt.tm_hour, tt.tm_min, tt.tm_sec, //
					   std::chrono::duration_cast<std::chrono::milliseconds>(p.time_since_epoch()).count() % 1000);
}
