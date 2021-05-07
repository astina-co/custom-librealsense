#include "streamer.h"
#include <exception.h>

#include "defines.h"

symb::scanner::Streamer::StreamerPtr::StreamerPtr(std::vector<StreamConfig> &configs, rs2::device &device) : config_(configs), device_(device), dummy_color_profile_(nullptr)
{
	if (config_.empty())
	{
		throw error::BadStateError(fmt::format("Streamer Stream Config Empty ({}:{})", FILENAME, __LINE__));
	}
	std::vector<StreamConfig> depth_config;
	std::vector<StreamConfig> color_config;
	for (const StreamConfig &config: config_)
	{
		switch (config.stream)
		{
			case rs2_stream::RS2_STREAM_INFRARED:
			case rs2_stream::RS2_STREAM_DEPTH:
				depth_config.emplace_back(config);
				break;
			case rs2_stream::RS2_STREAM_COLOR:
				if (!config.dummy)
				{
					color_config.emplace_back(config);
				}
				else
				{
					dummy_color_stream_ = true;
					dummy_color_profile_ = std::make_unique<rs2::video_stream_profile>(config.to_profile(utility::get_color_sensor(device_)).as<rs2::video_stream_profile>());
				}
				break;
			default:
				throw error::BadStateError(fmt::format("Invalid Stream Type {} ({}:{})", config.name, FILENAME, __LINE__));
		}
	}
	if (!depth_config.empty())
	{
		sensors_.emplace_back(Sensor(device_.first<rs2::depth_sensor>(), depth_config));
	}
	if (!color_config.empty())
	{
		sensors_.emplace_back(Sensor(utility::get_color_sensor(device_), color_config));
	}

	for (Sensor &sensor: sensors_)
	{
		sensor.open();
	}
}


symb::scanner::Streamer::StreamerPtr::~StreamerPtr()
{ 
	if (started())
	{
		{
			for (Sensor &sensor : sensors_)
			{
				try
				{
					sensor.get().stop();
				}
				catch (const rs2::error &e)
				{
					const std::string msg = e.what();
					if (!utility::contains(msg, "UVC device is not streaming"))
					{
					}
				}
			}
		}
	}
}

void symb::scanner::Streamer::StreamerPtr::start()
{
	started_ = true;
	for (Sensor &sensor : sensors_)
	{
		sensor.get().start(sync);
	}
}

void symb::scanner::Streamer::StreamerPtr::stop()
{
	started_ = false;
	for (Sensor &sensor : sensors_)
	{
		try
		{
			sensor.get().stop();
		}
		catch (const rs2::error &e)
		{
			const std::string msg = e.what();
			if (!utility::contains(msg, "UVC device is not streaming"))
			{
				throw;
			}
		}
	}
}

void symb::scanner::Streamer::StreamerPtr::close()
{
	if (nullptr == this)
	{
		throw error::UnRecoverableError(fmt::format("Streamer is NullPointer ({}:{})", FILENAME, __LINE__));
	}
	std::mutex m;
	std::ostringstream e_oss;
	for (Sensor &sensor : sensors_)
	{
		Sensor s(sensor);
			try
			{
				s.close();
			}
			catch (const rs2::error &e)
			{
				std::lock_guard<std::mutex> _(m);
				e_oss << "sensor close threw [" << e.get_failed_function() << '(' << e.get_failed_args() << "): " << e.what() << "] ";
			}
			catch (const std::exception &e)
			{
				std::lock_guard<std::mutex> _(m);
				e_oss << "sensor close threw [" << e.what() << "] ";
			}
	}
	if (!e_oss.str().empty())
	{
		throw error::RecoverableError(fmt::format("{} ({}:{})", e_oss.str(), FILENAME, __LINE__));
	}
}

// SetModeから呼ばれる
symb::scanner::Frameset symb::scanner::Streamer::StreamerPtr::get_frameset(const int count = 4, const long long timeout = 6000)
{
	if (!started()) {
		throw error::BadStateError(fmt::format("Streamer not started  ({}:{})", FILENAME, __LINE__));
	}
	SetThreadPriority(GetCurrentThread(), ABOVE_NORMAL_PRIORITY_CLASS);
	rs2::frameset fs;
	const std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout);
	while (sync.poll_for_frames(&fs)) {}

	std::ostringstream oss;
	std::vector<Frame> frames;
	while (frames.size() < static_cast<size_t>(count))
	{
		try
		{
			std::chrono::steady_clock::time_point current_time = std::chrono::steady_clock::now();

			if (end >= current_time)
			{
				// SPDLOG_TRACE("Capturing... {}", std::chrono::duration_cast<std::chrono::milliseconds>(end - current_time).count());
				fs = sync.wait_for_frames(static_cast<unsigned int>(std::chrono::duration_cast<std::chrono::milliseconds>(end - current_time).count()));
				bool valid = true;
				for (const StreamConfig &config: config_)
				{
					switch (config.stream)
					{
						case rs2_stream::RS2_STREAM_COLOR:
							if (!config.dummy && !fs.get_color_frame())
								valid = false;
							break;
						case rs2_stream::RS2_STREAM_DEPTH:
							if (!fs.get_depth_frame()) valid = false;
							break;
						case rs2_stream::RS2_STREAM_INFRARED:
							if (!fs.get_infrared_frame()) valid = false;
							break;
						default:
							break;
					}
				}
				if (valid)
				{
					if (dummy_color_stream_) {
						fs = this->append_dummy_color_stream(fs);
					}
					frames.emplace_back(Frame(fs));
				}
				else
				{
					oss.str(std::string());
					oss << (fs.get_color_frame() ? "" : " [No Color Frame]");
					oss << (fs.get_depth_frame() ? "" : " [No Depth Frame]");
					oss << (fs.get_infrared_frame() ? "" : " [No Infrared Frame]");
				}
			}
			else
			{
				frames.clear();
				throw error::TimeoutError(fmt::format("Device Capture Timeout[{}] {} ({}:{})", timeout, oss.str(), FILENAME, __LINE__));
			}
		}
		catch (const rs2::error &e)
		{
			if (!utility::contains(e.what(), "Frame did not arrive in time!") || end > std::chrono::steady_clock::now())
			{
				throw error::TimeoutError(fmt::format("rs2::error at {}({}): {} ({}:{})", e.get_failed_function(), e.get_failed_args(), e.what(), FILENAME, __LINE__));
			}
		}
		catch (const std::exception &e)
		{
			throw error::RecoverableError(fmt::format("Streamer cannot get frames {} ({}:{})", e.what(), FILENAME, __LINE__));
		}
	}
	SetThreadPriority(GetCurrentThread(), NORMAL_PRIORITY_CLASS);
	return Frameset(frames);
}

// Capture,Liveから呼ばれる
symb::scanner::Frameset symb::scanner::Streamer::StreamerPtr::get_capture_frameset(const int count = 4, const long long timeout = 6000)
{
	if (!started()) {
		throw error::BadStateError(fmt::format("Streamer not started  ({}:{})", FILENAME, __LINE__));
	}
	SetThreadPriority(GetCurrentThread(), ABOVE_NORMAL_PRIORITY_CLASS);
	rs2::frameset fs;
	const std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout);
	while (sync.poll_for_frames(&fs)) {}

	std::ostringstream oss;
	std::vector<Frame> frames;
	while (frames.size() < static_cast<size_t>(count))
	{
		try
		{
			std::chrono::steady_clock::time_point current_time = std::chrono::steady_clock::now();

			if (end >= current_time)
			{
				// SPDLOG_TRACE("Capturing... {}", std::chrono::duration_cast<std::chrono::milliseconds>(end - current_time).count());
				fs = sync.wait_for_frames(static_cast<unsigned int>(std::chrono::duration_cast<std::chrono::milliseconds>(end - current_time).count()));
				bool valid = true;
				for (const StreamConfig &config: config_)
				{
					switch (config.stream)
					{
						case rs2_stream::RS2_STREAM_COLOR:
							if (!config.dummy && !fs.get_color_frame())
								valid = false;
							break;
						case rs2_stream::RS2_STREAM_DEPTH:
							if (!fs.get_depth_frame()) valid = false;
							break;
						case rs2_stream::RS2_STREAM_INFRARED:
							if (!fs.get_infrared_frame()) valid = false;
							break;
						default:
							break;
					}
				}
				if (valid)
				{
					if (dummy_color_stream_) {
						fs = this->append_dummy_color_stream(fs);
					}
					frames.emplace_back(Frame(fs));
				}
				else
				{
					oss.str(std::string());
					oss << (fs.get_color_frame() ? "" : " [No Color Frame]");
					oss << (fs.get_depth_frame() ? "" : " [No Depth Frame]");
					oss << (fs.get_infrared_frame() ? "" : " [No Infrared Frame]");
				}
			}
			else
			{
				frames.clear();
				SPDLOG_ERROR(fmt::format("Device Capture Timeout[{}] {} ({}:{})", timeout, oss.str(), FILENAME, __LINE__));
				throw error::UnRecoverableTimeoutError(fmt::format("Device Capture Timeout[{}] {} ({}:{})", timeout, oss.str(), FILENAME, __LINE__));
			}
		}
		catch (const rs2::error &e)
		{
			if (!utility::contains(e.what(), "Frame did not arrive in time!") || end > std::chrono::steady_clock::now())
			{
				SPDLOG_ERROR(fmt::format("rs2::error at {}({}): {} ({}:{})", e.get_failed_function(), e.get_failed_args(), e.what(), FILENAME, __LINE__));
				throw error::UnRecoverableTimeoutError(fmt::format("rs2::error at {}({}): {} ({}:{})", e.get_failed_function(), e.get_failed_args(), e.what(), FILENAME, __LINE__));
			}
		}
		catch (const error::UnRecoverableTimeoutError &e)
		{
			throw e;
		}
		catch (const std::exception& e)
		{
			SPDLOG_ERROR(fmt::format("Streamer cannot get frames {} ({}:{})", e.what(), FILENAME, __LINE__));
			throw error::UnRecoverableError(fmt::format("Streamer cannot get frames {} ({}:{})", e.what(), FILENAME, __LINE__));
		}
		catch(...)
		{
			// その他のエラーもExceptionを発生させる。
			throw std::runtime_error(fmt::format("Streamer threw exception [???]").c_str());
		}
	}
	SetThreadPriority(GetCurrentThread(), NORMAL_PRIORITY_CLASS);
	return Frameset(frames);
}

rs2_intrinsics symb::scanner::Streamer::StreamerPtr::get_intrinsics(const rs2_stream stream_type)
{
	for (Sensor &sensor : sensors_)
	{
		try
		{
			return sensor.get_intrinsics(stream_type);
		}
		catch (...)
		{
		}
	}
	throw error::BadStateError(fmt::format("Streamer get_intrinsics cannot find stream {}  ({}:{})", rs2_stream_to_string(stream_type), FILENAME, __LINE__));
}

rs2::frameset symb::scanner::Streamer::StreamerPtr::append_dummy_color_stream(const rs2::frameset &frame)
{
	rs2::processing_block bundler([this](rs2::frame f, rs2::frame_source &src) {
		std::vector<rs2::frame> bundle;
		bundle.push_back(f);

		auto prof = *this->dummy_color_profile_;
		auto color_frame = src.allocate_video_frame(prof, f, 8 * 3, prof.width(), prof.height(), 0);
		// fill with gray color
		memset((void *)color_frame.get_data(), 128, color_frame.get_data_size());
		bundle.push_back(std::move(color_frame));

		auto fs = src.allocate_composite_frame(bundle);
		src.frame_ready(fs);
		bundle.clear();
	});
	rs2::frame_queue q;
	bundler.start(q);
	bundler.invoke(frame);

	return q.wait_for_frame();
}
