#include "sensor.h"
#include "../include/sensor.h"

#include <exception.h>

#include "defines.h"

symb::scanner::Sensor::SensorPtr::SensorPtr(const rs2::sensor &sensor, const std::vector<StreamConfig> &configs) : rs2_sensor_(sensor)
{
	for (const StreamConfig &config : configs)
	{
		//const std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
		profiles_.emplace_back(config.to_profile(rs2_sensor_));
		//SPDLOG_TRACE("profiles_.emplace_back(config.to_profile(rs2_sensor_)) took {}ms", std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count());
	}
}

symb::scanner::Sensor::SensorPtr::~SensorPtr()
{
	try {
		close();
	}
	catch (const std::exception &e)
	{
		const std::string msg = e.what();
		if (!utility::contains(msg, "UVC device was not opened!"))
		{
			SPDLOG_WARN("Sensor Destructor Close Error {} ({}:{})", e.what(), FILENAME, __LINE__);
		}
	}
}

void symb::scanner::Sensor::SensorPtr::open()
{
	try
	{
		if (rs2_sensor_)
		{
			try
			{
				rs2_sensor_.stop();
				rs2_sensor_.close();
			}
			catch (...)
			{
			}
			rs2_sensor_.open(profiles_);
		}
		else
		{
			throw error::UnRecoverableError(fmt::format("Sensor Failed Opening [rs2_sensor instance is null] ({}:{})", FILENAME, __LINE__));
		}
	}
	catch (const rs2::error &e)
	{
		throw error::RecoverableError(fmt::format("Sensor Failed Opening [{}({}): {}] ({}:{})", e.get_failed_function(), e.get_failed_args(), e.what(), FILENAME, __LINE__));
	}
}

static std::mutex m;

void symb::scanner::Sensor::SensorPtr::close()
{
	try
	{
		if (rs2_sensor_)
		{
			//! ここでロックすることでClose処理がシリアルに実行される & Live中に処理が止まることがなくなる
			// std::lock_guard<std::mutex> _(m);
			rs2_sensor_.close();
		}
		else
		{
			throw error::UnRecoverableError(fmt::format("Sensor Failed Closing [rs2_sensor instance is null] ({}:{})", FILENAME, __LINE__));
		}
	}
	catch (const rs2::error &e)
	{
		if (!utility::contains(e.what(), "UVC device was not opened"))
		{
			throw error::RecoverableError(fmt::format("Sensor Failed Closing [{}({}): {}] ({}:{})", e.get_failed_function(), e.get_failed_args(), e.what(), FILENAME, __LINE__));
		}
	}
}

rs2_intrinsics symb::scanner::Sensor::SensorPtr::get_intrinsics(const rs2_stream stream)
{
	for (const rs2::stream_profile &profile: profiles_)
	{
		if (profile.stream_type() == stream)
		{
			try
			{
				return profile.as<rs2::video_stream_profile>().get_intrinsics();
			}
			catch (const rs2::recoverable_error &e)
			{
				throw error::RecoverableError(fmt::format("Sensor failed to get intrinsics [{}({}): {}] ({}:{})", e.get_failed_function(), e.get_failed_args(), e.what(), FILENAME, __LINE__));
			}
			catch (const rs2::error &e)
			{
				throw error::UnRecoverableError(fmt::format("Sensor failed to get intrinsics [{}({}): {}] ({}:{})", e.get_failed_function(), e.get_failed_args(), e.what(), FILENAME, __LINE__));
			}
			catch (const std::exception &e)
			{
				throw error::UnRecoverableError(fmt::format("Sensor failed to get intrinsics [{}] ({}:{})", e.what(), FILENAME, __LINE__));
			}
		}
	}
	throw error::UnRecoverableError(fmt::format("Sensor get_intrinsics cannot find stream [{}] ({}:{})", rs2_stream_to_string(stream), FILENAME, __LINE__));
}
