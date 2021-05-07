#include "sensor_info.h"
#include "exception.h"

#include "defines.h"

float symb::scanner::SensorInfo::get_option(const rs2_option option) const
{
	// SPDLOG_TRACE("Check support");
	if (!sensor_.supports(option))
	{
		throw error::OptionUnsupportedError(fmt::format("Option unsupported for get_option({}) ({}:{})", rs2_option_to_string(option), FILENAME, __LINE__), option);
	}
	// SPDLOG_TRACE("get_option({})", rs2_option_to_string(option));
	try
	{
		return sensor_.get_option(option);
	}
	catch (const rs2::error &e)
	{
		throw error::RecoverableError(fmt::format("sensor_.get_option(option) failed {}({}): {} ({}:{})", e.get_failed_function(), e.get_failed_args(), e.what(), FILENAME, __LINE__));
	}
}

void symb::scanner::SensorInfo::set_option(const rs2_option option, const float value)
{
	// SPDLOG_TRACE("Check support");
	if (!sensor_.supports(option))
	{
		throw error::OptionUnsupportedError(fmt::format("Option unsupported for set_option({}, {}) ({}:{})", 
			rs2_option_to_string(option), value, FILENAME, __LINE__), option);
	}
	// SPDLOG_TRACE("Check read-only");
	// if (!sensor_.is_option_read_only(option))
	// {
	//	throw std::runtime_error(fmt::format("Option is read-only for set_option({}, {})", rs2_option_to_string(option), value));
	// }
	// SPDLOG_TRACE("Check range");
	try
	{
		const rs2::option_range range = sensor_.get_option_range(option);
		if (value < range.min || range.max < value)
		{
			throw error::OptionOutOfRangeError(
				fmt::format("Value out of range for set_option({}, {}): range [{}, {}] ({}:{})", rs2_option_to_string(option), 
					value, range.min, range.max, FILENAME, __LINE__),  option, value, range.min, range.max);
		}
	}
	catch (const rs2::error &e)
	{
		throw error::RecoverableError(
			fmt::format("sensor.get_option_range({}) failed {}({}): {} ({}:{})", 
				rs2_option_to_string(option), e.get_failed_function(), e.get_failed_args(), e.what(), FILENAME, __LINE__));
	}
	// SPDLOG_TRACE("set_option({}, {})", rs2_option_to_string(option), value);
	try
	{
		return sensor_.set_option(option, value);
	}
	catch (const rs2::error &e)
	{
		throw error::RecoverableError(fmt::format("sensor.set_option() failed {}({}): {} ({}:{})", 
			e.get_failed_function(), e.get_failed_args(), e.what(), FILENAME, __LINE__));
	}
}

bool symb::scanner::SensorInfo::is_depth_sensor() const
{
	try
	{
		return sensor_.is<rs2::depth_sensor>();
	}
	catch (const rs2::error &e)
	{
		throw error::RecoverableError(fmt::format("sensor_.is<rs2::depth_sensor>() failed {}({}): {} ({}:{})", 
			e.get_failed_function(), e.get_failed_args(), e.what(), FILENAME, __LINE__));
	}
}

float symb::scanner::SensorInfo::get_depth_scale() const
{
	if (!is_depth_sensor())
	{
		throw error::BadStateError(fmt::format("SensorInfo sensor is not depth sensor ({}:{})", FILENAME, __LINE__));
	}
	try
	{
		return sensor_.as<rs2::depth_sensor>().get_depth_scale();
	}
	catch (const rs2::error &e)
	{
		throw error::RecoverableError(
			fmt::format("sensor.as<rs2::depth_sensor>().get_depth_scale() failed {}({}): {} ({}:{})", 
				e.get_failed_function(), e.get_failed_args(), e.what(), FILENAME, __LINE__));
	}
}