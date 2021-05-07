#include "preset.h"

#include <fmt/format.h>
#include <librealsense2/rs_advanced_mode.hpp>
#include "exception.h"
#include "defines.h"

void symb::scanner::Preset::set_preset(const rs2::device &device, const std::string &json)
{
	const int tries = 3; // 1回では失敗する可能性があるので複数回やってからエラーを投げる (3回じゃなくても良い)
	for (int i = 0; i < tries; i++)
	{
		try
		{
			if (!device.is<rs400::advanced_mode>())
			{
				device.as<rs400::advanced_mode>().toggle_advanced_mode(true);
			}
			rs400::advanced_mode advanced_device = device.as<rs400::advanced_mode>();
			advanced_device.load_json(json);
			break;
		}
		catch (const rs2::error &e)
		{
			if (i == tries - 1)
			{
				throw error::RecoverableError(fmt::format("set_preset failed {}({}):{} ({}:{})", e.get_failed_function(), e.get_failed_args(), e.what(), FILENAME, __LINE__));
			}
		}
	}
}

void symb::scanner::Preset::set_depth_table(const rs2::device &device, const unsigned int depth_unit, const unsigned int disparity_shift)
{
	const int tries = 3; // 1回では失敗する可能性があるので複数回やってからエラーを投げる (3回じゃなくても良い)
	for (int i = 0; i < tries; i++)
	{
		try
		{
			if (!device.is<rs400::advanced_mode>())
			{
				device.as<rs400::advanced_mode>().toggle_advanced_mode(true);
			}
			rs400::advanced_mode advanced_device = device.as<rs400::advanced_mode>();
			STDepthTableControl depth_table = advanced_device.get_depth_table();
			depth_table.depthUnits = depth_unit;
			depth_table.disparityShift = disparity_shift;
			advanced_device.set_depth_table(depth_table);
			break;
		}
		catch (const rs2::error &e)
		{
			if (i == tries - 1)
			{
				throw error::RecoverableError(fmt::format("set_depth_table failed {}({}):{} ({}:{})", e.get_failed_function(), e.get_failed_args(), e.what(), FILENAME, __LINE__));
			}
		}
	}
}

std::string symb::scanner::Preset::get_preset(const rs2::device &device)
{
	const int tries = 3; // 1回では失敗する可能性があるので複数回やってからエラーを投げる (3回じゃなくても良い)
    std::string preset_data;
	for (int i = 0; i < tries; i++)
	{
		try
		{
			if (!device.is<rs400::advanced_mode>())
			{
				device.as<rs400::advanced_mode>().toggle_advanced_mode(true);
			}
			rs400::advanced_mode advanced_device = device.as<rs400::advanced_mode>();
			preset_data = advanced_device.serialize_json();
			std::cout << preset_data << std::endl;

			break;
		}
		catch (const rs2::error &e)
		{
			if (i == tries - 1)
			{
				throw error::RecoverableError(fmt::format("get_preset failed {}({}):{} ({}:{})", e.get_failed_function(), e.get_failed_args(), e.what(), FILENAME, __LINE__));
			}
		}
	}
	return preset_data;
}
