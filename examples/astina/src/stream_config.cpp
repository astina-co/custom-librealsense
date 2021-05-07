#include "stream_config.h"

symb::scanner::StreamConfig::StreamConfig(const rs2_stream stream, const rs2_format format, const int fps, const int width, const int height, const int index, const bool dummy)
	: stream(stream), format(format), fps(fps), width(width), height(height), index(index), dummy(dummy)
{
	if (dummy) {
		assert(stream == RS2_STREAM_COLOR);
	}
}

rs2::stream_profile symb::scanner::StreamConfig::to_profile(const rs2::sensor &sensor) const
{
	return static_cast<rs2::stream_profile>(utility::get_profile(sensor, stream, format, fps, width, height, index));
}