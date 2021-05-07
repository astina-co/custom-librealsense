#include "frameset.h"
#include <exception.h>

#include "defines.h"
#include "spdlog/spdlog.h"

std::vector<symb::scanner::Frame> symb::scanner::Frameset::vector() const
{
	if (!initialized_)
	{
		throw error::BadStateError(fmt::format("Frameset is Empty ({}:{})", FILENAME, __LINE__));
	}
	return frames_;
}

size_t symb::scanner::Frameset::size() const
{
	if (!initialized_)
	{
		throw error::BadStateError(fmt::format("Frameset is Empty ({}:{})", FILENAME, __LINE__));
	}
	return frames_.size();
}

symb::scanner::Frame symb::scanner::Frameset::get_filtered() const
{
	if (!initialized_)
	{
		throw error::BadStateError(fmt::format("Frameset is Empty ({}:{})", FILENAME, __LINE__));
	}
	const size_t frame_count = frames_.size();

	if (frame_count <= 0)
	{
		throw error::BadStateError(fmt::format("Frameset size is lesser than zero ({}:{})", FILENAME, __LINE__));
	}

	const rs2::disparity_transform depth_to_disparity(true); //!<
	const rs2::disparity_transform disparity_to_depth(false); //!<
	// const rs2::decimation_filter decimation_filter(4.0f); //!<
	const rs2::spatial_filter spatial_filter(0.71f, 25, 5.0f, 0); //!<
	const rs2::temporal_filter temporal_filter(0.05f, 96, frame_count <= 2 ? 5 : (frame_count <= 5 ? 5 : (frame_count <= 8 ? 7 : 8))); //!<
	const rs2::threshold_filter threshold_filter(0.16f, 1.4f); //!<

	rs2::frameset rs2_frameset;

	for (const Frame &new_frameset : frames_)
	{
		if (!new_frameset.from_stream(rs2_stream::RS2_STREAM_DEPTH))
		{
			throw error::BadStateError(fmt::format("Frame does not contain depth data at filter ({}:{})", FILENAME, __LINE__));
		}

		rs2_frameset = std::move(new_frameset);
		try
		{
			rs2_frameset = std::move(depth_to_disparity.process(rs2::frame(std::move(rs2_frameset))));
			rs2_frameset = std::move(spatial_filter.process(rs2::frame(std::move(rs2_frameset))));
			rs2_frameset = std::move(temporal_filter.process(rs2::frame(std::move(rs2_frameset))));
			rs2_frameset = std::move(disparity_to_depth.process(rs2::frame(std::move(rs2_frameset))));
		}
		catch (const rs2::error &e)
		{
			throw error::UnRecoverableError(fmt::format("Frameset Unable to filter frameset [{}({}) {}] ({}:{})", 
				e.get_failed_function(), e.get_failed_args(), e.what(), FILENAME, __LINE__));
		}
	}

	try
	{
		rs2_frameset = std::move(threshold_filter.process(rs2::frame(std::move(rs2_frameset))));
	}
	catch (const rs2::error &e)
	{
		throw error::UnRecoverableError(fmt::format("Frameset Unable to filter frameset [{}({}) {}] ({}:{})", 
			e.get_failed_function(), e.get_failed_args(), e.what(), FILENAME, __LINE__));
	}

	// return Frame(rs2_frameset);
	return Frame(rs2::align(RS2_STREAM_COLOR).process(rs2_frameset));
}

symb::scanner::Frame symb::scanner::Frameset::get_aligned() const
{
	if (!initialized_)
	{
		SPDLOG_ERROR(fmt::format("Frameset is Empty ({}:{})", FILENAME, __LINE__));
		throw error::BadStateError(fmt::format("Frameset is Empty ({}:{})", FILENAME, __LINE__));
	}
	const size_t frame_count = frames_.size();

	if (frame_count <= 0)
	{
		SPDLOG_ERROR(fmt::format("Frameset size is lesser than zero ({}:{})", FILENAME, __LINE__));
		throw error::BadStateError(fmt::format("Frameset size is lesser than zero ({}:{})", FILENAME, __LINE__));
	}

	rs2::frameset rs2_frameset;

	for (const Frame &new_frameset : frames_)
	{
		if (!new_frameset.from_stream(rs2_stream::RS2_STREAM_DEPTH))
		{
			SPDLOG_ERROR(fmt::format("Frame does not contain depth data at filter ({}:{})", FILENAME, __LINE__));
			throw error::BadStateError(fmt::format("Frame does not contain depth data at filter ({}:{})", FILENAME, __LINE__));
		}

		rs2_frameset = std::move(new_frameset);
	}

	return Frame(rs2::align(RS2_STREAM_COLOR).process(rs2_frameset));
}
