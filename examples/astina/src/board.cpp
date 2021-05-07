#include "board.h"

#include <fstream>
#include <future>

#include <fmt/format.h>
#include <opencv2/imgcodecs.hpp>
#include <spdlog/spdlog.h>

#include "utility.h"

symb::scanner::CalibrationBoard::CalibrationBoard() : square_count_x(0), square_count_y(0), square_length(0), marker_length(0), marker_size(0), thickness(0) {}

bool symb::scanner::CalibrationBoard::generate(const int square_count_x, const int square_count_y, const float square_length, const float marker_length, const int marker_size, const float thickness)
{
	SPDLOG_TRACE("Generating Calibration Board ({}, {}, {}, {}, {}, {})", square_count_x, square_count_y, square_length, marker_length, marker_size, thickness);

	this->square_count_x = square_count_x;
	this->square_count_y = square_count_y;
	this->square_length = square_length;
	this->marker_length = marker_length;
	this->marker_size = marker_size;
	this->thickness = thickness;

	const bool stat = exec_async([&](int index) {
		SPDLOG_DEBUG("Generating ChAruCo Board {}/{}", index + 1, num_of_sides);
		dictionaries[index] = cv::aruco::Dictionary::create(square_count_x * square_count_y, marker_size, index);
		boards[index] = cv::aruco::CharucoBoard::create(square_count_x, square_count_y, square_length, marker_length, dictionaries[index]);
		SPDLOG_DEBUG("Generated ChAruCo Board {}/{}", index + 1, num_of_sides);

		return true;
	});

	SPDLOG_DEBUG("Generated Calibration Board");

	return stat;
}

bool symb::scanner::CalibrationBoard::load(const std::experimental::filesystem::path &dir)
{
	SPDLOG_DEBUG("Loading Calibration Board");

	const bool stat = exec_async([&](int index) {
		SPDLOG_DEBUG("Loading ChAruCo Board {}/{}", index + 1, num_of_sides);

		const std::experimental::filesystem::path filename = create_board_data_filepath(dir, index);

		// ファイルの存在確認をする
		if (!std::ifstream(filename).is_open())
		{
			SPDLOG_ERROR("Failure Loaded ChAruCo Board {}/{} {}", index + 1, num_of_sides, filename.string());
			return false;
		}

		// ファイルから読み込む
		cv::FileStorage fs(filename.string(), cv::FileStorage::READ);

		cv::Mat bytes_list;
		int max_correction_bits;
		fs["ByteList"] >> bytes_list;
		fs["MaxCorrectionBits"] >> max_correction_bits;

		fs["SquareCountX"] >> square_count_x;
		fs["SquareCountY"] >> square_count_y;
		fs["SquareLength"] >> square_length;
		fs["MarkerLength"] >> marker_length;
		fs["MarkerSize"] >> marker_size;
		fs["Thickness"] >> thickness;

		fs.release();

		dictionaries[index] = cv::makePtr<cv::aruco::Dictionary>(cv::aruco::Dictionary(bytes_list, marker_size, max_correction_bits));
		boards[index] = cv::aruco::CharucoBoard::create(square_count_x, square_count_y, square_length, marker_length, dictionaries[index]);

		SPDLOG_DEBUG("Loaded ChAruCo Board {}/{}", index + 1, num_of_sides);

		return true;
	});

	SPDLOG_DEBUG("Loaded Calibration Board");

	return stat;
}

bool symb::scanner::CalibrationBoard::save(const std::experimental::filesystem::path &dir)
{
	SPDLOG_DEBUG("Saving Calibration Board");

	utility::mkdir_if_not_exist(dir);

	const bool stat = exec_async([&](int index) {
		SPDLOG_DEBUG("Saving ChAruCo Board {}/{}", index + 1, num_of_sides);

		cv::FileStorage fs(create_board_data_filepath(dir, index).string(), cv::FileStorage::WRITE);
		const cv::Ptr<cv::aruco::Dictionary> &dictionary = dictionaries[index];

		fs << "ByteList" << dictionary->bytesList;
		fs << "MaxCorrectionBits" << dictionary->maxCorrectionBits;

		fs << "SquareCountX" << square_count_x;
		fs << "SquareCountY" << square_count_y;
		fs << "SquareLength" << square_length;
		fs << "MarkerLength" << marker_length;
		fs << "MarkerSize" << marker_size;
		fs << "Thickness" << thickness;

		fs.release();

		SPDLOG_DEBUG("Saved ChAruCo Board {}/{}", index + 1, num_of_sides);

		return true;
	});

	SPDLOG_DEBUG("Saved Calibration Board");

	return stat;
}

bool symb::scanner::CalibrationBoard::save_image(const std::experimental::filesystem::path &dir, int square_length, float margin)
{
	SPDLOG_DEBUG("Saving Calibration Board Image");

	utility::mkdir_if_not_exist(dir);

	const bool stat = exec_async([&](int index) {
		SPDLOG_DEBUG("Saving ChAruCo Board Image {}/{}", index + 1, num_of_sides);

		const int margin_pixel = static_cast<int>(static_cast<float>(square_length) - static_cast<float>(square_length) * margin);
		const cv::Size size(square_count_x * square_length + 2 * margin_pixel, square_count_y * square_length + 2 * margin_pixel);
		cv::Mat image;
		boards[index]->draw(size, image, margin_pixel);
		cv::imwrite(create_board_image_filepath(dir, index).string(), image, {cv::IMWRITE_PNG_COMPRESSION, 0});

		SPDLOG_DEBUG("Saved ChAruCo Board Image {}/{}", index + 1, num_of_sides);

		return true;
	});

	SPDLOG_DEBUG("Saved Calibration Board Image");

	return stat;
}

bool symb::scanner::CalibrationBoard::is_valid() const
{
	bool valid = true;

	for (auto &&board : boards)
	{
		valid &= !board->dictionary->bytesList.empty();
	}

	return valid;
}

bool symb::scanner::CalibrationBoard::exec_async(std::function<bool(int index)> fn)
{
	std::future<bool> futures[num_of_sides];

	// スレッドを生成する
	for (int i = 0; i < num_of_sides; i++)
	{
		futures[i] = std::async(std::launch::async, fn, i);
	}

	// スレッドが終了するまで待つ
	bool stat = true;

	for (auto &&future : futures)
	{
		stat &= future.get();
	}

	return stat;
}

std::experimental::filesystem::path symb::scanner::CalibrationBoard::create_board_data_filepath(const std::experimental::filesystem::path &dir, int side_index)
{
	return dir / fmt::format("board{}.xml", side_index + 1);
}

std::experimental::filesystem::path symb::scanner::CalibrationBoard::create_board_image_filepath(const std::experimental::filesystem::path &dir, int side_index)
{
	return dir / fmt::format("board{}.png", side_index + 1);
}
