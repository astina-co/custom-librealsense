#pragma once
#include <filesystem>

#include <opencv2/opencv.hpp>

#include "helper/camera_info_with_data.h"
#include "exception.h"

namespace symb
{
namespace scanner
{
namespace helper
{

/**
 * @brief カメラ情報を外部から読み込む
 */
class CameraInfoReader
{
public:
	/**
	 * @brief jsonファイルからカメラ情報を読み込む
	 * 
	 * @param json_path jsonファイルのパス
	 */
	CameraInfoReader(const std::experimental::filesystem::path &json_path)
	{ 
		if (!json_path.has_extension())
		{
			throw error::InvalidParameterError("Json Invalid file path");
		}

		if (json_path.extension() != ".json")
		{
			throw error::InvalidParameterError("Json Invalid file extension [json]");
		}

		fs.open(json_path.string(), cv::FileStorage::READ | cv::FileStorage::FORMAT_JSON);
		if (!fs.isOpened())
		{
			throw error::BadStateError(fmt::format("Unable to open Json File {}", json_path.string()));
		}

		if (!std::experimental::filesystem::exists(json_path))
		{
			throw error::InvalidParameterError(fmt::format("Json File {} does not exist", json_path.string()));
		}

		cv::FileNode root = fs["cameras"];
		const size_t size = root.size();
		for (unsigned long i = 0ul; i < size; i++)
		{
			cv::FileNode fn = root[i];
			std::string serial, id, location;
			int level;
			fn["serial"] >> serial;
			fn["id"] >> id;
			fn["location"] >> location;
			fn["level"] >> level;
			result.emplace_back(std::make_shared<CameraInfoWithData>(serial, id, location, level));
		}
	}

	/**
	 * @brief デストラクタ
	 */
	~CameraInfoReader()
	{
		fs.release();
	}

	/**
	 * @brief 読み込んだカメラ情報を取得
	 * 
	 * @return カメラ情報群
	 */
	std::vector<std::shared_ptr<CameraInfoWithData>> getResult()
	{ 
		return result;
	}

private:
	cv::FileStorage fs;
	std::vector<std::shared_ptr<CameraInfoWithData>> result;
};

} // namespace helper

} // namespace scanner

} // namespace symb
