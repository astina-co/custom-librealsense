#pragma once

#include "fmt/format.h"
#include "spdlog/spdlog.h"

#include <librealsense2/rs.hpp>

namespace symb
{
namespace scanner
{

/**
 * @brief デバイスのセンサの設定を取得・改変
 */
class SensorInfo
{
public:
	/**
	 * @brief コンストラクタ
	 * 
	 * @param sensor 対象のrs2::sensorインスタンス
	 */
	SensorInfo(const rs2::sensor &sensor) : sensor_(sensor){};

	/**
	 * @brief 設定を取得する
	 *
	 * @param option 取得する情報のオプション
	 * @return 取得結果
	 */
	float get_option(const rs2_option option) const;

	/**
	 * @brief 設定を改変する
	 *
	 * @param option 改変する情報のオプション
	 * @param 改変後の値
	 */
	void set_option(const rs2_option option, const float value);

	/**
	 * @return センサがデプスセンサか
	 */
	bool is_depth_sensor() const;

	/**
	 * @return デプスセンサの撮影スケール
	 */
	float get_depth_scale() const;

private:
	const rs2::sensor sensor_; //!< rs2::sensorインスタンス
};

} // namespace scanner

} // namespace symb
