#pragma once

#include <librealsense2/rs.hpp>
#include <spdlog/spdlog.h>

#include "stream_config.h"

namespace symb
{
namespace scanner
{

/**
 * @brief デバイスのセンサ
 */
class Sensor
{
public:

	/**
	 * @brief コピーコンストラクタ
	 * 
	 * @param sensor コピーを行うSensorインスタンス
	 */
	Sensor(const Sensor &sensor) : ptr_(sensor.ptr_){};

	/**
	 * @brief コンストラクタ
	 * 
	 * @param sensor 管理を行うセンサ
	 * @param configs センサ設定群
	 */
	Sensor(const rs2::sensor &sensor, const std::vector<StreamConfig> &configs) : ptr_(std::make_shared<SensorPtr>(sensor, configs)){};

	/**
	 * @brief rs2::sensorインスタンスを取得
	 * 
	 * @return rs2::sensorインスタンス
	 */
	rs2::sensor get()
	{
		if (ptr_ == nullptr)
		{
			throw std::runtime_error("Sensor is null");
		}
		return ptr_->get();
	}

	/**
	 * @brief ストリームを開放する
	 */
	void open()
	{
		if (ptr_ == nullptr)
		{
			throw std::runtime_error("Sensor is null");
		}
		ptr_->open();
	};

	/**
	 * @brief ストリームを閉鎖する
	 */
	void close()
	{
		if (ptr_ == nullptr)
		{
			throw std::runtime_error("Sensor is null");
		}
		ptr_->close();
	};

	/**
	 * @brief センサの特定のストリームの内部パラメータを取得する
	 * 
	 * @param stream ストリーム
	 */
	rs2_intrinsics get_intrinsics(const rs2_stream stream)
	{
		if (ptr_ == nullptr)
		{
			throw std::runtime_error("Sensor is null");
		}
		return ptr_->get_intrinsics(stream);
	}

	Sensor &operator=(const Sensor &other)
	{
		if (this != &other)
		{
			ptr_ = other.ptr_;
		}
		return *this;
	}

private:
	class SensorPtr
	{
	public:
		/**
		 * @brief コンストラクタ
		 *
		 * @param sensor 管理を行うセンサ
		 * @param configs センサ設定群
		 */
		SensorPtr(const rs2::sensor &sensor, const std::vector<StreamConfig> &configs);

		/**
		 * @brief デコンストラクタ
		 */
		~SensorPtr();

		/**
		 * @brief rs2::sensorインスタンスを取得
		 *
		 * @return rs2::sensorインスタンス
		 */
		rs2::sensor get() { return rs2_sensor_; }

		/**
		 * @brief ストリームを開放する
		 */
		void open();

		/**
		 * @brief ストリームを閉鎖する
		 */
		void close();

		/**
		 * @brief センサの特定のストリームの内部パラメータを取得する
		 *
		 * @param stream ストリーム
		 */
		rs2_intrinsics get_intrinsics(const rs2_stream stream);

	private:
		bool opened = false; //!< ストリームが開放されているか
		std::vector<rs2::stream_profile> profiles_; // ストリーム開放時に使用するプロファイル群
		rs2::sensor rs2_sensor_; // rs2::sensorインスタンス
	};

	std::shared_ptr<SensorPtr> ptr_; //!< ポインタ
};

} // namespace scanner

} // namespace symb
