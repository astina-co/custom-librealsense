#pragma once

#include <librealsense2/rs.hpp>

#include "frame.h"
#include "frameset.h"
#include "sensor.h"
#include "stream_config.h"
#include "utility.h"

namespace symb
{
namespace scanner
{

/**
 * @brief デバイスのストリームを管理
 */
class Streamer
{
public:
	/**
	 * @brief デフォルトコンストラクタ
	 */
	Streamer() = default;

	/**
	 * @brief コンストラクタ
	 *
	 * @param configs ストリームパラメータ群
	 * @param device ストリームを開始するrs2::deviceインスタンス
	 */
	Streamer(std::vector<StreamConfig> &configs, rs2::device &device) : ptr_(std::make_shared<StreamerPtr>(configs, device)){};

	/**
	 *@return ストリームが開始されたか
	 */
	bool started()
	{
		if (ptr_ == nullptr)
			throw std::runtime_error("Streamer is null");
		return ptr_->started();
	};

	/**
	 * @brief ストリームを開始する
	 */
	void start()
	{
		if (ptr_ == nullptr)
			throw std::runtime_error("Streamer is null");
		ptr_->start();
	}

	/**
	 * @brief ストリームを停止する
	 */
	void stop()
	{
		if (ptr_ == nullptr)
			throw std::runtime_error("Streamer is null");
		ptr_->stop();
	}

	/**
	 * @brief ストリームを閉鎖する / [※注意※] 他カメラの start, stop, get_frameset, get_intrinsics の実行を妨げるのでマルチスレッドの際は注意すること
	 */
	void close()
	{
		if (ptr_ == nullptr)
			throw std::runtime_error("Streamer is null");
		ptr_->close();
	}

	/**
	 * @brief 開放された状態のストリームからフレームを取得する
	 *
	 * @param count 連続取得を行うフレーム数
	 * @param timeout 撮影時間の上限(過ぎたらエラーが投げられる)
	 * @return Framesetインスタンス
	 */
	Frameset get_frameset(const int count, const long long timeout)
	{
		if (ptr_ == nullptr)
			throw std::runtime_error("Streamer is null");
		return ptr_->get_frameset(count, timeout);
	}

	/**
	 * @brief 開放された状態のストリームからフレームを取得する(Capture,Live用)
	 *
	 * @param count 連続取得を行うフレーム数
	 * @param timeout 撮影時間の上限(過ぎたらエラーが投げられる)
	 * @return Framesetインスタンス
	 */
	Frameset get_capture_frameset(const int count, const long long timeout)
	{
		if (ptr_ == nullptr)
			throw std::runtime_error("Streamer is null");
		return ptr_->get_capture_frameset(count, timeout);
	}

	/**
	 * @brief 開放中のセンサから内部パラメータを取得する
	 *
	 * @param stream_type 取得する内部パラメータのストリームタイプ
	 * @return 内部パラメータ
	 */
	rs2_intrinsics get_intrinsics(const rs2_stream stream_type)
	{
		if (ptr_ == nullptr)
			throw std::runtime_error("Streamer is null");
		return ptr_->get_intrinsics(stream_type);
	}

	Streamer &operator=(const Streamer &other)
	{
		if (this != &other)
		{
			ptr_ = other.ptr_;
		}
		return *this;
	}

	operator bool() { return ptr_ != nullptr; }

private:
	/**
	 * @brief Streamerクラスの共有ポインタクラス
	 */
	class StreamerPtr
	{
	public:
		/**
		 * @brief コンストラクタ
		 *
		 * @param configs ストリームパラメータ群
		 * @param device ストリームを開始するrs2::deviceインスタンス
		 */
		StreamerPtr(std::vector<StreamConfig> &configs, rs2::device &device);

		/**
		 * @brief デコンストラクタ / stop()およびclose()を呼び忘れた場合はここで自動的に呼ぶ
		 */
		~StreamerPtr();

		/**
		 *@return ストリームが開始されたか
		 */
		bool started() { return started_; }

		/**
		 * @brief ストリームを開始する
		 */
		void start();

		/**
		 * @brief ストリームを停止する
		 */
		void stop();

		/**
		 * @brief ストリームを閉鎖する / [※注意※] 他カメラの start, stop, get_frameset, get_intrinsics の実行を妨げるのでマルチスレッドの際は注意すること
		 */
		void close();

		/**
		 * @brief 開放された状態のストリームからフレームを取得する
		 *
		 * @param count 連続取得を行うフレーム数
		 * @param timeout 撮影時間の上限(過ぎたらエラーが投げられる)
		 * @return Framesetインスタンス
		 */
		Frameset get_frameset(const int count, const long long timeout);

		/**
		 * @brief 開放された状態のストリームからフレームを取得する(Capture,Live用)
		 *
		 * @param count 連続取得を行うフレーム数
		 * @param timeout 撮影時間の上限(過ぎたらエラーが投げられる)
		 * @return Framesetインスタンス
		 */
		Frameset get_capture_frameset(const int count, const long long timeout);

		/**
		 * @brief 開放中のセンサから内部パラメータを取得する
		 *
		 * @param stream_type 取得する内部パラメータのストリームタイプ
		 * @return 内部パラメータ
		 */
		rs2_intrinsics get_intrinsics(const rs2_stream stream_type);

	private:
		const std::vector<StreamConfig> config_; // ストリームパラメータ
		rs2::device device_; // ストリームを行うrs2::device
		std::vector<Sensor> sensors_; // 管理を行うSensorインスタンス群

		const int frame_buffer_size = 4; //!< フレームバッファのサイズで自由に設定可能
		// const int frame_buffer_size = 2; //!< フレームバッファのサイズで自由に設定可能
		rs2::syncer sync = rs2::syncer(frame_buffer_size); //!< センサ間でフレームの動機をとるためのハンドラ
		bool started_ = false; // ストリームが開始しているか

		bool dummy_color_stream_ = false;
		std::unique_ptr<rs2::video_stream_profile> dummy_color_profile_;

		rs2::frameset append_dummy_color_stream(const rs2::frameset &frame);
	};

	std::shared_ptr<StreamerPtr> ptr_ = nullptr; //!< ポインタ
};

} // namespace scanner

} // namespace symb
