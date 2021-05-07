#pragma once

#include "spdlog/spdlog.h"
#include <librealsense2/rs.hpp>
#include <map>
#include <queue>
#include <thread>

#include "camera_info.h"

namespace symb
{
namespace scanner
{

/**
 * @brief デバイスの接続状況を管理するクラス
 */
class DeviceManager
{
public:
	/**
	 * @brief コンストラクタ / 接続ハンドラスレッドが開始する
	 */
	DeviceManager();

	/**
	 * @brief デストラクタ
	 */
	~DeviceManager();

	/**
	 * @brief 指定のデバイスのリセットを行う
	 *
	 * @param serials リセットをするデバイスのシリアル番号のベクトル / 指定がない場合は接続しているものすべてをリセットする
	 */
	void reset(const std::vector<std::string> &serials = std::vector<std::string>());

	/**
	 * @brief シリアル番号からデバイスが新規接続かの情報を取得する
	 *
	 * @param serial シリアル番号
	 * @return 新規接続か否か
	 */
	bool is_fresh(const std::string &serial);

	/**
	 * @brief シリアル番号からデバイスの接続状況を取得する
	 *
	 * @param serial シリアル番号
	 * @return 接続状況
	 */
	bool is_connected(const std::string &serial);

	/**
	 * @brief カメラ情報からrs2::deviceインスタンスを取得する
	 *
	 * @param serial シリアル番号
	 * @return rs2::deviceのインスタンス
	 */
	rs2::device &get_rs2_device(const std::string &serial);

	void stop();

	unsigned long long get_system_exception_count()
	{ 
		std::lock_guard<std::mutex> _(gse_);
		return system_exception_cnt_;
	}

private:

	std::mutex gse_;
	std::mutex se_;
	unsigned long long system_exception_cnt_ = 0;

	std::thread handler_;							//!< 接続をリアルタイムで管理
	std::mutex m_;									//!< events_へのアクセス競合防止
	std::queue<rs2::event_information> events_;		//!< contextからハンドラへ接続状況を流すqueue
	std::atomic<bool> alive_ = true;				//!< ハンドラ停止用

	std::mutex n_;									//!< devices_へのアクセス競合防止
	std::map<std::string, rs2::device> devices_;	//!< デバイスのシリアル番号と対応するrs2::deviceインスタンスのマップ
	std::map<std::string, bool> fresh_devices_;	    //!< 新規で追加されたデバイス, is_connected() または get_rs2_device() でfalseに変わる

	std::atomic <bool> reset_all_ = false;			//!< reset()メソッドからハンドラへリセットを指示するためのフラグ
	std::mutex o_;									//!< reset_serials_へのアクセス競合防止
	std::vector<std::string> reset_serials_;		//!< (reset_all_==true)時にリセットするデバイスのシリアル番号群
};

} // namespace scanner

} // namespace symb
