#pragma once

#include <fmt/format.h>
#include <string>

#include "board.h"

namespace symb
{
namespace scanner
{

/**
 * @brief ユーザ管理用のデバイス情報
 */
class CameraInfo
{
public:
	/**
	 * @brief コンストラクタ (コピー用)
	 *
	 * @param camera_info カメラ情報
	 */
	CameraInfo(const CameraInfo& camera_info);

	/**
	 * @brief コンストラクタ
	 *
	 * @param serial カメラのシリアル番号
	 * @param id ユーザ管理用ID
	 * @param location ユーザ管理用カメラ位置
	 * @param level ユーザ管理用カメラ高さ
	 */
	CameraInfo(const std::string &serial, const std::string &id, const std::string &location, const int level);

	/**
	 * @brief カメラのシリアル番号を取得
	 *
	 * @return カメラのシリアル番号
	 */
	std::string serial() const { return serial_;  }

	/**
	 * @brief カメラのユーザ管理用IDを取得
	 *
	 * @return カメラのユーザ管理用ID
	 */
	std::string id() const { return id_; }

	/**
	 * @brief カメラのユーザ管理用位置を取得
	 *
	 * @return カメラのユーザ管理用位置
	 */
	std::string location() const { return location_; }

	/**
	 * @brief カメラのユーザ管理用高さを取得
	 *
	 * @return カメラのユーザ管理用高さ
	 */
	int level() const { return level_; }

	/**
	 * @brief カメラのユーザ管理用デバッグ名を取得
	 *
	 * @return カメラのユーザ管理用デバッグ名
	 */
	std::string name() const { return fmt::format("Camera({},{},{},{})", serial_, id_, location_, level_); }

	/**
	 * @brief カメラの使用キャリブレーションボード面を取得
	 *
	 * @return カメラの使用キャリブレーションボード面
	 */
	Side side() const { return (id_[0] == 'A' || id_[0] == 'D') ? Side::Front : Side::Rear; }

private:
	std::string serial_; //!< カメラのシリアル番号
	std::string id_; //!< カメラのユーザ管理用ID
	std::string location_; //!< カメラのユーザ管理用位置
	int level_; //!< カメラのユーザ管理用高さ
};

inline CameraInfo::CameraInfo(const CameraInfo & camera_info) : serial_(camera_info.serial_), id_(camera_info.id_), location_(camera_info.location_), level_(camera_info.level_)
{
}

inline CameraInfo::CameraInfo(const std::string &serial, const std::string &id, const std::string &location, const int level): serial_(serial), id_(id), location_(location), level_(level)
{
}

} // namespace scanner

} // namespace symb
