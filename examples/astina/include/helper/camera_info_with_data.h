#pragma once

#include "calibration.h"

#include "camera_info.h"
#include "frameset.h"

namespace symb
{
namespace scanner
{
namespace helper
{

/**
 * @brief ユーザ管理用のデバイス情報(データ付き)
 */
class CameraInfoWithData : public CameraInfo
{
public:
	/**
	 * @brief コンストラクタ
	 *
	 * @param serial カメラのシリアル番号
	 * @param id ユーザ管理用ID
	 * @param location ユーザ管理用カメラ位置
	 * @param level ユーザ管理用カメラ高さ
	 */
	CameraInfoWithData(const std::string &serial, const std::string &id, const std::string &location, const int level) : CameraInfo(serial, id, location, level){};

	CalibrationData calibration; //!< キャリブレーションデータ
	Frameset frameset; //!<	撮影Frameset
	float depth_scale = 0; //!< 撮影スケール
	rs2_intrinsics intrinsics; //!< カメラの内部パラメータ
};

} // namespace helper

} // namespace scanner

} // namespace symb
