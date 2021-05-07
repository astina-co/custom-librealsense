#pragma once

#include "fmt/format.h"
#include <librealsense2/rs.hpp>
#include <mutex>
#include <map>

#include "utility.h"

namespace symb
{
namespace scanner
{

/**
 * @brief センサストリーム開放用のストリームパラメータ
 */
class StreamConfig
{
public:
	/**
	 * @brief コンストラクタ / 使用可能な組み合わせはURL参照(https://bit.ly/34vRVJD)
	 *
	 * @param stream ストリームタイプ (Color, Depth, Infrared 等)
	 * @param format データフォーマット (RGB8, RAW16, Y16, BGRA8, RGBA8, BGR8, YUYV, Y10BPACK, RAW10, Y8, UYVY, Z16)
	 * @param fps データ取得FPS (100, 90, 60, 30, 15, 6)
	 * @param width フレーム横幅
	 * @param height フレーム縦幅
	 * @param index ストリームインデックス(赤外線ストリームが複数種類あったりするので)
	 * @param dummy ダミーフラグ（帯域幅の問題でストリームを開く代わりに使う。原理的には無しでいいが、rs2::alignを使うため＆既存のコードへの影響範囲を減らすために作った。カラー専用）
	 */
	StreamConfig(const rs2_stream stream, const rs2_format format, const int fps, const int width, const int height, const int index = 0, const bool dummy = false);

	const rs2_stream stream; //!< ストリームタイプ
	const rs2_format format; //!< データフォーマット
	const int fps; //!< データ取得FPS
	const int width; //!< フレーム横幅
	const int height; //!< フレーム縦幅
	const int index; //!< ストリームインデックス
	const bool dummy; //!< ダミーフラグ
	const std::string name =
		fmt::format("StreamConfig(Type: {}, Format: {}, FPS: {}, Resolution: {}x{}, Index: {})", rs2_stream_to_string(stream), rs2_format_to_string(format), fps, width, height, index); //!< デバッグ用文字列

	/**
	 * @brief sensorに基づいてrs2::stream_profileに変換する
	 * 
	 * @param sensor プロファイル検索用センサ
	 * @return rs2::sensorのopen()に使われるrs2::stream_profile
	 */
	rs2::stream_profile to_profile(const rs2::sensor &sensor) const;
};

} // namespace scanner

} // namespace symb
