#pragma once

#include <librealsense2/rs.hpp>

namespace symb
{
namespace scanner
{

/**
 * @brief デプスカメラのプリセット設定
 */
namespace Preset
{
/**
 * @brief デバイスのデプスカメラのプリセットを設定する
 * 
 * @param device 設定を行うデバイス
 * @param json 設定に使用するjsonの文字列
 */
void set_preset(const rs2::device &device, const std::string &json);

/**
 * @brief デバイスのデプスカメラのデプステーブルの設定をする
 *
 * @param device 設定を行うデバイス
 * @param depth_unit
 * @param disparity_shift
 */
void set_depth_table(const rs2::device &device, const unsigned int depth_unit, const unsigned int disparity_shift);

/**
 * @brief デバイスのデプスカメラのプリセットを取得する
 * 
 * @param device 取得するデバイス
 */
std::string get_preset(const rs2::device &device);

}

} // namespace scanner

} // namespace symb
