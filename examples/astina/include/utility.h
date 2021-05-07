#pragma once

#include <ctime>
#include <filesystem>
#include <librealsense2/rs.hpp>
#include <memory>
#include <sstream>
#include <map>

#include <Open3D/Geometry/Pointcloud.h>

namespace symb
{
namespace scanner
{

/**
 * @brief ユーティリティ
 */
namespace utility
{

/**
 * @brief 一次元の範囲変数
 */
struct Range1Df
{
	float min;
	float max;
};

/**
 * @brief 三次元の範囲変数
 */
struct Range3Df
{
	Range1Df x;
	Range1Df y;
	Range1Df z;
};

/**
 * @brief ディレクトリであるか判定する
 *
 * @param path
 * @return ディレクトリであるか
 */
bool is_dir(const std::experimental::filesystem::path &path);

/**
 * @brief フォルダがない場合はフォルダを作成する
 *
 * @param path フォルダを作成するパス
 * @return 正常に完了したか
 */
bool mkdir_if_not_exist(const std::experimental::filesystem::path &path) noexcept;

/**
 * @brief 日付時間の文字列を取得する
 *
 * @return 日付時間の文字列
 */
std::string get_date_time();

/**
 * @brief 文字列(find_in)中の文字列(target)を探索する
 *
 * @param find_in
 * @paran target
 * @return 探索に成功したか
 */
bool contains(const std::string &find_in, const std::string &target);

/**
 * @brief 文字列を結合する
 * 
 * @param str_list 結合する文字列のリスト
 * @param out 結合した文字列
 * @return std::string 
 */
std::string join(const std::vector<std::string> &str_list, std::string &out);

/**
 * @brief セパレータ文字列で文字列を分割する
 *
 * @param str 分割する文字列
 * @param separator セパレータ文字列
 * @return 分割した文字列の配列
 */
std::vector<std::string> split(const std::string &str, const std::string &separator);

/**
 * @brief 指定のデバイスのRGBストリームを含むセンサを取得
 *
 * @param device 指定のデバイス
 * @return RGBストリームを含むセンサ
 */
rs2::sensor get_color_sensor(const rs2::device &device);

/**
 * @brief センサインスタンスとストリームパラメータからrs2::video_stream_profileを取得
 *
 * @param sensor rs2::sensorインスタンスs
 * @param stream_type ストリームタイプ
 * @param format データフォーマット
 * @param fps データ取得FPS
 * @param width フレーム横幅
 * @param height フレーム縦幅
 * @param index ストリームインデックス
 * @return sensor.open()用のrs2::video_stream_profileインスタンス
 */
rs2::video_stream_profile get_profile(const rs2::sensor &sensor, const rs2_stream stream_type, const rs2_format format, const int fps, const int width, const int height, const int index = 0);

Eigen::Vector3d transform_single(const Eigen::Matrix4d &matrix, const Eigen::Vector3d &point);

std::vector<Eigen::Vector3d> transform(const Eigen::Matrix4d &matrix, const std::vector<Eigen::Vector3d> &pointcloud);

std::tuple<bool, int> coord2id(const int x, const int y, const int w, const int h);

std::tuple<bool, Eigen::Vector3d> get_point(const int id, const std::pair<std::vector<int>, std::vector<Eigen::Vector3d>> &points);

Eigen::Vector3d sum(const std::vector<Eigen::Vector3d> &points);

Eigen::Vector3d mean(const std::vector<Eigen::Vector3d> &points);

Eigen::MatrixXd convert_pointcloud_to_mat(const std::vector<Eigen::Vector3d> &pointcloud);

std::pair<std::vector<int>, std::vector<Eigen::Vector3d>> merge_points(const std::map<int, std::vector<Eigen::Vector3d>> &map);

Eigen::Vector3d id_to_color(const std::string &id);

std::string matrix_to_string(const Eigen::MatrixXd &matrix);

} // namespace utility

} // namespace scanner

} // namespace symb
