#pragma once

#include <filesystem>

#include <opencv2/aruco/charuco.hpp>

namespace symb
{
namespace scanner
{

/**
 * @brief キャリブレーションボードの面
 */
enum struct Side
{
	Front, //!< 表側
	Rear, //!< 裏側
};

/**
 * @brief キャリブレーションボード
 *
 * 板の両面にそれぞれ異なるChAruCoマーカーが印刷された板（キャリブレーションボード）を表す。
 */
class CalibrationBoard
{
public:
	/**
	 * @brief コンストラクタ
	 */
	CalibrationBoard();

	/**
	 * @brief キャリブレーションボード情報を生成する
	 *
	 * @param squaresX X方向のチェスボードの四角の数
	 * @param squaresY Y方向のチェスボードの四角の数
	 * @param squareLength チェスボードの四角の一辺の長さ [m]
	 * @param markerLength マーカーの四角の一辺の長さ [m]
	 * @param marker_size マーカー内の一辺のセルの数
	 * @param thickness ボードの厚さ[m]
	 * @return 成功したか
	 */
	bool generate(const int square_count_x, const int square_count_y, const float square_length, const float marker_length, const int marker_size, const float thickness);

	/**
	 * @brief キャリブレーションボード情報を読み込む
	 *
	 * @param dir 読み込むディレクトリパス
	 * @return 成功したか
	 */
	bool load(const std::experimental::filesystem::path &dir);

	/**
	 * @brief キャリブレーションボード情報を保存する
	 *
	 * @param dir 保存先のディレクトリパス
	 * @return 成功したか
	 */
	bool save(const std::experimental::filesystem::path &dir);

	/**
	 * @brief キャリブレーションボード画像を保存する
	 *
	 * @param dir 保存先のディレクトリパス
	 * @param square_length チェスボードの四角の一辺の長さ [単位:Pixel]
	 * @param margin チェスボードの四角の中の、マーカーのサイズの割合 0.0 < x <= 1.0
	 * @return 成功したか
	 */
	bool save_image(const std::experimental::filesystem::path &dir, int square_length = 200, float margin = 0.6);

	/**
	 * @brief 正しいキャリブレーションボードかを判定する
	 *
	 * @return 正しいキャリブレーションボードか
	 */
	bool CalibrationBoard::is_valid() const;

	/**
	 * @brief ChAruCoマーカー辞書を取得する
	 *
	 * @param side 取得する面
	 * @return ChAruCoマーカー辞書
	 */
	cv::Ptr<cv::aruco::Dictionary> get_dictionary(Side side) const { return dictionaries[static_cast<int>(side)]; }

	/**
	 * @brief ChAruCoボードを取得する
	 *
	 * @param side 取得するキャリブレーションボードの面
	 * @return ChAruCoボード
	 */
	cv::Ptr<cv::aruco::CharucoBoard> get_board(Side side) const { return boards[static_cast<int>(side)]; }

	/**
	 * @brief X方向のチェスボードの四角の数を取得する
	 *
	 * @return X方向のチェスボードの四角の数
	 */
	inline int get_square_count_x() const { return square_count_x; }

	/**
	 * @brief Y方向のチェスボードの四角の数を取得する
	 *
	 * @return Y方向のチェスボードの四角の数
	 */
	inline int get_square_count_y() const { return square_count_y; }

	/**
	 * @brief チェスボードの四角の一辺の長さを取得する
	 *
	 * @return チェスボードの四角の一辺の長さ [m]
	 */
	inline float get_square_length() const { return square_length; }

	/**
	 * @brief マーカーの四角の一辺の長さを取得する
	 *
	 * @return マーカーの四角の一辺の長さ [m]
	 */
	inline int get_marker_size() const { return marker_size; }

	/**
	 * @brief マーカー内の一辺のセルの数を取得する
	 *
	 * @return マーカー内の一辺のセルの数
	 */
	inline float get_marker_length() const { return marker_length; }

	/**
	 * @brief キャリブレーションボードの厚さ
	 *
	 * @return キャリブレーションボードの厚さ [m]
	 */
	inline float get_thickness() const { return thickness; }

private:
	/**
	 * @brief 非同期でコールバック関数を処理する
	 *
	 * @param fn キャリブレーションボードを処理するコールバック
	 * @return 成功したか
	 */
	static bool exec_async(std::function<bool(int index)> fn);

	/**
	 * @brief ChAruCoマーカー辞書のファイルパスを作成する
	 *
	 * @param dir 保存ディレクトリパス
	 * @param side_index キャリブレーションボードの面
	 * @return ファイルパス
	 */
	static std::experimental::filesystem::path create_board_data_filepath(const std::experimental::filesystem::path &dir, int side_index);

	/**
	 * @brief ChAruCoマーカー画像のファイルパスを作成する
	 *
	 * @param dir 保存ディレクトリパス
	 * @param side_index キャリブレーションボードの面
	 * @return ファイルパス
	 */
	static std::experimental::filesystem::path create_board_image_filepath(const std::experimental::filesystem::path &dir, int side_index);

	int square_count_x; //!< X方向のチェスボードの四角の数
	int square_count_y; //!< Y方向のチェスボードの四角の数
	float square_length; //!< チェスボードの四角の一辺の長さ [単位:m]
	float marker_length; //!< マーカーの四角の一辺の長さ [単位:m]
	int marker_size; //!< マーカー内の一辺のセルの数
	float thickness; //!< ボードの厚さ

	static const int num_of_sides = 2; //!< キャリブレーションボードの面数

	cv::Ptr<cv::aruco::Dictionary> dictionaries[num_of_sides]; //!< ChAruCoマーカー辞書
	cv::Ptr<cv::aruco::CharucoBoard> boards[num_of_sides]; //!< ChAruCoボード
};

} // namespace scanner

} // namespace symb