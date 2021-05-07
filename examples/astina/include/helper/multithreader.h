#pragma once

#include "camera_info_with_data.h"
#include "exception.h"
#include <vector>

namespace symb
{
namespace scanner
{
namespace helper
{

/**
 * @brief 複数デバイスの同時使用を手助けするクラス
 * 
 * 使用方法はcapture.cppまたはendurance.cpp参照
 */
class MultiThreader
{
public:

	/**
	 * @brief コンストラクタ
	 * 
	 * @param objs 処理対象のカメラ群
	 * @param lambda 処理を行う関数
	 * @param timeout 実行タイムアウト(デフォルト:30秒) / タイムアウトが過ぎたらエラーとして扱われるが、実行は続く
	 */
	MultiThreader(std::vector<std::shared_ptr<CameraInfoWithData>> &objs, std::function<void(std::shared_ptr<CameraInfoWithData>)> &lambda,
				  const std::chrono::seconds &timeout = std::chrono::seconds(30));

	/**
	 * @brief コピーコンストラクタ
	 *
	 * @param other
	 */
	MultiThreader(const MultiThreader &other);

	/**
	 * @brief デストラクタ
	 *
	 * @param other
	 */
	~MultiThreader();

	std::vector<std::shared_ptr<CameraInfoWithData>> success; //!< 処理が全行程完了したカメラ群
	std::vector<std::shared_ptr<CameraInfoWithData>> fail;    //!< 処理が途中で失敗したカメラ群
	std::vector<std::pair<std::shared_ptr<CameraInfoWithData>, error::Error>> errors; // 処理が途中で失敗したカメラ群とそのエラー情報
};

} // namespace helper

} // namespace scanner

} // namespace symb
