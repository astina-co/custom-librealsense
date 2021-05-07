#pragma once

#include "fmt/format.h"

#include "frame.h"

namespace symb
{
namespace scanner
{

/**
 * @brief 複数撮影フレーム
 * 
 * 複数Frameインスタンスを保有する
 */
class Frameset
{
public:
	/**
	 * @brief デフォルトコンストラクタ
	 *
	 */
	Frameset() = default;

	/**
	 * @brief コンストラクタ
	 *
	 * @param frames Frameのvector
	 */
	Frameset(const std::vector<Frame> &frames) : frames_(frames), initialized_(true) {}

	/**
	 * @brief 保管されている複数のフレームからフィルタリングをかけてFrameを生成
	 *
	 * @return 生成されたFrame
	 */
	Frame get_filtered() const;

	/**
	 * @brief 保管されている複数のフレームからalignをかけてFrameを生成
	 *
	 * @return 生成されたFrame
	 */
	Frame get_aligned() const;

	/**
	 * @brief Frameのvectorを返す / 個別のフレームを扱う用
	 *
	 * @return Frameのvector
	 */
	std::vector<Frame> vector() const;

	Frameset &operator=(const Frameset &other)
	{
		if (this != &other)
		{
			initialized_ = other.initialized_;
			frames_ = other.frames_;
		}
		return *this;
	}

	/**
	 * @brief Frameset含まれるFrameの個数
	 *
	 * @return Frameの個数
	 */
	size_t size() const;

	/**
	 * @brief index番目のFrameを取得
	 *
	 * @param index アクセスするフレームのインデックス
	 * @return 対象のフレーム
	 */
	Frame Frameset::operator[](int index) const
	{
		if (!initialized_)
		{
			throw std::runtime_error("Frameset is Empty");
		}
		if (index >= frames_.size())
		{
			throw std::runtime_error("Index out of bounds");
		}
		return frames_[index];
	}

private:
	bool initialized_ = false; //!< frameが含まれているか
	std::vector<Frame> frames_; //!< frameのvector
};

} // namespace scanner

} // namespace symb
