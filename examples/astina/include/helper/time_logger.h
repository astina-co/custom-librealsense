#pragma once

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace symb
{
namespace scanner
{
/**
 * @brief APIの使用を補助するクラス群
 */
namespace helper
{

/**
 * @brief 時間計測とログ
 */
class TimeLogger
{
public:
	/**
	 * @brief コンストラクタ
	 *
	 * @param path ログを記録するパス
	 * @param headers ログを行うヘッダ
	 */
	TimeLogger(const std::experimental::filesystem::path &path, std::vector<std::string> &headers)
	{
		file_.open(path, std::ios::out);
		file_ << "session,";
		for (const std::string &header : headers)
		{
			file_ << header << ",";
		}
		file_.flush();
		start();
	}

	/**
	 * @brief 時間計測の開始
	 */
	void start() { start_time_ = std::chrono::steady_clock::now(); }

	/**
	 * @brief 記録
	 */
	void log()
	{
		file_ << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time_).count() << ",";
		file_.flush();
	}

	/**
	 * @brief 記録と時間計測の開始
	 */
	void log_start()
	{
		log();
		start();
	}

	/**
	 * @brief 次の行へ移動
	 */
	void next()
	{
		file_ << std::endl;
		file_ << ++session_number_ << ",";
		file_.flush();
	}

	/**
	 * @brief デストラクタ
	 */
	~TimeLogger()
	{
		try
		{
			file_.close();
		}
		catch (...)
		{
		}
	}

	/**
	 * @brief 行番号の取得
	 * 
	 * @return 行番号
	 */
	int session_number() const { return session_number_; }

private:
	int session_number_ = 0; //!< 行番号
	std::ofstream file_; //!< ファイルストリーム
	std::chrono::steady_clock::time_point start_time_; //!< 時間計測の開始時間
};

} // namespace helper

} // namespace scanner

} // namespace symb
