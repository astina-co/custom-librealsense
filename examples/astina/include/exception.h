#pragma once

#include <excpt.h>
#include <windows.h>

#include <exception>
#include <string>

#include <librealsense2/rs.hpp>

#include "frame.h"

namespace symb
{
namespace scanner
{
/**
 * @brief symb::scanner APIから投げられるエラー群
 */
namespace error
{

/**
 * @brief エラーの種類
 */
enum class ErrorType
{
	DEFAULT, //!< デフォルト
	UNKNOWN, //!< 原因不明
	INVALID_PARAMETER, //!< 不正な関数パラメータ
	BAD_STATE, //!< 不正なステート(実装が良くない可能性が高い)
	CALIBRATION, //!< キャリブレーション時に発生
	SENSOR_INFO, //!< センサ情報の取得・改変時発生
	TIMEOUT //!< タイムアウト発生(主にMultithreaderから)
};

/**
 * @brief キャリブレーション時のエラーの種類
 */
enum class CalibrationErrorType
{
	DEFAULT, //!< デフォルト
	NO_MARKERS, //!< マーカー未検出
	INSUFFICIENT_MARKERS, //!< マーカー不足
	FAILED_CORNER_ESTIMATION //!< 原点推定失敗
};

/**
 * @brief センサ情報の取得・改変時のエラーの種類
 */
enum class OptionErrorType
{
	DEFAULT, //!< デフォルト
	UNSUPPORTED, //!< optionパラメータがセンサに非対応
	OUT_OF_RANGE //!< 設定する値が範囲外
};

/**
 * @brief ベースエラー
 */
class Error : public std::runtime_error
{
public:
	Error(const std::string &msg, const bool requires_reset, const bool recoverable) : std::runtime_error(msg)
	{
		this->requires_reset = requires_reset;
		this->recoverable = recoverable;
	}

	Error(const Error &e) : std::runtime_error(e.what())
	{
		if (this != &e)
		{
			this->requires_reset = e.requires_reset;
			this->recoverable = e.recoverable;
			this->markers_found = e.markers_found;
			this->frame_ = e.frame_;
			this->option = e.option;
			this->value = e.value;
			this->min = e.min;
			this->max = e.max;
			this->type = e.type;
			this->calibration_error_type = e.calibration_error_type;
			this->option_error_type = e.option_error_type;
		}
	}

	/**
	 * @brief デバイスのソフトリセットの必要性を取得
	 *
	 * @return デバイスのソフトリセットが必要か
	 */
	inline bool requiresReset() const noexcept { return requires_reset; }

	/**
	 * @brief デバイスの復帰が可能か
	 *
	 * @return デバイスの復帰が可能か
	 */
	inline bool isRecoverable() const noexcept { return recoverable; }

	/**
	 * @brief エラーの種類を取得
	 *
	 * @return エラーの種類
	 */
	inline ErrorType getErrorType() const noexcept { return type; }

	/**
	 * @brief キャリブレーション時のエラーの種類を取得
	 *
	 * @return キャリブレーション時のエラーの種類
	 */
	inline CalibrationErrorType getCalibrationErrorType() const noexcept { return calibration_error_type; }

	/**
	 * @brief センサ情報の取得・改変時のエラーの種類を取得
	 *
	 * @return センサ情報の取得・改変時のエラーの種類
	 */
	inline OptionErrorType getOptionType() const noexcept { return option_error_type; }

	static const ErrorType class_type = ErrorType::DEFAULT; //!< クラスのエラーの種類(ベースエラーからの拡張用)
	static const CalibrationErrorType class_calibration_error_type = CalibrationErrorType::DEFAULT; //!< クラスのキャリブレーション時のエラーの種類(ベースエラーからの拡張用)
	static const OptionErrorType class_option_error_type = OptionErrorType::DEFAULT; //!< クラスのセンサ情報の取得・改変時のエラーの種類(ベースエラーからの拡張用)

	/**
	 * @brief コンストラクト時のクラスタイプとテンプレートのクラスが一致しているか
	 *
	 * @return コンストラクト時のクラスタイプとテンプレートのクラスが一致しているか
	 */
	template <class T> bool is() const { return T::class_type == type && T::class_calibration_error_type == calibration_error_type && T::class_option_error_type == option_error_type; }

	/**
	 * @brief テンプレートのクラスは拡張
	 *
	 * @return クラス拡張されたインスタンス
	 */
	template <class T> T as() const
	{
		if (!is<T>())
			throw std::runtime_error("Bad Extention");
		return T(*this);
	}

protected:
	// ErrorType == CALIBRATION parameters
	int markers_found = 0; //!< 検出されたマーカー数 (キャリブレーション時のエラーのみ)
	Frame frame_; //!< キャリブレーションにしようしたフレーム (キャリブレーション時のエラーのみ)

	// ErrorType == SENSOR_INFO parameters
	rs2_option option; //!< 設定するセンサ情報 (センサ情報の取得・改変時のエラーのみ)
	float value = 0.0f; // 設定する値 (センサ情報の取得・改変時のエラーのみ)
	float min = 0.0f; // 設定先の最小値 (センサ情報の取得・改変時のエラーのみ)
	float max = 0.0f; // 設定先の最大値 (センサ情報の取得・改変時のエラーのみ)

	ErrorType type = ErrorType::DEFAULT;
	CalibrationErrorType calibration_error_type = CalibrationErrorType::DEFAULT;
	OptionErrorType option_error_type = OptionErrorType::DEFAULT;

private:
	bool requires_reset = false; //!< リセットで復帰ができるか
	bool recoverable = false; //!< 復帰可能か(やり直し等)
};

/**
 * @brief やり直しが効くエラー
 */
class RetryableError : public Error
{
public:
	RetryableError(const std::string &msg) : Error(msg, false, true){};
	RetryableError(const Error &e) : Error(e){};
};

/**
 * @brief リセット後にやり直しが効くかもしれないエラー
 */
class RecoverableError : public Error
{
public:
	RecoverableError(const std::string &msg) : Error(msg, true, true){};
	RecoverableError(const Error &e) : Error(e){};
};

/**
 * @brief リセットができない or リセットしてもそのままでは復帰できないエラー
 */
class UnRecoverableError : public Error
{
public:
	UnRecoverableError(const std::string &msg) : Error(msg, false, false){};
	UnRecoverableError(const Error &e) : Error(e){};
};

/**
 * @brief symb::scanner APIにとって不明なエラー
 */
class UnknownError : public Error
{
public:
	UnknownError(const std::string &msg) : Error(msg, false, false) { this->type = ErrorType::UNKNOWN; };
	UnknownError(const Error &e) : Error(e){};

	static const ErrorType class_type = ErrorType::UNKNOWN;
};

/**
 * @brief 不正なパラメータを使用した際に投げられるエラー
 */
class InvalidParameterError : public UnRecoverableError
{
public:
	InvalidParameterError(const std::string &msg) : UnRecoverableError(msg) { this->type = ErrorType::INVALID_PARAMETER; };
	InvalidParameterError(const Error &e) : UnRecoverableError(e){};

	static const ErrorType class_type = ErrorType::INVALID_PARAMETER;
};

/**
 * @brief 不正なステート時に投げられるエラー(実装が良くない可能性が高い)
 */
class BadStateError : public UnRecoverableError
{
public:
	BadStateError(const std::string &msg) : UnRecoverableError(msg) { this->type = ErrorType::BAD_STATE; };
	BadStateError(const Error &e) : UnRecoverableError(e){};

	static const ErrorType class_type = ErrorType::BAD_STATE;
};

/**
 * @brief キャリブレーションエラー
 */
class CalibrationError : public RetryableError
{
public:
	CalibrationError(const std::string &msg) : RetryableError(msg) { this->type = ErrorType::CALIBRATION; };
	CalibrationError(const Error &e) : RetryableError(e){};

	static const ErrorType class_type = ErrorType::CALIBRATION;
};

/**
 * @brief マーカー未検出エラー
 */
class NoMarkersFoundError : public CalibrationError
{
public:
	NoMarkersFoundError(const std::string &msg, const Frame &frame) : CalibrationError(msg)
	{
		this->markers_found = 0;
		this->frame_ = frame;
		this->calibration_error_type = CalibrationErrorType::NO_MARKERS;
	};
	NoMarkersFoundError(const Error &e) : CalibrationError(e){};

	/**
	 * @brief 検出したマーカーの数
	 * @return 検出したマーカーの数
	 */
	inline int getMarkersFound() const { return markers_found; }

	/**
	 * @brief キャリブレーションに使用したエラー
	 * @return キャリブレーションに使用したエラー
	 */
	inline Frame getFrame() const { return frame_; }

	static const CalibrationErrorType class_calibration_error_type = CalibrationErrorType::NO_MARKERS;
};

/**
 * @brief マーカー不足エラー
 */
class InsufficientMarkersError : public CalibrationError
{
public:
	InsufficientMarkersError(const std::string &msg, const int markers_found, const Frame &frame) : CalibrationError(msg)
	{
		this->markers_found = markers_found;
		this->frame_ = frame;
		this->calibration_error_type = CalibrationErrorType::INSUFFICIENT_MARKERS;
	};
	InsufficientMarkersError(const Error &e) : CalibrationError(e){};

	/**
	 * @brief 検出したマーカーの数
	 * @return 検出したマーカーの数
	 */
	inline int getMarkersFound() const { return markers_found; }

	/**
	 * @brief キャリブレーションに使用したエラー
	 * @return キャリブレーションに使用したエラー
	 */
	inline Frame getFrame() const { return frame_; }

	static const CalibrationErrorType class_calibration_error_type = CalibrationErrorType::INSUFFICIENT_MARKERS;
};

/**
 * @brief 原点推定失敗エラー
 */
class CornerEstimationError : public CalibrationError
{
public:
	CornerEstimationError(const std::string &msg, const int markers_found, const Frame &frame) : CalibrationError(msg)
	{
		this->markers_found = markers_found;
		this->frame_ = frame;
		this->calibration_error_type = CalibrationErrorType::FAILED_CORNER_ESTIMATION;
	};
	CornerEstimationError(const Error &e) : CalibrationError(e){};

	/**
	 * @brief 検出したマーカーの数
	 * @return 検出したマーカーの数
	 */
	inline int getMarkersFound() const { return markers_found; }

	/**
	 * @brief キャリブレーションに使用したエラー
	 * @return キャリブレーションに使用したエラー
	 */
	inline Frame getFrame() const { return frame_; }

	static const CalibrationErrorType class_calibration_error_type = CalibrationErrorType::FAILED_CORNER_ESTIMATION;
};

/**
 * @brief センサ情報の取得・改変時に発生するエラー
 */
class SensorInfoError : public RecoverableError
{
public:
	SensorInfoError(const std::string &msg, const rs2_option option) : RecoverableError(msg)
	{
		this->option = option;
		this->type = ErrorType::SENSOR_INFO;
	};

	SensorInfoError(const Error &e) : RecoverableError(e){};

	/**
	 * @brief オプション
	 * @return オプション
	 */
	inline rs2_option getOption() const { return option; };

	static const ErrorType class_type = ErrorType::SENSOR_INFO;
};

/**
 * @brief 設定非対応エラー
 */
class OptionUnsupportedError : public SensorInfoError
{
public:
	OptionUnsupportedError(const std::string &msg, const rs2_option option) : SensorInfoError(msg, option) { this->option_error_type = OptionErrorType::UNSUPPORTED; }

	OptionUnsupportedError(const Error &e) : SensorInfoError(e){};

	static const OptionErrorType class_option_error_type = OptionErrorType::UNSUPPORTED;
};

/**
 * @brief 設定範囲外エラー
 */
class OptionOutOfRangeError : public SensorInfoError
{
public:
	OptionOutOfRangeError(const std::string &msg, rs2_option option, const float value, const float min, const float max) : SensorInfoError(msg, option)
	{
		this->value = value;
		this->min = min;
		this->max = max;
		this->option_error_type = OptionErrorType::OUT_OF_RANGE;
	}

	OptionOutOfRangeError(const Error &e) : SensorInfoError(e){};

	/**
	 * @brief 設定値の取得
	 * @return 設定値
	 */
	inline float getValue() const { return value; }

	/**
	 * @brief 設定の最小値の取得
	 * @return 設定の最小値
	 */
	inline float getMin() const { return min; }

	/**
	 * @brief 設定の最大値の取得
	 * @return 設定の最大値
	 */
	inline float getMax() const { return max; }

	static const OptionErrorType class_option_error_type = OptionErrorType::OUT_OF_RANGE;
};

/**
 * @brief タイムアウトエラー
 */
class TimeoutError : public RecoverableError
{
public:
	TimeoutError(const std::string &msg) : RecoverableError(msg) { this->type = ErrorType::TIMEOUT; };
	TimeoutError(const Error &e) : RecoverableError(e){};

	static const ErrorType class_type = ErrorType::TIMEOUT;
};


/**
 * @brief タイムアウトエラー（リカバリー不能）
 */
class UnRecoverableTimeoutError : public UnRecoverableError
{
public:
	UnRecoverableTimeoutError(const std::string &msg) : UnRecoverableError(msg) { this->type = ErrorType::TIMEOUT; };
	UnRecoverableTimeoutError(const Error &e) : UnRecoverableError(e){};

	static const ErrorType class_type = ErrorType::TIMEOUT;
};

} // namespace error

} // namespace scanner

} // namespace symb
