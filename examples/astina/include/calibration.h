#pragma once

#define _USE_MATH_DEFINES

#include <Open3d/3rdparty/Eigen/Eigen/Eigen>
#include <fmt/format.h>
#include <librealsense2/rs.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/opencv.hpp>
#include <spdlog/spdlog.h>

#include "board.h"
#include "frameset.h"
#include "utility.h"

namespace symb
{
namespace scanner
{

/**
 * @brief キャリブレーションデータ
 *
 * 外部パラメータの計算等を行う
 */
class CalibrationData
{
public:
	/**
	 * @brief デフォルトコンストラクタ
	 */
	CalibrationData() = default;

	/**
	 * @brief xmlファイルからデータを読み取るコンストラクタ
	 *
	 * @param xml_path xmlファイルのパス
	 */
	CalibrationData(const std::experimental::filesystem::path &xml_path) : ptr_(std::make_shared<CalibrationDataPtr>(xml_path)) {}

	/**
	 * @brief xmlファイルからデータを読み取るコンストラクタ (キャリブレーション時のPNGキャプチャ指定はデバッグ用)
	 *
	 * @param xml_path xmlファイルのパス
	 * @param png_before マーカー設置前のキャプチャデータ
	 * @param png_after マーカー設置後のキャプチャデータ
	 */
	CalibrationData(const std::experimental::filesystem::path &xml_path, 
		const std::experimental::filesystem::path &png_before, 
		const std::experimental::filesystem::path &png_after,
		const std::experimental::filesystem::path &png_depth)
		: ptr_(std::make_shared<CalibrationDataPtr>(xml_path, png_before, png_after, png_depth))
	{
	}

	/**
	 * @brief 新規でキャプチャデータを生成するコンストラクタ
	 *
	 * @param calibration_board キャリブレーションボード管理用インスタンス
	 * @param depth_scale デプスカメラの撮影スケール (単位距離) [m]
	 * @param rs2_intrinsics デプスカメラの内部パラメータ
	 * @param frame キャリブレーションを行うフレーム
	 * @param side Front or Rear
	 */
	CalibrationData(const CalibrationBoard &calibration_board, const Side &side, const Frameset &frameset) 
		: ptr_(std::make_shared<CalibrationDataPtr>(calibration_board, side, frameset)) {}

	static CalibrationData Identity(const CalibrationFrame &frame);

	/**
	 * @brief 保有するキャプチャデータが有効かを取得
	 *
	 * @return 保有するキャプチャデータが有効か
	 */
	bool is_valid() const
	{
		if (ptr_ == nullptr)
		{
			throw std::runtime_error("CalibrationData is empty");
		}
		return ptr_->is_valid();
	}

	/**
	 * @brief キャリブレーションに使用したデプスカメラの撮影スケールを取得
	 *
	 * @return キャリブレーションに使用したデプスカメラの撮影スケール
	 */
	float get_depth_scale() const
	{
		if (ptr_ == nullptr)
		{
			throw std::runtime_error("CalibrationData is empty");
		}
		return ptr_->get_depth_scale();
	}

	/**
	 * @brief キャリブレーションに使用したデプスカメラの内部パラメータを取得
	 *
	 * @return キャリブレーションに使用したデプスカメラの内部パラメータ
	 */
	Eigen::Matrix3d get_intrinsics() const
	{
		if (ptr_ == nullptr)
		{
			throw std::runtime_error("CalibrationData is empty");
		}
		return ptr_->get_intrinsics();
	}

	/**
	 * @brief キャリブレーションに使用したデプスカメラの内部パラメータの逆行列を取得
	 *
	 * @return キャリブレーションに使用したデプスカメラの内部パラメータの逆行列
	 */
	Eigen::Matrix3d get_intrinsics_inv() const
	{
		if (ptr_ == nullptr)
		{
			throw std::runtime_error("CalibrationData is empty");
		}
		return ptr_->get_intrinsics_inv();
	}

	/**
	 * @brief キャリブレーションで生成した外部パラメータを取得
	 *
	 * @return キャリブレーションで生成した外部パラメータ
	 */
	Eigen::Matrix4d get_extrinsics() const
	{
		if (ptr_ == nullptr)
		{
			throw std::runtime_error("CalibrationData is empty");
		}
		return ptr_->get_extrinsics();
	}

	/**
	 * @brief キャリブレーションで取得したカメラの歪みを取得
	 *
	 * @return キャリブレーションで取得したカメラの歪み
	 */
	cv::Mat get_cv_dist_coeffs() const
	{
		if (ptr_ == nullptr)
		{
			throw std::runtime_error("CalibrationData is empty");
		}
		return ptr_->get_cv_dist_coeffs();
	}

	/**
	 * @brief 原点からみたカメラの座標(OpenCVのVec3d)を取得
	 *
	 * @return 原点からみたカメラの座標(OpenCVのVec3d)
	 */
	cv::Vec3d get_cv_camera_coordinate() const
	{
		if (ptr_ == nullptr)
		{
			throw std::runtime_error("CalibrationData is empty");
		}
		return ptr_->get_cv_camera_coordinate();
	}

	/**
	 * @brief 原点からみたカメラの座標(EigenのVector3d)を取得
	 *
	 * @return 原点からみたカメラの座標(EigenのVector3d)
	 */
	Eigen::Vector3d get_camera_coordinate() const
	{
		if (ptr_ == nullptr)
		{
			throw std::runtime_error("CalibrationData is empty");
		}
		return ptr_->get_camera_coordinate();
	}

	void pretransform(const Eigen::Matrix4d &transform)
	{
		if (ptr_ == nullptr)
		{
			throw std::runtime_error("CalibrationData is empty");
		}
		ptr_->pretransform(transform);
	}

	/**
	 * @brief キャリブレーションデータをファイルに保存する
	 *
	 * @param xml_path 保存先のxmlファイルのパス
	 */
	void save_xml(const std::experimental::filesystem::path &xml_path) const
	{
		if (ptr_ == nullptr)
		{
			throw std::runtime_error("CalibrationData is empty");
		}
		ptr_->save_xml(xml_path);
	}

	/**
	 * @brief キャリブレーション時に使用したキャプチャデータを保存する(デバッグ用)
	 *
	 * @param png_before_path マーカー無し画像の保存先
	 * @param png_after_path マーカー有り画像の保存先
	 */
	void save_images(const std::experimental::filesystem::path &png_before_path, const std::experimental::filesystem::path &png_after_path,
					 const std::experimental::filesystem::path &png_depth_path) const
	{
		if (ptr_ == nullptr)
		{
			throw std::runtime_error("CalibrationData is empty");
		}
		ptr_->save_images(png_before_path, png_after_path, png_depth_path);
	}

	CalibrationData &operator=(const CalibrationData &other)
	{
		if (this != &other)
		{
			ptr_ = other.ptr_;
		}

		return *this;
	}

private:
	CalibrationData(Frameset &frame);

	class CalibrationDataPtr
	{
	public:
		/**
		 * @brief デフォルトコンストラクタ
		 */
		CalibrationDataPtr() = default;

		/**
		 * @brief xmlファイルからデータを読み取るコンストラクタ
		 *
		 * @param xml_path xmlファイルのパス
		 */
		CalibrationDataPtr(const std::experimental::filesystem::path &xml_path);

		/**
		 * @brief xmlファイルからデータを読み取るコンストラクタ (キャリブレーション時のPNGキャプチャ指定はデバッグ用)
		 *
		 * @param xml_path xmlファイルのパス
		 * @param png_before マーカー設置前のキャプチャデータ
		 * @param png_after マーカー設置後のキャプチャデータ
		 */
		CalibrationDataPtr(const std::experimental::filesystem::path &xml_path, 
			const std::experimental::filesystem::path &png_before, 
			const std::experimental::filesystem::path &png_after,
			const std::experimental::filesystem::path &png_depth);

		/**
		 * @brief 新規でキャプチャデータを生成するコンストラクタ
		 *
		 * @param calibration_board キャリブレーションボード管理用インスタンス
		 * @param depth_scale デプスカメラの撮影スケール (単位距離) [m]
		 * @param rs2_intrinsics デプスカメラの内部パラメータ
		 * @param frame キャリブレーションを行うフレーム
		 * @param side 面
		 */
		CalibrationDataPtr(const CalibrationBoard &calibration_board, const Side side, const Frameset &frameset);

		CalibrationDataPtr(const CalibrationFrame &frame);

		/**
		 * @brief キャリブレーションに使用したデプスカメラの撮影スケールを取得
		 *
		 * @return キャリブレーションに使用したデプスカメラの撮影スケール
		 */
		float get_depth_scale() const;

		/**
		 * @brief 保有するキャプチャデータが有効かを取得
		 *
		 * @return 保有するキャプチャデータが有効か
		 */
		bool is_valid() const;

		/**
		 * @brief キャリブレーションに使用したデプスカメラの内部パラメータを取得
		 *
		 * @return キャリブレーションに使用したデプスカメラの内部パラメータ
		 */
		Eigen::Matrix3d get_intrinsics() const;

		/**
		 * @brief キャリブレーションに使用したデプスカメラの内部パラメータの逆行列を取得
		 *
		 * @return キャリブレーションに使用したデプスカメラの内部パラメータの逆行列
		 */
		Eigen::Matrix3d get_intrinsics_inv() const;

		/**
		 * @brief キャリブレーションで生成した外部パラメータを取得
		 *
		 * @return キャリブレーションで生成した外部パラメータ
		 */
		Eigen::Matrix4d get_extrinsics() const;

		/**
		 * @brief キャリブレーションで取得したカメラの歪みを取得
		 *
		 * @return キャリブレーションで取得したカメラの歪み
		 */
		cv::Mat get_cv_dist_coeffs() const;

		/**
		 * @brief 原点からみたカメラの座標(OpenCVのVec3d)を取得
		 *
		 * @return 原点からみたカメラの座標(OpenCVのVec3d)
		 */
		cv::Vec3d get_cv_camera_coordinate() const;

		/**
		 * @brief 原点からみたカメラの座標(EigenのVector3d)を取得
		 *
		 * @return 原点からみたカメラの座標(EigenのVector3d)
		 */
		Eigen::Vector3d get_camera_coordinate() const;

		void pretransform(const Eigen::Matrix4d &transform);

		/**
		 * @brief キャリブレーションデータをファイルに保存する
		 *
		 * @param xml_path 保存先のxmlファイルのパス
		 */
		void save_xml(const std::experimental::filesystem::path &xml_path) const;

		/**
		 * @brief キャリブレーション時に使用したキャプチャデータを保存する(デバッグ用)
		 *
		 * @param png_before_path マーカー無し画像の保存先
		 * @param png_after_path マーカー有り画像の保存先
		 */
		void save_images(const std::experimental::filesystem::path &png_before_path, const std::experimental::filesystem::path &png_after_path, 
			const std::experimental::filesystem::path &png_depth_path) const;

		bool has_images() const { return _has_canvas; }

		const std::string depth_scale_index = "depth_scale"; //!< キャリブレーションデータ保存の際のインデックス(撮影スケール)
		const std::string intrinsics_index = "cv_intrinsics"; //!< キャリブレーションデータ保存の際のインデックス(内部パラメータ)
		const std::string dist_coeffs_index = "cv_dist_coeffs"; //!< キャリブレーションデータ保存の際のインデックス(歪み)
		const std::string extrinsics_index = "cv_extrinsics"; //!< キャリブレーションデータ保存の際のインデックス(外部パラメータ)
		const std::string camera_coordinate_index = "cv_camera_coordinate"; //!< キャリブレーションデータ保存の際のインデックス(カメラの座標)

		CalibrationDataPtr &operator=(const CalibrationDataPtr &other)
		{
			if (this != &other)
			{
				_intrinsics = other._intrinsics;
				_intrinsics_inv = other._intrinsics_inv;
				_extrinsics = other._extrinsics;
				_depth_scale = other._depth_scale;
				_cv_intrinsics = other._cv_intrinsics;
				_cv_dist_coeffs = other._cv_dist_coeffs;
				_has_canvas = other._has_canvas;
				_clean_canvas = other._clean_canvas;
				_drawn_canvas = other._drawn_canvas;
				_depth_canvas = other._depth_canvas;
				_cv_camera_coordinate = other._cv_camera_coordinate;
				_is_private = other._is_private;
			}

			return *this;
		}

	private:
		Eigen::Matrix3d _intrinsics = Eigen::Matrix3d::Identity(); //!< 内部パラメータ
		Eigen::Matrix3d _intrinsics_inv = Eigen::Matrix3d::Identity(); //!< 内部パラメータの逆行列
		Eigen::Matrix4d _extrinsics = Eigen::Matrix4d::Identity(); //!< 外部パラメータ

		float _depth_scale;

		cv::Mat _cv_intrinsics; //!< 内部パラメータ
		cv::Mat _cv_dist_coeffs; //!< 歪み

		bool _has_canvas = false; //!< フレームを保持しているか
		cv::Mat _clean_canvas; //!< マーカー無しフレーム
		cv::Mat _drawn_canvas; //!< マーカー有りフレーム
		cv::Mat _depth_canvas; //!< デプスの生データフレーム

		cv::Vec3d _cv_camera_coordinate; //!< カメラの座標

		bool _is_private = false;
	};

	CalibrationData(const std::shared_ptr<CalibrationDataPtr> &ptr) : ptr_(ptr){};

	std::shared_ptr<CalibrationDataPtr> ptr_; //!< ポインタ
};

} // namespace scanner

} // namespace symb