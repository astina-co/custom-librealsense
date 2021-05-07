#pragma once

#include "fmt/format.h"

#include <Open3D/Geometry/PointCloud.h>
#include <Open3d/3rdparty/Eigen/Eigen/Eigen>
#include <experimental/filesystem>
#include <librealsense2/rs.hpp>
#include <map>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/opencv.hpp>
#include <sensor.h>

#include "board.h"

namespace symb
{
namespace scanner
{

/**
 * @brief 撮影フレーム
 *
 * rs2::framesetインスタンスを1つ保有する
 */
class Frame
{
public:
	/**
	 * @brief デフォルトコンストラクタ
	 */
	Frame() {}

	Frame(const Frame &frame) : frameset_(frame.raw()), initialized_(true) { operator=(frame); }

	/**
	 * @brief コンストラクタ
	 *
	 * @param frame 使用するrs2::framesetのインスタンス
	 */
	Frame(const rs2::frameset &frame) : frameset_(frame), initialized_(true)
	{
		frameset_.keep();
		rs2::frame f;
		if (f = frameset_.get_color_frame())
		{
			// SPDLOG_TRACE("rs2::frameset has color frame addr: {}", f.get_data());
			frames_.insert(std::make_pair(rs2_stream::RS2_STREAM_COLOR, f));
		}
		if (f = frameset_.get_depth_frame())
		{
			// SPDLOG_TRACE("rs2::frameset has depth frame addr: {}", f.get_data());
			frames_.insert(std::make_pair(rs2_stream::RS2_STREAM_DEPTH, f));
		}
		if (f = frameset_.get_infrared_frame())
		{
			// SPDLOG_TRACE("rs2::frameset has infrared frame addr: {}", f.get_data());
			frames_.insert(std::make_pair(rs2_stream::RS2_STREAM_INFRARED, f));
		}
	}

	/**
	 * @brief 指定したストリームタイプに対するアラインメントを施したフレームを取得
	 *
	 * @param stream_type 指定のストリーム種
	 * @return アラインメントが施されたフレーム
	 */
	Frame get_aligned_to(const rs2_stream stream_type) const;

	/**
	 * @brief
	 *
	 * @return 保有しているrs2::framesetインスタンス
	 */
	rs2::frameset raw() const;

	/**
	 * @brief 指定したストリーム種のフレームが含まれているかを取得
	 *
	 * @param stream_type 指定のストリーム種
	 * @return 指定したストリーム種のフレームが含まれているか
	 */
	bool from_stream(const rs2_stream stream_type) const;

	/**
	 * @brief cv::Mat画像化したデータの取得
	 *
	 * stream_type = rs2_stream::RS2_STREAM_DEPTHの時はカラー化した画像を返す
	 *
	 * @param stream_type 指定のストリーム種
	 * @param convert デプスデータをカラー画像に変換するか
	 * @return 画像化したデータ
	 */
	cv::Mat get_image(const rs2_stream stream_type, const bool convert = true, const bool filter = false) const;

	rs2::video_stream_profile get_calibration_profile();

	cv::Mat get_intrinsics();

	cv::Mat get_dist_coeffs();

	/**
	 * @brief 代入
	 *
	 * @param other
	 * @return Frame
	 */
	Frame &operator=(const Frame &other)
	{
		if (this != &other)
		{
			initialized_ = other.initialized_;
			frameset_ = other.frameset_;
			frames_ = other.frames_;
			has_intrinsics = other.has_intrinsics;
			calibration_profile_ = other.calibration_profile_;
			cv_intrinsics_ = other.cv_intrinsics_;
			cv_dist_coeffs_ = other.cv_dist_coeffs_;
			has_markers = other.has_markers;
			markers_ = other.markers_;
			has_charuco = other.has_charuco;
			charuco_ = other.charuco_;
			has_single_pose = other.has_single_pose;
			single_pose_ = other.single_pose_;
			has_charuco_pose = other.has_charuco_pose;
			charco_pose_ = other.charco_pose_;
			has_image = other.has_image;
			drawn_image = other.drawn_image;
			has_corner_pointcloud = other.has_corner_pointcloud;
			corner_pointcloud_ = other.corner_pointcloud_;
		}
		return *this;
	}

	/**
	 * @brief
	 *
	 *@return rs2::frameset
	 */
	operator rs2::frameset() const { return raw(); }

	/**
	 * @brief
	 *
	 *@return bool
	 */
	operator bool() const { return initialized_; }

protected:
	bool initialized_ = false; //!< フレームが設定されているか
	rs2::frameset frameset_; //!< 保管されるrs2::framesetインスタンス
	std::map<rs2_stream, rs2::frame> frames_; //!< 各ストリーム種に対するrs2::frame (時短用)

	bool has_intrinsics = false;
	void retrieve_intrinsics();
	rs2::stream_profile calibration_profile_;
	cv::Mat cv_intrinsics_;
	cv::Mat cv_dist_coeffs_;

	bool has_markers = false;
	std::pair<std::vector<int>, std::vector<std::vector<cv::Point2f>>> markers_;

	bool has_charuco = false;
	std::pair<std::vector<int>, std::vector<cv::Point2f>> charuco_;

	bool has_single_pose = false;
	std::pair<std::vector<cv::Vec3d>, std::vector<cv::Vec3d>> single_pose_;

	bool has_charuco_pose = false;
	std::pair<cv::Vec3d, cv::Vec3d> charco_pose_;

	bool has_image = false;
	cv::Mat drawn_image;

	bool has_depth = false;
	cv::Mat depth_image;

	bool has_corner_pointcloud = false;
	std::tuple<double, std::pair<std::vector<int>, open3d::geometry::PointCloud>> corner_pointcloud_;
};

class CalibrationFrame : public Frame
{
public:
	CalibrationFrame(const Frame &frame, const cv::Ptr<cv::aruco::Dictionary> &dictionary, const cv::Ptr<cv::aruco::CharucoBoard> &board) : Frame(frame), dictionary_(dictionary), board_(board) {}

	std::pair<std::vector<int>, std::vector<std::vector<cv::Point2f>>> get_markers();

	std::pair<std::vector<int>, std::vector<cv::Point2f>> get_charuco();

	std::pair<std::vector<cv::Vec3d>, std::vector<cv::Vec3d>> get_single_pose();

	std::pair<cv::Vec3d, cv::Vec3d> get_pose();

	Eigen::Matrix4d get_pose_mat();

	cv::Mat get_calibrated_image();

	cv::Mat get_aligned_depth_image();

	std::tuple<double, std::pair<std::vector<int>, open3d::geometry::PointCloud>> get_corner_pointcloud();

	CalibrationFrame &operator=(const CalibrationFrame &other)
	{
		if (this != &other)
		{
			Frame::operator=(other);
		}
		return *this;
	}

	operator Frame() { return Frame().operator=(*this); }

private:
	cv::Ptr<cv::aruco::Dictionary> dictionary_;
	cv::Ptr<cv::aruco::CharucoBoard> board_;
};

} // namespace scanner

} // namespace symb
