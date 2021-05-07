#pragma once

#include "fmt/format.h"

#include <Open3D/Geometry/KDTreeFlann.h>
#include <Open3D/IO/ClassIO/PointCloudIO.h>
#include <Open3D/Integration/ScalableTSDFVolume.h>

#include <opencv2/imgproc/types_c.h>
#include <opencv2/opencv.hpp>

#include <librealsense2/rsutil.h>

#include "calibration.h"
#include "frame.h"

#include "PoissonReconLib.h"

namespace symb
{
namespace scanner
{

/**
 * @brief 点群生成の補助
 */
class PointCloud
{
public:
	/**
	 * @brief デフォルトコンストラクタ
	 */
	PointCloud() = default;

	/**
	 * @brief コンストラクタ
	 */
	PointCloud(const std::shared_ptr<open3d::geometry::PointCloud> &point_cloud);

	/**
	 * @brief 点群生成コンストラクタ
	 *
	 * @param calibration 点群生成に使うキャリブレーションデータ
	 * @param frame 点群生成に使うフレーム
	 * @param depth_scale 撮影時に使用したデプスカメラの撮影スケール
	 * @param range 点群の生成を許す範囲({0.f,0.f,0.f,0.f,0.f,0.f}で範囲制限を外す)
	 */
	PointCloud(const CalibrationData &calibration, const Frame &frame, const float depth_scale, const utility::Range3Df range = utility::Range3Df{0.f, 0.f, 0.f, 0.f, 0.f, 0.f});

	/**
	 * @brief 複数点群合成コンストラクタ / 合成された点群が生成される
	 *
	 * @param point_clouds 複数のPointCloudが含まれたvector
	 */
	PointCloud(const std::vector<PointCloud> &point_clouds);

	/**
	 * @brief 点群数を絞る / 指定の半径内の点群数を1とする
	 *
	 * @param voxel_size 指定の半径[m]
	 */
	void downSample(const double voxel_size);

	/**
	 * @brief 法線を推定する(向きは未解決)
	 *
	 * @param knn k-nearest-neighbor
	 */
	void estimateNormals(const int knn);

	/**
	 * @brief 法線を推定する(向きは未解決)
	 *
	 * @param radius
	 * @param knn k-nearest-neighbor
	 */
	void estimateNormalsHybrid(const double radius, const int knn);

	/**
	 * @brief 法線の向きを解決
	 *
	 * @param camera_coordinate カメラがある側に法線を向ける
	 */
	void orientNormals(const Eigen::Vector3d &camera_coordinate);

	/**
	 * @brief 他点群から離れている点群を削除 / ノイズ削減に使われる
	 *
	 * @param nb_neighbors 近くにこの数の点群が無いと消される
	 * @param std_ratio 削除を行う標準偏差(小さいほど削除が行われやすくなる)
	 */
	void removeStatisticalOutliers(const int nb_neighbors, const double std_ratio);

	/**
	 * @brief PoissonReconを用いて点群をメッシュ化し、ファイルに保存する
	 *
	 * @param depth メッシュ化を行うイテレーション数(処理時間に大幅影響)
	 * @param samplesPerNode この値を大きく(20.0とか)すると、点群の削りすぎを防止できる
	 * @param save_path メッシュデータを保存するパス
	 */
	void generateMesh(const int depth, const float samplesPerNode, const std::experimental::filesystem::path &save_path);

	void scaleUp(const float scale);

	void extractLargestConnectedComponent(const double radius, const int knn);

	/**
	 * @brief Open3D準拠の点群管理オブジェクトの取得
	 *
	 * @return std::shared_ptr<open3d::geometry::PointCloud>インスタンス
	 */
	inline std::shared_ptr<open3d::geometry::PointCloud> ptr() const { return point_cloud_; }

	/**
	 * @brief Open3D準拠の点群管理オブジェクトの取得(オーバーロード)
	 *
	 * @return 型を明示的にstd::shared_ptr<open3d::geometry::PointCloud>に指定
	 */
	operator std::shared_ptr<open3d::geometry::PointCloud>() const { return ptr(); }

	/**
	 * @brief Open3D準拠の点群管理インスタンスの取得(オーバーロード)
	 *
	 * @return 型を明示的にopen3d::geometry::PointCloudに指定
	 */
	operator open3d::geometry::PointCloud() const { return *point_cloud_; }

	PointCloud &operator=(const PointCloud &other)
	{
		if (this != &other)
		{
			point_cloud_->points_ = other.point_cloud_->points_;
			point_cloud_->colors_ = other.point_cloud_->colors_;
			point_cloud_->normals_ = other.point_cloud_->normals_;
		}
		return *this;
	}

private:
	std::shared_ptr<open3d::geometry::PointCloud> point_cloud_ = std::make_shared<open3d::geometry::PointCloud>(); //!< Open3D準拠の点群管理オブジェクト
};

} // namespace scanner

} // namespace symb