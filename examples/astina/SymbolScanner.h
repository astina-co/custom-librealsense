// SymbolScanner.h
#pragma once

#include <Open3D/Open3D.h>
#include <Open3D/Open3DConfig.h>

#ifdef SYMBOLSCANNER_EXPORTS
#define SYMBOLSCANNER_API __declspec(dllexport)
#else
#define SYMBOLSCANNER_API __declspec(dllimport)
#if OPEN3D_VERSION_MAJOR != 0
#error "Wrong Open3D Version! Use 0.7.0"
#endif
#if OPEN3D_VERSION_MINOR != 7
#error "Wrong Open3D Version! Use 0.7.0"
#endif
#if OPEN3D_VERSION_PATCH != 0
#error "Wrong Open3D Version! Use 0.7.0"
#endif
#if OPEN3D_VERSION_TWEAK != 0
#error "Wrong Open3D Version! Use 0.7.0"
#endif
#endif

// #define SYMBOLSCANNER_API

enum CAMERA_MODE
{
	NONE,
	CAPTURE,
	LIVE,
	CALIBRATE
};

typedef struct _f_range1d
{
	float min;
	float max;
} _f_range1d;

typedef struct _f_range3d
{
	_f_range1d x;
	_f_range1d y;
	_f_range1d z;
} _f_range3d;

typedef struct _board_params
{
	int square_count_x;
	int square_count_y;
	float square_length;
	float marker_length;
	int marker_size;
	float thickness;
} _board_params;

struct InitializeParams
{
	_board_params board_params;
	std::string config_file_path;
	std::string preset_json_path;
	_f_range3d range;
	bool cut_floor;
	double offset_y;
	double foot_range;
	std::string calibration_dir;
	std::string log_dir;
	bool enable_set_preset;
	unsigned int thread_num;
};

struct CaptureParams
{
	const unsigned int frame_count;
	const unsigned long long timeout;
};

// Nomura
// ノイズ除去を複数回実施できるように変更
// 現状の点群で試したところ、以下の値が良さげ。
// オリジナルの点群
//   noise_nb_neighbors 400, noise_sed_ratio=2.5f
//   noise_nb_neighbors 200, noise_sed_ratio=2.5f
//   noise_nb_neighbors 100, noise_sed_ratio=2.5f
// ダウンサンプリングした点群では2回
//   noise_nb_neighbors 300, noise_sed_ratio=2.0f
//   noise_nb_neighbors 100, noise_sed_ratio=2.5f
typedef struct NoiseFilterParams
{
	int noise_nb_neighbors = 0;
	float noist_std_ratio = 0.0f;
} _noise_filter_params;

struct FilterParams
{
	const float down_sample_voxel_size = 0.0f;
	const std::vector<_noise_filter_params> noise_filter_params;
	const int normal_knn = 0;
	// extract_radiusメートル以内のextract_knn個の点をつなげてクラスタを作る
	const double extract_radius; // 0.02 (2cm)
	const int extract_knn; // 20
};

// SymbolErrorsのエラーメッセージを保存しておく
static char symbol_error_message[5000];

#ifdef __cplusplus
extern "C"
{
#endif

	SYMBOLSCANNER_API void Initialize(const struct InitializeParams params);

	SYMBOLSCANNER_API void Terminate(const bool &enable_multithread=true);
	SYMBOLSCANNER_API void TerminateSequential(void);

    SYMBOLSCANNER_API void SetMode(const CAMERA_MODE mode, const unsigned int timeout=0, const bool &enable_multithread=true, const bool &enable_test_cap=true);
    SYMBOLSCANNER_API void SetModeWithoutTestCapture(const CAMERA_MODE mode, const int timeout = 0);
    SYMBOLSCANNER_API void SetModeSequential(const CAMERA_MODE mode);
	SYMBOLSCANNER_API void SetColorMode(const CAMERA_MODE mode);
	SYMBOLSCANNER_API void SetColorModeSequential(const CAMERA_MODE mode);
	SYMBOLSCANNER_API void SetColorModeWithoutTestCapture(const CAMERA_MODE mode, const int timeout = 0);

	SYMBOLSCANNER_API CAMERA_MODE GetMode(void);

	SYMBOLSCANNER_API unsigned int GetEnabledDeviceCount(void);

	SYMBOLSCANNER_API void SetPreset(const unsigned int retry_count, const char *preset_json_path, const bool &enable_multithread=true);
	SYMBOLSCANNER_API void SetPresetSequential(const unsigned int retry_count, const char *preset_json_path);

	SYMBOLSCANNER_API void SetMaskID(const std::vector<std::string> &enabled, const bool &enable_multithread=true);
	SYMBOLSCANNER_API void SetMaskIDSequential(const std::vector<std::string> &enabled);


	SYMBOLSCANNER_API void GenerateCalibration(void);

	SYMBOLSCANNER_API void SaveCalibration(void);

	SYMBOLSCANNER_API void SaveCalibrationToDir(const char *calibration_data_dir);

	SYMBOLSCANNER_API void LoadCalibration(void);

	SYMBOLSCANNER_API void LoadCalibrationFromDir(const char *calibration_data_dir);

	SYMBOLSCANNER_API bool IsCalibrated(void);

	SYMBOLSCANNER_API void AssertConnection(const bool &enable_multithread=true);

	SYMBOLSCANNER_API void AssertConnectionSequential();

	SYMBOLSCANNER_API void Capture(const unsigned int retry_count, const struct CaptureParams params);

	SYMBOLSCANNER_API void CaptureSequential(const unsigned int retry_count, const struct CaptureParams params);

	SYMBOLSCANNER_API void SaveRawCapture(const char *save_raw_dir);

	SYMBOLSCANNER_API void SavePointCloud(const char *save_pc_dir, const bool save_color_data);

	// 各カメラの点群はダウンサンプリングしていないものを保存する
	// SavePointCloudにまとめたかったが、他に影響すると困るので別メソッドにした
	SYMBOLSCANNER_API void SaveOrgPointCloud(const char *save_pc_dir, const bool save_color_data, const int normal_knn);

	SYMBOLSCANNER_API void StartLive(const struct FilterParams filter_params, std::function<void(const char *, const std::shared_ptr<open3d::geometry::PointCloud>)> &callback, const bool &enable_multithread=true);

	SYMBOLSCANNER_API void StartLiveSequential(const struct FilterParams filter_params, std::function<void(const char *, const std::shared_ptr<open3d::geometry::PointCloud>)> &callback);

	SYMBOLSCANNER_API void StopLive(const bool &enable_multithread=true);

	SYMBOLSCANNER_API void StopLiveSequential(void);

	SYMBOLSCANNER_API bool StreamingLive(void);

	SYMBOLSCANNER_API void ForceResetCameras(const bool &enable_multithread=true);

	SYMBOLSCANNER_API void ForceResetCamerasSequential(void);

	// 床の雄セットパラメータを設定（更新）
	SYMBOLSCANNER_API void SetFloorOffsetParam(const bool _cut_floor, const double _offset_y, const double _foot_range = 0.0f);

	// Nomura 追加
	// symbolErrors.errorsの取得
	SYMBOLSCANNER_API char* GetSymbolErrors();

#ifdef __cplusplus
}
#endif

class UnInitializedError : public std::runtime_error
{
public:
	SYMBOLSCANNER_API UnInitializedError(const std::string &routine_name);
};

class UnCalibratedError : public std::runtime_error
{
public:
	SYMBOLSCANNER_API UnCalibratedError(const std::string &routine_name);
};

class InvalidModeError : public std::runtime_error
{
public:
	SYMBOLSCANNER_API InvalidModeError(const std::string &routine_name, const std::vector<CAMERA_MODE> &valid_modes);
};

class InvalidFrameModeError : public std::runtime_error
{
public:
	SYMBOLSCANNER_API InvalidFrameModeError(const std::string &routine_name, const std::vector<CAMERA_MODE> &valid_modes);
};

class StreamingSafeGuardError : public std::runtime_error
{
public:
	SYMBOLSCANNER_API StreamingSafeGuardError(const std::string &routine_name);
};

class InvalidPointCloudError : public std::runtime_error
{
public:
	SYMBOLSCANNER_API InvalidPointCloudError(const std::string &routine_name);
};

class SymbolErrors : public std::runtime_error
{
public:
	SYMBOLSCANNER_API SymbolErrors(const std::vector<std::runtime_error> &errors);
	SYMBOLSCANNER_API std::vector<std::runtime_error> errors(void) const;

private:
	std::vector<std::runtime_error> errors_;
};

// タイムアウトをSymbolErrorsから分離
class SymbolTimeoutErrors : public std::runtime_error
{
public:
	SYMBOLSCANNER_API SymbolTimeoutErrors(const std::vector<std::runtime_error> &errors);
	SYMBOLSCANNER_API std::vector<std::runtime_error> errors(void) const;

private:
	std::vector<std::runtime_error> errors_;
};

SYMBOLSCANNER_API const std::string GetPreset(const unsigned int retry_count);

SYMBOLSCANNER_API std::shared_ptr<open3d::geometry::PointCloud> GeneratePointCloud(const struct FilterParams params);
