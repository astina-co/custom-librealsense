#ifndef POISSON_RECON_DLL_WRAPPER
#define POISSON_RECON_DLL_WRAPPER


#ifdef RECONLIBRARY_EXPORTS
#define RECONLIBRARY_API __declspec(dllexport)
#else
#define RECONLIBRARY_API __declspec(dllimport)
#endif

//ERROR CODE
// Argument values for exit()
//#define EXIT_SUCCESS 0
//#define EXIT_FAILURE 1
#define EXIT_INPUT_PATH_FAILURE		2
#define EXIT_OUTPUT_PATH_FAILURE	3
#define EXIT_CAMERA_NUM_FAILURE		4
#define EXIT_FILE_COMP_FAILURE		5

//DEFAULT
#define DEFAULT_DEPTH				8

#define MAX_CAMERAS					20

//! Algorithm parameters
typedef struct ReconParams_t
{
	char*	in;
	char*	out;
	int		depth;
	float	samplesPerNode;
	double	voxel;
	bool	crop;
}ReconParams;

extern "C" RECONLIBRARY_API int ReconstructFromDir(ReconParams* params_);
extern "C" RECONLIBRARY_API int Reconstruct(ReconParams* params);
//extern "C" RECONLIBRARY_API int Reconstruct(ReconParams* params, char* out_);
//extern "C" RECONLIBRARY_API int Reconstruct(ReconParams* params, int* i_);

#endif // POISSON_RECON_DLL_WRAPPER
