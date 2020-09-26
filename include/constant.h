#ifndef BFM_CONSTANT_H
#define BFM_CONSTANT_H


// Print
#define PRINT_RED "\033[31m"
#define PRINT_GREEN "\033[32m"
#define COLOR_END "\033[0m"
#ifndef BFM_SHUT_UP
	#define BFM_DEBUG(fmt, ...) printf(fmt, ##__VA_ARGS__);
#else
	#define BFM_DEBUG(fmt, ...)
#endif 
#define BFM_ERROR(fmt, ...) printf("\033[31m" "[Error] " fmt "\033[0m", ##__VA_ARGS__);


// Detailed see function `writePly`
enum ModelWriteMode {
	ModelWriteMode_Invalid = 0L << 0,
	ModelWriteMode_None = 1L << 0, 
	ModelWriteMode_PickLandmark = 1L << 1,
	ModelWriteMode_CameraCoord = 1L << 2,
	ModelWriteMode_NoExpr = 1L << 3,
};


// Function status used for return
enum BfmStatus {
	BfmStatus_Error = 0,
	BfmStatus_Ok = 1
};


// Constant
const unsigned int N_DLIB_LANDMARK = 68;
const unsigned int N_EXT_PARAMS = 6;	// roll, yaw, pitch, tx, ty, tz
const unsigned int N_INT_PARAMS = 4;	// fx, fy, cx, cy


#endif // BFM_CONSTANT_H