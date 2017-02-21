/* for detecting the source of memory leaks */
//#include "crtdbg.h"  
//#define new new(_NORMAL_BLOCK, __FILE__, __LINE__)

//OR
#define _CRTDBG_MAP_ALLOC
#include <stdlib.h>
//#include <crtdbg.h>



#define RVLWIN
//#define RVLOPENNI
//#define RVLVTK
//#define RVL3DOBJ2
//#define RVL_DEBUG   // for debugging purposes 
//#define RVL_LOG_FILE
//#define SVS
//#define RVLPLEA
//#define HEIV
//#define SURF
//#define RVLSTEREO_CORRESPONDENCE_SAD_ARRAY
//#define RVLPFF
//#define LOG_FILE
//#define LOG_FILE_EDGE_LINES
//#define LOG_FILE_EDGE_CONTOURS
//#define LOG_FILE_EDGE_CURVES
//#define LOG_FILE_EDGE_MAP
//#define LOG_FILE_STEREO_CORRESP
//#define LOG_FILE_STEREO_VISION
//#define LOG_FILE_3D_INDEXING
//#define LOG_FILE_INDEXING
//#define LOG_FILE_STAIRCASE_RECOGNITION
//#define LOG_FILE_3D_SCENE_CORRESP
//#define DAT_FILE_STAIRCASE_RECOGNITION
//#define LOG_FILE_BACK_PROJECTION

#define _CRT_SECURE_NO_WARNINGS 
