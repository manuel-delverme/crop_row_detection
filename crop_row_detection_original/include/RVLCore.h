// RVLCore.h

//#pragma once
//
//using namespace System;
//
//namespace RVLCore {
//
//	public ref class Class1
//	{
//		// TODO: Add your methods for this class here.
//	};
//}

#include "Platform.h"

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "cv.h"
#include "RVLConst.h"
#include "RVLBuffer.h"
#include "RVLMem.h"
#include "RVLArray.h"
#include "RVLQListArray.h"
#include "RVLWChain.h"
#include "RVLMChain.h"
#include "RVLUtil.h"
#include "RVLCamera.h"
#include "RVLParameterList.h"
#include "RVLGUI.h"
#include "RVLAImage.h"
#include "RVLStereoVision.h"
#include "RVLVisionSystem.h"


extern CvMat *RVLMatrix31;
extern CvMat *RVLMatrixHeader31;
extern CvMat *RVLMatrix33;
extern CvMat *RVLMatrixHeaderA33;
extern CvMat *RVLMatrixHeaderB33;
extern CvMat *RVLMatrixHeaderC33;
extern double RVLVector3[3];
extern double RVLMatrixA33[9];
extern double RVLMatrixB33[9];
extern double RVLMatrixC33[9];
extern int chi2_LookUpTable[1201];
extern unsigned char RVLColorMap[64 * 3];
extern double lnFactorial_LookUpTable[101];