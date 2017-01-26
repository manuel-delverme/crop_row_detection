// RVLBuffer.cpp: implementation of the CRVLBuffer class.
//
//////////////////////////////////////////////////////////////////////

//#include "stdafx.h"

#include "Platform.h"

#include <math.h>

#include "RVLConst.h"
#include "RVLBuffer.h"

#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CRVLBuffer::CRVLBuffer()
{	
	m_bOwnData = TRUE;
}

CRVLBuffer::~CRVLBuffer()
{
	if(m_bOwnData)
		delete[] DataBuff;
}
