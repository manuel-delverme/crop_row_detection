// RVLC2D.h: interface for the CRVLC2D class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_RVLC2D_H__81BB923D_A347_4BF5_84A0_50460A314AB5__INCLUDED_)
#define AFX_RVLC2D_H__81BB923D_A347_4BF5_84A0_50460A314AB5__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "RVLClass.h"

#define RVLC2D_FLAG_TYPE				0x003F0000
#define RVLC2D_FLAG_TYPE_SIFT			0x00010000
#define RVLC2D_FLAG_TYPE_SURF			0x00020000
#define RVLC2D_FLAG_TYPE_RSIFT			0x00040000
#define RVLC2D_FLAG_TYPE_GFTT			0x00080000
#define RVLC2D_FLAG_TYPE_HSIFT			0x00100000
#define RVLC2D_FLAG_TYPE_SISP			0x00200000
#define RVLC2D_FLAG_HOG					0x00400000
#define RVLC2D_FLAG_SIFT_FROM_FILE		0x00000001

struct RVLIPOINT
{
	int u, v;
	void *vp;
};

class CRVLC2D : public CRVLClass  
{
public:
	RVL2DARRAY m_Array;
	int m_dpRow;
	int m_dvpNeighbor4[4];
	int m_dvpNeighbor8[8];
	int m_dvpEdge[4][2];
	int m_dvpEdge2[4];
	int m_EdgeShift[4];
	int m_dvpNextEdge[4];
	int m_dpInitEdge[4];
	//int m_duEdge[4];
	//int m_dvEdge[4];
	//RVLRECT m_ROI;
	double m_stdU;
	double m_stdd;
	double m_minz;

public:
	void Init();
	CRVLC2D();
	virtual ~CRVLC2D();

};

#endif // !defined(AFX_RVLC2D_H__81BB923D_A347_4BF5_84A0_50460A314AB5__INCLUDED_)
