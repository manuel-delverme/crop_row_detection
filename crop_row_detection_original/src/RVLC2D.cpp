// RVLC2D.cpp: implementation of the CRVLC2D class.
//
//////////////////////////////////////////////////////////////////////

//#include "stdafx.h"

#include "RVLCore.h"
#include "RVLC2D.h"

//#ifdef _DEBUG
//#undef THIS_FILE
//static char THIS_FILE[]=__FILE__;
//#define new DEBUG_NEW
//#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CRVLC2D::CRVLC2D()
{
	//memset(&m_ROI, 0, sizeof(RVLRECT));
	m_minz = 100.0;		// mm
}

CRVLC2D::~CRVLC2D()
{

}

void CRVLC2D::Init()
{
	m_dpRow = m_Array.m_ElementSize * m_Array.m_Width;

	m_dvpNeighbor4[0] = m_Array.m_ElementSize;
	m_dvpNeighbor4[1] = m_dpRow;
	m_dvpNeighbor4[2] = -m_Array.m_ElementSize;
	m_dvpNeighbor4[3] = -m_dpRow;
	
	m_dvpNeighbor8[0] = m_Array.m_ElementSize;
	m_dvpNeighbor8[1] = m_dpRow + m_Array.m_ElementSize;
	m_dvpNeighbor8[2] = m_dpRow;
	m_dvpNeighbor8[3] = m_dpRow - m_Array.m_ElementSize;
	m_dvpNeighbor8[4] = -m_Array.m_ElementSize;
	m_dvpNeighbor8[5] = -m_dpRow - m_Array.m_ElementSize;
	m_dvpNeighbor8[6] = -m_dpRow;
	m_dvpNeighbor8[7] = -m_dpRow + m_Array.m_ElementSize;

	m_dvpEdge[0][0] = m_dvpNeighbor8[0];
	m_dvpEdge[0][1] = m_dvpNeighbor8[1];
	m_dvpEdge[1][0] = m_dvpNeighbor8[1];
	m_dvpEdge[1][1] = m_dvpNeighbor8[2];
	m_dvpEdge[2][0] = m_dvpNeighbor8[2];
	m_dvpEdge[2][1] = 0;
	m_dvpEdge[3][0] = 0;
	m_dvpEdge[3][1] = m_dvpNeighbor8[0];

	m_dvpEdge2[0] = m_dvpNeighbor8[0];
	m_dvpEdge2[1] = m_dvpNeighbor8[2];
	m_dvpEdge2[2] = 0;
	m_dvpEdge2[3] = 0;

	m_EdgeShift[0] = 8;
	m_EdgeShift[1] = 0;
	m_EdgeShift[2] = 8;
	m_EdgeShift[3] = 0;

	m_dvpNextEdge[0] = m_dvpNeighbor8[0];
	m_dvpNextEdge[1] = m_dvpNeighbor8[2];
	m_dvpNextEdge[2] = m_dvpNeighbor8[4];
	m_dvpNextEdge[3] = m_dvpNeighbor8[6];

	m_dpInitEdge[0] = 0;
	m_dpInitEdge[1] = 0;
	m_dpInitEdge[2] = m_dvpNeighbor8[4];
	m_dpInitEdge[3] = m_dvpNeighbor8[6];

	//m_duEdge[0] = 3;
	//m_duEdge[1] = 2;
	//m_duEdge[2] = 1;
	//m_duEdge[3] = 2;

	//m_dvEdge[0] = 2;
	//m_dvEdge[1] = 3;
	//m_dvEdge[2] = 2;
	//m_dvEdge[3] = 1;
/*
	if(m_ROI.top == m_ROI.bottom)
	{
		m_ROI.left = 0;
		m_ROI.top = 0;
		m_ROI.right = m_Array.m_Width - 1;
		m_ROI.bottom = m_Array.m_Height - 1;
	}*/
}

