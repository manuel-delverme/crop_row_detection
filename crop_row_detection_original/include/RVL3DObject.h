// RVL3DObject3.h: interface for the CRVL3DObject3 class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_RVL3DOBJECT3_H__35293DEF_4C46_498A_AE60_985EB631A319__INCLUDED_)
#define AFX_RVL3DOBJECT3_H__35293DEF_4C46_498A_AE60_985EB631A319__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "RVLObject2.h"

struct RVL3DMESH
{
	RVLQLIST TriangleList;
	RVLQLIST VertexList;
	int nVertices;
};

class CRVL3DObject : public CRVLObject2  
{
public:
	virtual double * GetCX();
	virtual double * GetX();
	/*virtual void UpdateX(	CRVL3DObject *pSObject,
							CRVL3DPose *pPoseSM);*/
	/*virtual BOOL Match(	CRVL3DObject *pSObject, 
						CRVL3DPose *pPose,
						double &MatchQuality);*/
	//virtual void UncertTransf(CRVL3DPose *pPose);
	//virtual void Transf(CRVL3DPose *pPose);
	virtual void UpdateParams();
	//CRVL3DObject();
	virtual ~CRVL3DObject();

	//inline BYTE *GetProjection()
	//{
	//	return m_pData + m_pClass->m_iDataProjectPtr;
	//};	
	//virtual CRVL2DObject3 * GetProjection();
};

#endif // !defined(AFX_RVL3DOBJECT3_H__35293DEF_4C46_498A_AE60_985EB631A319__INCLUDED_)
