// RVL2DPose.h: interface for the CRVL2DPose class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_RVL2DPOSE_H__D0610C42_09E2_47F1_8EA9_F7857EE0971B__INCLUDED_)
#define AFX_RVL2DPOSE_H__D0610C42_09E2_47F1_8EA9_F7857EE0971B__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#define RVL2DPOSE_PARAM_FLAGS_U				0x00000001
#define RVL2DPOSE_PARAM_FLAGS_IU			0x00000002

class CRVL2DPose  
{
public:
	DWORD m_ParamFlags;
	int m_iU[2];			// [VAR 0]
	double m_U[2];			// [VAR 1]
	double m_Phi;			// [VAR 2]
	double m_cs, m_sn;		// [VAR 3]
	double *m_C;			// [VAR 4]

public:
	void UpdateiU(CRVLCamera *pCamera);
	CRVL2DPose();
	virtual ~CRVL2DPose();

};

#endif // !defined(AFX_RVL2DPOSE_H__D0610C42_09E2_47F1_8EA9_F7857EE0971B__INCLUDED_)
