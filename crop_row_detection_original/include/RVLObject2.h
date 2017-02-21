// RVLObject2.h: interface for the CRVLObject2 class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_RVLOBJECT2_H__2B060E00_DB5D_4CD7_8F30_829640561FBE__INCLUDED_)
#define AFX_RVLOBJECT2_H__2B060E00_DB5D_4CD7_8F30_829640561FBE__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#define RVLRELLIST_INDEX_NEIGHBORS				0
#define RVLRELLIST_INDEX_COMPONENTS				1	
#define RVLRELLIST_INDEX_SUPEROBJECTS			2
#define RVLRELLIST_INDEX_ELEMENT_SUPEROBJECTS	1
#define RVLOBJ2_FLAG_SELECTED				0x00000001
#define RVLOBJ2_FLAG_MATCHED				0x00000002
#define RVLOBJ2_FLAG_MATCHED_CORRECTLY		0x00000004
#define RVLOBJ2_FLAG_NONE					0x00000008
#define RVLOBJ2_FLAG_REJECTED				0x00000010
#define RVLOBJ2_FLAG_SEED					0x00000020
#define RVLOBJ2_FLAG_MARKED					0x00000040
#define RVLOBJ2_FLAG_DELETED				0x00000080
#define RVLOBJ2_FLAG_SINGLEMATCH			0x00000100
#define RVLOBJ2_FLAG_MARKED2				0x00000200
#define RVLOBJ2_FLAG_MARKED3				0x00000400
#define RVLOBJ2_FLAG_DOMINANT				0x00000800
#define RVLOBJ2_NOUNCERT					0x01

void RVLDeselectAllObjects(CRVLClass *pClass);
void RVLClearFlags(CRVLClass *pClass);

class CRVLObject2;

struct RVLOBJ2_CONNECTION
{
	void *vpObject;
	BYTE jConnector;
};

struct RVLMATCH
{
	CRVLObject2 *pObject;
	DWORD Quality;
	DWORD Flags;
};

struct RVL3DPOINT2
{
	void *vp2DRegion;
	int u, v, d;
	int iPix;
	int iPixRGB;
	double x, y, z;
	double XYZ[3];
	double pod;
	double weight;
	double e;
	int index;
	int segmentNumber;
	int refSegmentNumber;
	BYTE dIProjAngle;
	int dI;
	int iCell;
	//CRVLMPtrChain regionList;
	DWORD Flags;
};


void RVLAddObject(CRVLObject2 *pObject, 
				  CRVLClass *pClass);
void RestoreFromPermanentMem(CRVLClass *pClass);

#ifdef RVL_DEBUG
void RVLDebugPrintObjects(CRVLMPtrChain *pObjectArray,
						  char *Name);
#endif

class CRVLObject2  
{
public:
	BYTE *m_pData;			// [VAR 80 - bf] (See CRVLClass)
	CRVLClass *m_pClass;
	WORD m_Index;			// [VAR 00]
	DWORD m_Flags;
	DWORD m_ParamFlags;
	RVLARRAY *m_RelList;	// [VAR c0 - ff] (See CRVLClass)
public:
	virtual void Clear();
#ifdef RVL_DEBUG
	virtual void DebugPrint(char *Name);
#endif
	/*void GetRelList(CRVLMPtrChain2 *pRelList, 
		            int RelListID,
					CRVLMem *pMem = NULL);*/
	CRVLObject2 * GetMatch();
	virtual int MatchCost(CRVLObject2 *pObject);
	/*virtual void GetMatchCandidates(CRVLCamera *pCamera,
									CRVL2DCellArray *pCellArray,
									double sigS,
									CRVLMPtrChain *pMatchCandidateArray);*/
	virtual int MaxMatchQuality();
	virtual BOOL Match(CRVLObject2 *pObject, 
					   int &MatchQuality);
	virtual void Create(CRVLClass *pClass);
	virtual CRVLObject2 * Create2(CRVLClass *pClass);
	CRVLObject2 * Create3(CRVLClass *pClass);
	CRVLObject2();
	virtual ~CRVLObject2();
	virtual void Save(	FILE *fp, 
						DWORD Flags);
	virtual void Load(	FILE *fp, 
						DWORD Flags);

	inline BYTE * GetData(int DataID)
	{
		return m_pData + m_pClass->m_iData[DataID];
	};
};

#endif // !defined(AFX_RVLOBJECT2_H__2B060E00_DB5D_4CD7_8F30_829640561FBE__INCLUDED_)
