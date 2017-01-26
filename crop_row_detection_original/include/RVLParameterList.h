// RVLParameterList.h: interface for the CRVLParameterList class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_RVLPARAMETERLIST_H__953E0474_4446_4207_9FF5_4472D2843253__INCLUDED_)
#define AFX_RVLPARAMETERLIST_H__953E0474_4446_4207_9FF5_4472D2843253__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#define RVLPARAM_TYPE_FLAG		0
#define RVLPARAM_TYPE_INT		1
#define RVLPARAM_TYPE_DOUBLE	2
#define RVLPARAM_TYPE_STRING	3
#define RVLPARAM_TYPE_ID		4

struct RVLPARAM_ID_DATA
{
	char *IDName;
	DWORD Data;
};

struct RVLPARAM_DATA
{
	char *ParamName;
	void *pParam;	
	DWORD Type;
	CRVLMChain *pFlagIDList;
};


class CRVLParameterList  
{
public:
	int m_ParamNameLen;
	CRVLMChain m_List;
	CRVLMem *m_pMem;

public:
	void AddID(RVLPARAM_DATA *pParamData, 
			   char *IDName,  
			   DWORD Data);
	RVLPARAM_DATA * AddParam(char *ParamName, 
							 DWORD Type,
							 void *pParam);
	void Clear();
	void Init();
	void LoadParams(char *ParamFileName);
	void SaveParams(char *ParamFileName);
	void SetParam(char *line);
	BOOL GetParam(char *line, 
				  char *param, 
				  char **pvalue);
	CRVLParameterList();
	virtual ~CRVLParameterList();
	BOOL LoadParam(	char *ParamFileName, 
					char * ParamName);
	void SetParam(	char *ParamName, 
					char *value);
};

#endif // !defined(AFX_RVLPARAMETERLIST_H__953E0474_4446_4207_9FF5_4472D2843253__INCLUDED_)
