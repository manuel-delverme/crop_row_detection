// RVLParameterList.cpp: implementation of the CRVLParameterList class.
//
//////////////////////////////////////////////////////////////////////

#include "RVLCore.h"
#include "RVLParameterList.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CRVLParameterList::CRVLParameterList()
{
	m_ParamNameLen = 200;
}

CRVLParameterList::~CRVLParameterList()
{

}

void CRVLParameterList::SetParam(char *line)
{
	char ParamName[200];

	char *value;

	if(GetParam(line, ParamName, &value))
		SetParam(ParamName, value);
}

void CRVLParameterList::SetParam(char *ParamName, 
								 char *value)
{
	RVLPARAM_DATA *pParamData;
	DWORD *pDWParam;
	int *piParam;
	double *pfParam;
	char **pstrParam;

	m_List.Start();

	while(m_List.m_pNext)
	{
		pParamData = (RVLPARAM_DATA *)(m_List.GetNext());

		if(strcmp(ParamName, pParamData->ParamName) == 0)
		{
			switch(pParamData->Type){
			case RVLPARAM_TYPE_FLAG:
				pDWParam = (DWORD *)(pParamData->pParam);

				RVLPARAM_ID_DATA *pFlagData;

				pParamData->pFlagIDList->Start();

				while(pParamData->pFlagIDList->m_pNext)
				{
					pFlagData = (RVLPARAM_ID_DATA *)(pParamData->pFlagIDList->GetNext());

					if(strcmp(value, pFlagData->IDName) == 0)
					{
						*pDWParam |= pFlagData->Data;

						break;
					}					
				}
				
				break;
			case RVLPARAM_TYPE_INT:
				piParam = (int *)(pParamData->pParam);

				sscanf(value, "%d", piParam);

				break;
			case RVLPARAM_TYPE_DOUBLE:
				pfParam = (double *)(pParamData->pParam);

				sscanf(value, "%lf", pfParam);

				break;
			case RVLPARAM_TYPE_STRING:
				pstrParam = (char **)(pParamData->pParam);

				if(*pstrParam)
					delete[] (*pstrParam);

				*pstrParam = RVLCreateString(value);

				break;
			case RVLPARAM_TYPE_ID:
				pDWParam = (DWORD *)(pParamData->pParam);

				RVLPARAM_ID_DATA *pIDData;

				pParamData->pFlagIDList->Start();

				while(pParamData->pFlagIDList->m_pNext)
				{
					pIDData = (RVLPARAM_ID_DATA *)(pParamData->pFlagIDList->GetNext());

					if(strcmp(value, pIDData->IDName) == 0)
					{
						*pDWParam = pIDData->Data;

						break;
					}
				}

				break;
			}

			return;
		}
	}
}

BOOL CRVLParameterList::GetParam(char *line, 
								 char *param, 
								 char **pvalue)
{
	if(line[0] == '%')
		return FALSE;

	int linelen = strlen(line);

	if(line[linelen - 1] == 10)
		line[linelen - 1] = 0;

	char *equal = ": ";

	char *value = strstr(line, equal);

	if(value)
	{
		int paramlen = value - line;

		memcpy(param, line, paramlen);

		param[paramlen] = 0;
		
		*pvalue = value + 2;

		return TRUE;
	}
	else
		return FALSE;
}

void CRVLParameterList::LoadParams(char *ParamFileName)
{
	FILE *fp = fopen(ParamFileName, "r");

	char line[200];

	while(TRUE)
	{
		fgets(line, 200, fp);

		if(strstr(line, "end") == line)
			break;

		SetParam(line);
	}

	fclose(fp);	
}

void CRVLParameterList::SaveParams(char *ParamFileName)
{
	FILE *fp = fopen(ParamFileName, "w");

	RVLPARAM_DATA *pParamData;
	int ivalue;
	double fvalue;
	char *pstrvalue;
	DWORD DWvalue;

	m_List.Start();

	while(m_List.m_pNext)
	{
		pParamData = (RVLPARAM_DATA *)(m_List.GetNext());

		fprintf(fp, "%s: ", pParamData->ParamName);

		switch(pParamData->Type){
		case RVLPARAM_TYPE_FLAG:

			break;
		case RVLPARAM_TYPE_INT:
			ivalue = *((int *)(pParamData->pParam));

			fprintf(fp, "%d\n", ivalue);

			break;
		case RVLPARAM_TYPE_DOUBLE:
			fvalue = *((double *)(pParamData->pParam));

			fprintf(fp, "%lf\n", fvalue);

			break;
		case RVLPARAM_TYPE_STRING:
			pstrvalue = *((char **)(pParamData->pParam));

			fprintf(fp, "%s\n", pstrvalue);

			break;
		case RVLPARAM_TYPE_ID:
			DWvalue = *((DWORD *)(pParamData->pParam));

			RVLPARAM_ID_DATA *pIDData;

			pParamData->pFlagIDList->Start();

			while(pParamData->pFlagIDList->m_pNext)
			{
				pIDData = (RVLPARAM_ID_DATA *)(pParamData->pFlagIDList->GetNext());

				if(DWvalue == pIDData->Data)
				{
					fprintf(fp, "%s", pIDData->IDName);

					break;
				}
			}

			fprintf(fp, "\n");

			break;
		}	// switch(pParamData->Type)
	}	// while(m_List.m_pNext)

	fprintf(fp, "end\n");

	fclose(fp);
}

void CRVLParameterList::Init()
{
	m_List.Create(m_pMem, sizeof(RVLPARAM_DATA));
}

void CRVLParameterList::Clear()
{
	m_List.RemoveAll();
}

RVLPARAM_DATA * CRVLParameterList::AddParam(char *ParamName, 
											DWORD Type,
											void *pParam)
{
	RVLPARAM_DATA ParamData;

	ParamData.ParamName = (char *)(m_pMem->Alloc(m_ParamNameLen));

	strcpy(ParamData.ParamName, ParamName);

	ParamData.pParam = pParam;
	ParamData.Type = Type;

	if(Type == RVLPARAM_TYPE_ID || Type == RVLPARAM_TYPE_FLAG)
	{
		ParamData.pFlagIDList = (CRVLMChain *)(m_pMem->Alloc(sizeof(CRVLMChain)));

		ParamData.pFlagIDList->Create(m_pMem, sizeof(RVLPARAM_ID_DATA));
	}

	m_List.Add(&ParamData);

	return (RVLPARAM_DATA *)(m_List.GetLast());
}

void CRVLParameterList::AddID(RVLPARAM_DATA *pParamData, 
							  char *IDName, 
							  DWORD Data)
{
	if(pParamData->Type != RVLPARAM_TYPE_ID && pParamData->Type != RVLPARAM_TYPE_FLAG)
		return;

	RVLPARAM_ID_DATA IDData;

	IDData.IDName = (char *)(m_pMem->Alloc(strlen(IDName) + 1));

	strcpy(IDData.IDName, IDName);

	IDData.Data = Data;

	pParamData->pFlagIDList->Add(&IDData);
}

BOOL CRVLParameterList::LoadParam(char *ParamFileName, 
								  char * ParamName)
{
	FILE *fp = fopen(ParamFileName, "r");

	char line[200];

	char *value;
	char ParamName2[200];

	while(TRUE)
	{
		fgets(line, 200, fp);

		if(strstr(line, "end") == line)
			break;

		GetParam(line, ParamName2, &value);

		if(strcmp(ParamName2, ParamName) == 0)
		{
			SetParam(ParamName, value);

			fclose(fp);

			return TRUE;
		}
	}

	fclose(fp);

	return FALSE;
}
