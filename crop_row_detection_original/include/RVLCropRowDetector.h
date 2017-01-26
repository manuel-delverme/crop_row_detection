#pragma once
#include "time.h"

//#define RVLCRD_L1

struct RVLCRD_DPDATA
{
	int c;
	int id;
	double D;
	double minBV;
	double B;
};

class CRVLCropRowDetector
{
public:
	CRVLCropRowDetector(void);
	virtual ~CRVLCropRowDetector(void);
	void Apply(unsigned char * I, int w, int imNum);
	void Init(int h);
	void Display(unsigned char * DisplayImage, int w);
	void CreateParamList(CRVLMem * pMem);
	void ExGImage(unsigned char * I, unsigned char * dst, int width, int height);
	void DoubleOtsu(IplImage* I, IplImage* thresh);
	double Otsu(CvHistogram* hist);
	void HSVThreshold(IplImage* I, IplImage* thresh, CvScalar lower_bound, CvScalar upper_bound);
	void TripleOtsu(IplImage* I, IplImage* thresh);

public:
	int m_mind;
	int m_ndOctaves;
	int m_ndSamplesPerOctave;
	int m_h;
	int *m_c;
	int *m_id;
	int *m_d;
	int *m_idFilter;
	//NEW - visible detected row
	int m_visibleStart;
	int m_visibleStop;
	RVLCRD_DPDATA *m_DPData;
	double *m_bestScore;
	double m_a0;
	double m_b0;
	int m_d0;
	CRVLParameterList m_ParamList;
	double m_lambdac;
	double m_lambdad;
	double m_maxD;
	int m_nc;
	int m_nd;
	double m_vegetationImageTime;
	double m_templateMatchingTime;
	double m_optimizationTime;
	double m_totalTime;

private:
	double m_dstep;
	clock_t startTotal;
};
