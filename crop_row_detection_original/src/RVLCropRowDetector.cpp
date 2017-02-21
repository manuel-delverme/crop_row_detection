#include "RVLCore.h"
#include "RVLCropRowDetector.h"
#include "time.h"
#include <iostream>
//#include "timer.h"

//#define RVLCRD_DEBUG

using namespace std;

CRVLCropRowDetector::CRVLCropRowDetector(void)
{
	m_mind = 8;
	m_ndOctaves = 5;
	m_ndSamplesPerOctave = 70;
	m_a0 = 1;
	m_b0 = 3;
	m_d0 = 8;
	m_lambdac = 0.5;
	m_lambdad = 0.2;
	m_maxD = 1.0;

	m_c = NULL;
	m_id = NULL;
	m_d = NULL;
	m_idFilter = NULL;
	//NEW - visible detected row
	//m_visible = NULL;
	m_DPData = NULL;
	m_bestScore = NULL;
}

CRVLCropRowDetector::~CRVLCropRowDetector(void)
{
	if(m_c)
		delete[] m_c;

	if(m_id)
		delete[] m_id;

	if(m_d)
		delete[] m_d;

	if(m_idFilter)
		delete[] m_idFilter;

	//NEW - visible detected row
	//if(m_visible)
	//	delete[] m_visible;

	if(m_DPData)
		delete[] m_DPData;

	if(m_bestScore)
		delete[] m_bestScore;
}

void CRVLCropRowDetector::CreateParamList(CRVLMem * pMem)
{
	m_ParamList.m_pMem = pMem;

	RVLPARAM_DATA *pParamData;

	m_ParamList.Init();

	pParamData = m_ParamList.AddParam("CRD.a0", RVLPARAM_TYPE_DOUBLE, &m_a0);
	pParamData = m_ParamList.AddParam("CRD.b0", RVLPARAM_TYPE_DOUBLE, &m_b0);
	pParamData = m_ParamList.AddParam("CRD.d0", RVLPARAM_TYPE_INT, &m_d0);
	pParamData = m_ParamList.AddParam("CRD.mind", RVLPARAM_TYPE_INT, &m_mind);
	pParamData = m_ParamList.AddParam("CRD.ndOctaves", RVLPARAM_TYPE_INT, &m_ndOctaves);
	pParamData = m_ParamList.AddParam("CRD.ndSamplesPerOctave", RVLPARAM_TYPE_INT, &m_ndSamplesPerOctave);
	pParamData = m_ParamList.AddParam("CRD.lambdac", RVLPARAM_TYPE_DOUBLE, &m_lambdac);
	pParamData = m_ParamList.AddParam("CRD.lambdad", RVLPARAM_TYPE_DOUBLE, &m_lambdad);
	pParamData = m_ParamList.AddParam("CRD.maxD", RVLPARAM_TYPE_DOUBLE, &m_maxD);
}

void CRVLCropRowDetector::Init(int h)
{
	m_h = h;

	if(m_c)
		delete[] m_c;

	m_c = new int[h];

	if(m_id)
		delete[] m_id;

	m_id = new int[h];

	if(m_d)
		delete[] m_d;

	m_d = new int[h];

	if(m_idFilter)
		delete[] m_idFilter;

	m_idFilter = new int[h];

	//NEW - visible detected row
	//if(m_visible)
	//	delete[] m_visible;

	//m_visible = new int[h];

	if(m_DPData)
		delete[] m_DPData;

	m_nd = m_ndOctaves * m_ndSamplesPerOctave + 1;

	m_dstep = pow(2.0, 1.0 / (double)m_ndSamplesPerOctave);

	m_nc = (int)floor((double)m_mind * pow(m_dstep, m_nd)) + 1;

	m_DPData = new RVLCRD_DPDATA[m_h * m_nd * m_nc];

	if(m_bestScore)
		delete[] m_bestScore;

	m_bestScore = new double[h];
}

void CRVLCropRowDetector::Apply(unsigned char * I,
								int w, int imNum)
{

    // for(int u = 0; u < 10; u++){
    //     std::cout << "orig::I " << (double) I[u] << std::endl;
    // }
    startTotal = clock();
	double fa0 = (double)m_a0;
	double fb0 = (double)m_b0;
	double fd0 = (double)m_d0;

	double halfw = (double)(w / 2);

	int n = m_nc * m_nd;

	// assign score to each pair (c,d) for each image row

	int *II_ = new int[w + 1];

	int *II = II_ + 1;

	II[-1] = 0;

	unsigned char *I_ = I;	

	int *crange_ = new int[m_nd];

	RVLCRD_DPDATA *DP_ = m_DPData;

	int u, v;
	int id;
	int crange;
	int kStart, kEnd;
	int a, b, c;
	double d;
	double halfa, halfb, halfd;
	double fu, fub;
	double scale;
	int u0, u1, u2, u3;
	double Sa, Sb;
	int na, nb;
	int k;
	double score;
	int bestid, bestc, bestd;
	double fTmp;
	double bestScore;
	RVLCRD_DPDATA *DP__, *pDP, *pDP_;
	int c_position, crange2;

	//start Template Matching timer
	clock_t start, end;
	start = clock();

	//CPUTimer timer = new CPUTimer();
	//timer.Reset();
	//timer.Start();

	//FILE *fp, *fp2;
	//fp = fopen("Results\\score_cxd_205.txt", "w");
	//fp2 = fopen("Results\\score_cxd_count_205.txt", "w");
	
	//int debugCounter = 0;


	for(v = 0; v < m_h; v++, I_ += w, DP_ += n)
	{
		// integral image of the row
		II[0] = (int)(I_[0]);

		for(u = 1; u < w; u++)
			II[u] = II[u - 1] + I_[u];


		// convolutions

#ifdef RVLCRD_DEBUG
		int debug = 1;
#endif

		bestScore = 0.0;

		d = (double)m_mind;

		DP__ = DP_;



		for(id = 0; id < m_nd; id++, d *= m_dstep, DP__ += m_nc) {

#ifdef RVLCRD_DEBUG
            if(d > 10.0 * debug)
                debug++;
#endif
			crange = (int) floor(0.5 * d);

			crange_[id] = crange;

			scale = d / fd0;
			fTmp = floor(scale * fa0 + 0.5);
			a = (int) fTmp;
			halfa = 0.5 * fTmp;
			fTmp = floor(scale * fb0 + 0.5);
			b = (int) fTmp;
			halfb = 0.5 * fTmp;
			halfd = 0.5 * d;
			fub = halfd - halfb;

			pDP = DP__ + m_nc / 2 - crange;

			int asd = m_nc / 2 - crange;

			//FOR score IMAGE (d x c)
			//if(v == 30)
			//{
			//	fprintf(fp, "\n");
			//	fprintf(fp2, "%d\n", crange);
			//}
			//END IMAGE

			//cout << d << endl;

			for (c = -crange; c < crange; c++, pDP++, asd++) {

//cout << a << " " << b << " " << fa0 << " " << fb0 << endl;
				//cout << c << endl;



				kStart = (int) floor((-(double) (w / 2 + c) + halfa) / d);
				kEnd = (int) floor(((double) (w / 2 - c) + halfa) / d);


				fu = (double) c + (double) kStart * d + halfw;
/*if(v==0)
				cout << fu << " " << halfa << " " << fub << endl;*/

				u0 = cvFloor(fu - halfa);

				u1 = u0 + a - 1;


				u2 = DOUBLE2INT(fu + fub);

				u3 = u2 + b - 1;
				/*if(v==0)
                std::cout << halfa << " " << u0 << " " << u1 << " " << u2 << " " << u3 << std::endl;*/

				if (u1 >= 0) {
					Sa = II[u1];
					/*if(v==6)
						cout << II[u1] << endl;*/
					na = u1;

					//debugCounter++;
				} else {
					Sa = 0;
					na = 0;
				}


				if (u2 < 0)
					u2 = 0;

				if (u3 >= 0) {
					if (u2 > 0)
						Sb = (II[u3] - II[u2 - 1]);
					else if (u2 <= 0)
						Sb = (II[u3] - II[u2]);
					/*if(v==0)
					cout << "s "<< II[u3] << " " << -II[u2 ] << " " << u3 << " " << u2-1 << endl;*/
					nb = (u3 - u2 + 1);

					//debugCounter++;
				} else {
					Sb = 0;
					nb = 0;
				}

				fu += d;
				/*if(v==2)
					std::cout << u0 << " " << u1 << " " << u2 << " " << u3 << endl;//" " << fu << " " << fub << std::endl;*/
				/*int aaa;
				std::cin >> aaa;*/

				/*for(unsigned int i=0; i<20; i++) 
				  std::cout << I_[i] << std::endl;

			    for(unsigned int i=0; i<20; i++) 
			      std::cout << II[i] << std::endl;

			    int aaa;
			    std::cin >> aaa;*/

				for (k = kStart + 1; k < kEnd; k++, fu += d) {

					u0 = cvFloor(fu - halfa);

					u1 = u0 + a - 1;

					Sa += (II[u1] - II[u0 - 1]);
					na += (u1 - u0 + 1);

					u2 = DOUBLE2INT(fu + fub);

					u3 = u2 + b - 1;

					Sb += (II[u3] - II[u2 - 1]);
					nb += (u3 - u2 + 1);

					/*if(v==2)
						std::cout << k << " " << u0 << " " << u1 << " " << u2 << " " << u3 << endl;*/

					//std::cout << k << " " << u0 << " " << u1 << " " << u2 << " " << u3 << endl;//" " << fu << " " << std::endl;
					//std::cout << k << " " << Sa << " " << na << " " << Sb << " " << nb << std::endl;
					//std::cout << k << " " << II[u1] << " " << II[u0 - 1] << std::endl;


					//debugCounter += 2;
				}


				/*int aaa;
				std::cin >> aaa;*/
				u0 = cvFloor(fu - halfa);
				//cout << "u0:" << w <<  endl;

				u1 = u0 + a - 1;

				if (u1 >= w)
					//cout << "ah, ma tu guarda il merda" <<  endl;
					u1 = w - 1;

				//cout << "u1:" << u1 <<  endl;

				/*if(v==6)
                cout << II[u1] << " " << - II[u0 - 1] << endl;*/
				Sa += (II[u1] - II[u0 - 1]);
				na += (u1 - u0 + 1);

				//debugCounter++;

				u2 = DOUBLE2INT(fu + fub);
				//cout << "u2:" << u2 <<  endl;
				if (u2 < w) {
					u3 = u2 + b - 1;
					//cout << "u3:" << u3 <<  endl;

					if (u3 >= w)
						//cout << "una merdaviglia" <<  endl;
						u3 = w - 1;

					/*if(v==6)
					cout << "d"<<II[u3] << " " << - II[u2 - 1] << endl;*/
					Sb += (II[u3] - II[u2 - 1]);
					nb += (u3 - u2 + 1);

					//debugCounter++;
				}
				/*if(v==2)
					std::cout  << u0 << " " << u1 << " " << u2 << " " << u3 << endl;*/
				//score = (double)(nb * Sa - na * Sb) / (double)(na + nb);

/*if(v==0)
	cout << nb << " " << Sa << " " << na << " " << Sb << endl;*/

				score = (double) (nb * Sa - na * Sb) / (double) (na * nb);        // new score computation

				/*int aa;
                std::cin >> aa;*/

				/*if(v <=10)
					std::cout << "row: " << v << " score: " << score << " phase: " << d << " period: " << c << std::endl;*/

				//std::cout << d << " " << c << " " << na << " " << Sa << " " << Sb << " " << nb << " " << score << " " << II[u2 - 1] <<  std::endl;
				/*int aaa;
				std::cin >> aaa;*/

				pDP->D = score;

				//FOR score IMAGE (d x c)
				//if(v == 30)
				//{
				//	fprintf(fp, "%lf\t", score);
				//}

				//END IMAGE

				//copy D value for c values in range <-m_nc/2, m_nc/2>
				crange2 = 2 * crange;
				pDP_ = pDP + crange2; //pDP_ pointer for copying D value in c range <-m_nc/2, m_nc/2>
				int asd2 = asd + crange2;
				c_position = m_nc * 0.5 + c + crange2;

				while (c_position <= m_nc) {
					pDP_->D = score;

					pDP_ += crange2;
					asd2 += crange2;
					c_position += crange2;

				}

				pDP_ = pDP - crange2;
				asd2 = asd - crange2;
				c_position = m_nc * 0.5 + c - crange2;

				while (c_position >= 0) {
					pDP_->D = score;

					pDP_ -= crange2;
					asd2 -= crange2;
					c_position -= crange2;
				}


				if (score > bestScore) {
					bestScore = score;
					bestc = c;
					bestid = id;
					bestd = d;
				}

			}    // for(c = -crange; c < crange; c++)
		}
		m_c[v] = bestc;
		m_id[v] = bestid;
		m_d[v] = bestd;
		m_idFilter[v] = bestid;
		m_bestScore[v] = bestScore;
	}	// for(v = 0; v < h; v++, I_ += w)



    // FINE TEMPLATE MATCHING, INIZIO ALTRE COSE!!

	//std::cout << "Best period and phase candidates after TEMPLATE MATCHING!" << std::endl;

	/*for(unsigned int v=0; v<m_h; v++)
	{
		std::cout << "row: " << v << " c: " << m_c[v] << " d: " << m_d[v] << " best score: " << m_bestScore[v] << std::endl;
	}

	std::cin >> aa;*/

	//std::cout << "results after optimization" << std::endl;*/



	//fclose(fp);
	//fclose(fp2);

	//end Template Matching timer
	end = clock();
	m_templateMatchingTime = ((double)end - (double)start) / (double)CLOCKS_PER_SEC;

	//timer.End();
	//m_templateMatchingTime = timer.GetEllapsedMS();

	delete[] II_;

	// dynamic programing

	// m_DPData is a 3D data structure. It has h blocks, each block having m_nd rows and m_nc columns.
	// Each block corresponds to an image row.

    start = clock();
	DP_ = m_DPData;	// DP_ - ptr. to the first data in a block

	int iTmp = RVLMAX(m_nc, m_nd) + 1;

	int *v_ = new int[iTmp];
	//NEW d_development
	double *v__ = new double[iTmp];
	double *z = new double[iTmp];

	//int64 debugCounter1 = 0;
	//int64 debugCounter2 = 0;
	//int64 debugCounter3 = 0;
	//int64 debugCounter4 = 0;

	double maxz;
	double s;

	double Dnrm;

	//start Optimization timer
	//timer.Reset();
	//timer.Start();

	for(v = 0; v < m_h; v++, DP_ += n) {
		Dnrm = m_bestScore[v];
		DP__ = DP_;        // DP__ - ptr. to the first data in a block row
		for (id = 0; id < m_nd; id++, DP__ += m_nc) {
			crange = crange_[id];

			pDP = DP__;
			for (c = -m_nc / 2; c < m_nc / 2; c++, pDP++) {
				if (Dnrm >= 1.0) {
					pDP->B = 1.0 - pDP->D / Dnrm;

					if (pDP->B > m_maxD)
						pDP->B = m_maxD;
				} else
					pDP->B = m_maxD;

				if (v > 0)
					pDP->B += (pDP - n)->minBV;
			}
		}

		if (v < m_h - 1) {
			DP__ = DP_;
			maxz = (1.0 + m_lambdac * (double) (m_nc * m_nc)) / (2.0 * m_lambdac);
			for (id = 0; id < m_nd; id++, DP__ += m_nc) {
				crange = crange_[id];
				k = 0;
				v_[0] = -m_nc * 0.5;
				z[0] = -maxz;
				z[1] = maxz;
				pDP = DP__ + 1;

				for (c = -m_nc / 2 + 1; c < m_nc / 2; c++, pDP++) {
					while (true) {
						s = ((pDP->B + m_lambdac * (double) (c * c)) -
							 (DP__[v_[k] + m_nc / 2].B + m_lambdac * (double) (v_[k] * v_[k]))) /
							(2.0 * m_lambdac * (double) (c - v_[k]));

						if (s <= z[k])
							k--;
						else
							break;
					}
					k++;
					v_[k] = c;
					z[k] = s;
					z[k + 1] = maxz;
				}
				k = 0;

				pDP = DP__;
				for (c = -m_nc / 2; c < m_nc / 2; c++, pDP++) {
					while (z[k + 1] < (double) c) {
						k++;
					}

					iTmp = (c - v_[k]);
					pDP->minBV = DP__[v_[k] + m_nc / 2].B + m_lambdac * (double) (iTmp * iTmp);
					pDP->c = v_[k];
					pDP->id = id;
				}
			}    // for(id = 0; id < m_nd; id++)
			maxz = (1.0 + m_lambdad * (double) (m_nd * m_nd)) / (2.0 * m_lambdad);
			DP__ = DP_;

			for (c = 0; c < m_nc; c++, DP__++) {
				k = 0;
				v_[0] = 0; //for saving id
				v__[0] = (double) m_mind; //for saving d
				z[0] = -maxz;
				z[1] = maxz;

				pDP = DP__ + m_nc;
				d = (double) m_mind;

				for (id = 1; id < m_nd; id++, pDP += m_nc) {
					d *= m_dstep;
					while (true) {
						s = ((pDP->minBV + m_lambdad * (double) (d * d)) -
							 (DP__[m_nc * v_[k]].minBV + m_lambdad * (double) (v__[k] * v__[k]))) /
							(2.0 * m_lambdad * (double) (d - v__[k]));
						if (s <= z[k])
							k--;
						else
							break;
					}
					k++;
					v_[k] = id;
					v__[k] = d;
					z[k] = s;
					z[k + 1] = maxz;
				}

				k = 0;

				pDP = DP__;
				d = (double) m_mind;

				for (id = 0; id < m_nd; id++, pDP += m_nc) {
					while (z[k + 1] < d) {
						k++;
					}
					iTmp = (d - v__[k]);
					pDP->minBV = DP__[m_nc * v_[k]].minBV + (m_lambdad * (double) (iTmp * iTmp));
					pDP->c = DP__[m_nc * v_[k]].c;
					pDP->id = v_[k];
					d *= m_dstep;
				}

			}    //	for(c = 0; c < m_nc; c++)
		}    // if(v < m_h - 1)
	}
	DP_ = m_DPData + (m_h - 1) * n;
	DP__ = DP_;
	v = m_h - 1;

	RVLCRD_DPDATA *pBestNode = DP__;
	RVLCRD_DPDATA *ppp = m_DPData;
	d = (double)m_mind;

	for(id = 0; id < m_nd; id++, DP__ += m_nc, d *= m_dstep)
	{
		crange = crange_[id];
		pDP = DP__;
		for(c = -m_nc/2; c < m_nc/2; c++, pDP++)
			if(pDP->B < pBestNode->B)
			{
				pBestNode = pDP;
				bestc = c;
				bestid = id;
				bestd = d;
			}
	}

	m_c[v] = bestc;
	m_id[v] = bestid;
	m_d[v] = bestd;

	for(v = m_h - 2; v >= 0; v--) {
		pDP = pBestNode - n;

		DP_ -= n;

		pBestNode = DP_ + pDP->id * m_nc + m_nc / 2 + pDP->c;

		m_c[v] = pDP->c;
		m_id[v] = pDP->id;
		m_d[v] = (double)m_mind * pow(m_dstep, (double)(m_id[v]));
	}

	//end Optimization timer
	end = clock();
	m_optimizationTime = ((double)end - (double)start) / (double)CLOCKS_PER_SEC;
    std::cout << "optimization time: " << m_optimizationTime << std::endl;

	//m_totalTime = m_vegetationImageTime + m_templateMatchingTime + m_optimizationTime;
	m_totalTime = ((double)end - (double)startTotal) / (double)CLOCKS_PER_SEC;
	std::cout << "total time: " << m_totalTime << std::endl;

	delete[] crange_;
}

void CRVLCropRowDetector::Display(unsigned char * DisplayImage, int w)
{
	double halfw = (double)(w / 2);

	int u, v;
	int kStart, kEnd, k;
	double c, d;
	unsigned char *pPix;

	for(v = 0; v < m_h; v++)
	{
		//if(1/*v >= m_visibleStart && v <= m_visibleStop*/)
		//{
			c = (double)(m_c[v]);
			d = (double)m_mind * pow(m_dstep, (double)(m_id[v]));

			kStart = (int)floor(-(halfw + c) / d) + 1;
			kEnd = (int)floor((halfw - c) / d);		
			
			for(k = kStart; k <= kEnd; k++)
			{
				u = DOUBLE2INT(c + (double)k * d + halfw);

				pPix = DisplayImage + 3 * (u + v * w);

				*(pPix++) = 0;
				*(pPix++) = 0;
				*(pPix++) = 255;
				pPix = 0;
			}
		//}
	}
}

void CRVLCropRowDetector::ExGImage(unsigned char * I, unsigned char * dst, int width, int height)
{
	int i, cnt=0;
	double blueMax, greenMax, redMax;
	double blueNorm, greenNorm, redNorm;
	double blue, green, red;
	double sumNorm, ExG;

	//start Vegetation Image timer
	double start, end;
	startTotal = clock();

	//CPUTimer timer = new CPUTimer();
	//timer.Reset();
	//timer.Start();

	blueMax = I[0];
	greenMax = I[1];
	redMax = I[2];

	//find max in all chanels
	for(i=1; i<width*height; i++)
	{
		if(I[i*3] > blueMax)
			blueMax = I[i*3];

		if(I[i*3+1] > greenMax)
			greenMax = I[i*3+1];

		if(I[i*3+2] > redMax)
			redMax = I[i*3+2];
	}

    //std::cout << blueMax << " " << greenMax << " " << redMax << std::endl;  

	for(i=0; i<width*height; i++)
	{
		//normalize all pixels with max value of every channel
		blueNorm = (double) I[i*3] / blueMax;
		greenNorm = (double) I[i*3+1] / greenMax;
		redNorm = (double) I[i*3+2] / redMax;

		sumNorm = blueNorm + greenNorm + redNorm;

		//normalize every pixel so that sum of all channels is 1
		blue = blueNorm / sumNorm;
		green = greenNorm / sumNorm;
		red = redNorm / sumNorm;

		
		//ExG = 2*green - blue - red
		//ExG = abs((2*green - blue - red)*255.0);
		ExG = ((2*green - blue - red) > 0) ? (2*green - blue - red)*255.0 : 0;

		dst[i] = (unsigned char) ExG;
	}

	//end Vegetation Image timer
	end = clock();
	m_vegetationImageTime = ((double)end - (double)start) / (double)CLOCKS_PER_SEC;
}

void CRVLCropRowDetector::DoubleOtsu(IplImage* I, IplImage* thresh)
{
	CvHistogram* hist;
	int hist_size[] = {256};
	double t1, t2;
	int i;
	unsigned char *data;

	//first Otsu threshold		
	t1 = cvThreshold(I, thresh, NULL, 255, CV_THRESH_OTSU);

	//create mask to ignore pixels with intensity lower than t1
	IplImage* mask;
	mask = cvCreateImage(cvGetSize(I), IPL_DEPTH_8U, 1);
	cvZero(mask);

	//use data instead I->imageData for original DoubleOtsu (Montalvo et all)
	data = (unsigned char *)I->imageData;
	for(i=0; i < I->width * I->height; i++)
	{
		//if(I->imageData[i] > t1)
		if(data[i] > t1)
		{
			mask->imageData[i] = 1;
		}
	}

	//create histogram of new image (image I with mask)
	hist = cvCreateHist(1, hist_size, CV_HIST_ARRAY, NULL, 1);
	cvCalcHist(&I, hist, 0, mask);
	cvNormalizeHist(hist, 1.0);

	//calculate new threshold value using otsu method
	t2 = Otsu(hist);
	cvThreshold(I, thresh, t2, 255, CV_THRESH_BINARY);
	
	cvReleaseImage(&mask);
	cvReleaseHist(&hist);
}

double CRVLCropRowDetector::Otsu(CvHistogram* hist)
{
	double t2;
	int i;
	float w0, w1, u0, u1, sum_u0 = 0, sum_u1 = 0, F0, F1;

	t2 = 0;
	w0 = cvGetReal1D(hist,0);
	w1 = 1 - w0;

	sum_u0 = w0;

	//prema izrazu (14) iz "A threshold Selection Method from Gray-Level Histograms"
	//for(i = 1; i < 256; i++)
	//{
	//	sum_u1 += (i+1) * cvQueryHistValue_1D(hist,i);		
	//}

	//u0 = (w0 == 0) ? 0 : sum_u0/w0;
	//u1 = (w1 == 0) ? 0 : sum_u1/w1;

	//F0 = w0*w1*(u1-u0)*(u1-u0);

	//for(i=1; i<255; i++)
	//{
	//	w0 += cvQueryHistValue_1D(hist,i);	
	//	w1 = 1 - w0;

	//	sum_u0 += (i+1) * cvQueryHistValue_1D(hist,i);
	//	sum_u1 -= (i+1) * cvQueryHistValue_1D(hist,i);

	//	u0 = (w0 == 0) ? 0 : sum_u0/w0;
	//	
	//	u1 = (w1 == 0) ? 0 : sum_u1/w1;

	//	F1 = w0*w1*(u1-u0)*(u1-u0);

	//	if(F1 >= F0)
	//	{
	//		F0 = F1;
	//		t2 = i;
	//	}
	//}
	//end maksimizacija izraza (14)

	//prema izrazu (18) iz "A threshold Selection Method from Gray-Level Histograms" - koristi se u Matlabu
	float uT=0.0f, u=0.0f, w;

	t2 = 0;
	w = cvGetReal1D(hist,0);

	u = w;

	for(i = 0; i < 256; i++)
	{
		uT += (i+1) * cvGetReal1D(hist,i);		
	}

	//F0 = ((uT*w - u)*(uT*w - u))/(w*(1-w));
	F0 = (w == 0) ? 0 : ((uT*w - u)*(uT*w - u))/(w*(1-w));

	for(i=1; i<255; i++)
	{
		w += cvGetReal1D(hist,i);
		u += (i+1) * cvGetReal1D(hist,i);

		//F1 = ((uT*w - u)*(uT*w - u))/(w*(1-w));
		F1 = (w == 0) ? 0 : ((uT*w - u)*(uT*w - u))/(w*(1-w));

		if(F1 >= F0)
		{
			F0 = F1;
			t2 = i;
		}
	}
	//end maksimizacija izraza (18)

	return t2;
}

void CRVLCropRowDetector::HSVThreshold(IplImage* I, IplImage* thresh, CvScalar lower_bound, CvScalar upper_bound)
{
	//OpenCV HSV has ranges H: 0-180; S: 0-255; V: 0-255

	//convert BGR image to HSV image
	cvCvtColor(I, I, CV_BGR2HSV);

	//apply HSV threshold
	cvInRangeS(I, lower_bound, upper_bound, thresh);

	//return original image to BGR
	cvCvtColor(I, I, CV_HSV2BGR);
}

void CRVLCropRowDetector::TripleOtsu(IplImage* I, IplImage* thresh)
{
	CvHistogram* hist;
	int hist_size[] = {256};
	double t1, t2, t3;
	int i;
	unsigned char *data;

	//first Otsu threshold		
	t1 = cvThreshold(I, thresh, NULL, 255, CV_THRESH_OTSU);

	//create mask to ignore pixels with intensity lower than t1
	IplImage* mask;
	mask = cvCreateImage(cvGetSize(I), IPL_DEPTH_8U, 1);
	cvZero(mask);

	data = (unsigned char *)I->imageData;

	//create mask for first threshold
	for(i=0; i < I->width * I->height; i++)
	{		
		if(data[i] > t1)
		{
			mask->imageData[i] = 1;
		}
	}

	//create histogram of new image (image I with mask)
	hist = cvCreateHist(1, hist_size, CV_HIST_ARRAY, NULL, 1);
	cvCalcHist(&I, hist, 0, mask);
	cvNormalizeHist(hist, 1.0);

	//calculate new threshold value using otsu method
	t2 = Otsu(hist);

	//create mask for sceond threshold
	cvZero(mask);

	for(i=0; i < I->width * I->height; i++)
	{		
		if(data[i] > t1 && data[i] < t2)
		{
			mask->imageData[i] = 1;
		}
	}

	//create histogram with new mask (for thresholds t1 and t2)
	cvCalcHist(&I, hist, 0, mask);
	cvNormalizeHist(hist, 1.0);

	t3 = Otsu(hist);
	cvThreshold(I, thresh, t3, 255, CV_THRESH_BINARY);
	
	cvReleaseImage(&mask);
	cvReleaseHist(&hist);

}

