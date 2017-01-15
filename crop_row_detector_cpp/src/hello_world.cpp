#include "RVLCropRowDetector.h"
#include "time.h"
//#include "timer.h"

//#define RVLCRD_DEBUG

CRVLCropRowDetector::CRVLCropRowDetector(void)
{
	m_mind = 8;
	m_ndOctaves = 3;
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

	number_of_periods = m_ndOctaves * m_ndSamplesPerOctave + 1;

	m_dstep = pow(2.0, 1.0 / (double)m_ndSamplesPerOctave);

	number_of_phases = (int)floor((double)m_mind * pow(m_dstep, number_of_periods)) + 1;

	m_DPData = new RVLCRD_DPDATA[m_h * number_of_periods * number_of_phases];

	if(m_bestScore)
		delete[] m_bestScore;

	m_bestScore = new double[h];
}

void CRVLCropRowDetector::Apply(unsigned char * I, int w, int imNum) {
	double fa0 = (double)m_a0;
	double fb0 = (double)m_b0;
	double fd0 = (double)m_d0;

	double halfw = (double)(w / 2);

	int sizeof_row_data = number_of_phases * number_of_periods;

	// assign score to each pair (phase,d) for each image row

	int *II_ = new int[w + 1];

	int *II = II_ + 1;

	II[-1] = 0;

	unsigned char *I_ = I;	

	int *crange_ = new int[number_of_periods];

	RVLCRD_DPDATA *pointer_to_row_data = m_DPData;

	int u, row_number;
	int index_of_period;
	int phase_range;
	int kStart, kEnd;
	int a, b, phase;
	double d;
	double halfa, halfb, halfd;
	double fu, fub;
	double scale;
	int u0, u1, u2, u3;
	double Sa, Sb;
	int na, nb;
	int rightmost_parabola;
	double score;
	int bestid, bestc, bestd;
	double fTmp;
	double bestScore;
	RVLCRD_DPDATA *pointer_to_period_data, *pointer_to_tuple_data, *pDP_;
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

	for(row_number = 0; row_number < m_h; row_number++, I_ += w, pointer_to_row_data += sizeof_row_data)
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

		pointer_to_period_data = pointer_to_row_data;

		for(index_of_period = 0; index_of_period < number_of_periods; index_of_period++, d *= m_dstep, pointer_to_period_data += number_of_phases)
		{
#ifdef RVLCRD_DEBUG
			if(d > 10.0 * debug)
				debug++;
#endif
			phase_range = (int)floor(0.5 * d);

			crange_[index_of_period] = phase_range;

			scale = d / fd0;
			fTmp = floor(scale * fa0 + 0.5);
			a = (int)fTmp;
			halfa = 0.5 * fTmp;
			fTmp = floor(scale * fb0 + 0.5);
			b = (int)fTmp;
			halfb = 0.5 * fTmp;
			halfd = 0.5 * d;
			fub =  halfd - halfb;

			pointer_to_tuple_data = pointer_to_period_data + number_of_phases / 2 - phase_range;

			//FOR score IMAGE (d x phase)
			//if(v == 30)
			//{
			//	fprintf(fp, "\sizeof_row_data");
			//	fprintf(fp2, "%d\sizeof_row_data", phase_range);
			//}
			//END IMAGE

			for(phase = -phase_range; phase < phase_range; phase++, pointer_to_tuple_data++)
			{
				kStart = (int)floor((-(double)(w / 2 + phase) + halfa) / d);
				kEnd = (int)floor(((double)(w / 2 - phase) + halfa) / d);

				fu = (double)phase + (double)kStart * d + halfw;

				u0 = DOUBLE2INT(fu - halfa);

				u1 = u0 + a - 1;

				if(u1 >= 0) {
					Sa = II[u1];
					na = u1;

					//debugCounter++;
				} else {
					Sa = 0;
					na = 0;
				}

				u2 = DOUBLE2INT(fu + fub);

				u3 = u2 + b - 1;

				if(u2 < 0)
					u2 = 0;				

				if(u3 >= 0)
				{
					Sb = (II[u3] - II[u2 - 1]);
					nb = (u3 - u2 + 1);

					//debugCounter++;
				}
				else
				{
					Sb = 0;
					nb = 0;
				}

				fu += d;

				for(rightmost_parabola = kStart + 1; rightmost_parabola < kEnd; rightmost_parabola++, fu += d)
				{
					u0 = DOUBLE2INT(fu - halfa);

					u1 = u0 + a - 1;

					Sa += (II[u1] - II[u0 - 1]);
					na += (u1 - u0 + 1);

					u2 = DOUBLE2INT(fu + fub);

					u3 = u2 + b - 1;

					Sb += (II[u3] - II[u2 - 1]);
					nb += (u3 - u2 + 1);

					//debugCounter += 2;
				}

				u0 = DOUBLE2INT(fu - halfa);

				u1 = u0 + a - 1;

				if(u1 >= w)
					u1 = w - 1;

				Sa += (II[u1] - II[u0 - 1]);
				na += (u1 - u0 + 1);

				//debugCounter++;

				u2 = DOUBLE2INT(fu + fub);

				if(u2 < w)
				{
					u3 = u2 + b - 1;

					if(u3 >= w)
						u3 = w - 1;

					Sb += (II[u3] - II[u2 - 1]);
					nb += (u3 - u2 + 1);

					//debugCounter++;
				}

				//score = (double)(nb * Sa - na * Sb) / (double)(na + nb);
				score = (double)(nb * Sa - na * Sb) / (double)(na * nb);		// new score computation

				pointer_to_tuple_data->D = score;

				//FOR score IMAGE (d x phase)
				//if(v == 30)
				//{
				//	fprintf(fp, "%lf\t", score);
				//}

				//END IMAGE

				//copy D value for phase values in range <-m_nc/2, m_nc/2>
				crange2 = 2*phase_range;
				pDP_ = pointer_to_tuple_data + crange2; //pDP_ pointer for copying D value in phase range <-m_nc/2, m_nc/2>
				c_position = number_of_phases*0.5 + phase + crange2;

				while(c_position <= number_of_phases)
				{
					pDP_->D = score;

					pDP_ += crange2;
					c_position += crange2;
				}

				pDP_ = pointer_to_tuple_data - crange2;
				c_position = number_of_phases*0.5 + phase - crange2;

				while(c_position >= 0)
				{
					pDP_->D = score;

					pDP_ -= crange2;
					c_position -= crange2;
				}

				if(score > bestScore)
				{
					bestScore = score;
					bestc = phase;
					bestid = index_of_period;
					bestd = d;
				}
			}	// for(phase = -phase_range; phase < phase_range; phase++)
		}	// for(index_of_period = 0; index_of_period < m_nd; index_of_period++, d *= m_dstep)

		m_c[row_number] = bestc;
		m_id[row_number] = bestid;
		m_d[row_number] = bestd;
		m_idFilter[row_number] = bestid;
		m_bestScore[row_number] = bestScore;
	}	// for(v = 0; v < h; v++, I_ += w)

	//fclose(fp);
	//fclose(fp2);

	//end Template Matching timer
	end = clock();
	m_templateMatchingTime = ((double)end - (double)start) / (double)CLOCKS_PER_SEC;

	//timer.End();
	//m_templateMatchingTime = timer.GetEllapsedMS();

	delete[] II_;

	// dynamic programing

	// m_DPData is a 3D data structure. It has h blocks,
	// each block having m_nd rows and m_nc columns.
	// Each block corresponds to an image row.

	pointer_to_row_data = m_DPData;	// pointer_to_row_data - ptr. to the first data in a block

#ifdef RVLCRD_L1
	double BV;
	RVLCRD_DPDATA *pDPPrev;	
#else
	int iTmp = RVLMAX(number_of_phases, number_of_periods) + 1;

	int *parab_center_period = new int[iTmp];
	//NEW d_development
	double *v__ = new double[iTmp];
	double *intersection_points = new double[iTmp];

	//int64 debugCounter1 = 0;
	//int64 debugCounter2 = 0;
	//int64 debugCounter3 = 0;
	//int64 debugCounter4 = 0;

	double maxz;
	double s;
#endif

	double best_score_row_v;

	//start Optimization timer
	start = clock();
	//timer.Reset();
	//timer.Start();

	for(row_number = 0; row_number < m_h; row_number++, pointer_to_row_data += sizeof_row_data)
	{
#ifdef RVLCRD_DEBUG
		if(v == 236)
			int debug = 0;
#endif

		best_score_row_v = m_bestScore[row_number];

		pointer_to_period_data = pointer_to_row_data;		// pointer_to_period_data - ptr. to the first data in a block row

		for(index_of_period = 0; index_of_period < number_of_periods; index_of_period++, pointer_to_period_data += number_of_phases)
		{
            //OLD phase_range
			phase_range = crange_[index_of_period];

			//pointer_to_tuple_data = pointer_to_period_data + m_nc / 2 - phase_range;
			pointer_to_tuple_data = pointer_to_period_data;

			//for(phase = -phase_range; phase < phase_range; phase++, pointer_to_tuple_data++)
			for(phase = -number_of_phases/2; phase < number_of_phases/2; phase++, pointer_to_tuple_data++)
			{
				if(best_score_row_v >= 1.0) // f_low
				{
					pointer_to_tuple_data->B = 1.0 - pointer_to_tuple_data->D / best_score_row_v;

					if(pointer_to_tuple_data->B > m_maxD)
						pointer_to_tuple_data->B = m_maxD;
				}
				else
					pointer_to_tuple_data->B = m_maxD;
				
				if(row_number > 0)
					pointer_to_tuple_data->B += (pointer_to_tuple_data - sizeof_row_data)->minBV;
			}
		}

		if(row_number < m_h - 1)
		{
			pointer_to_period_data = pointer_to_row_data;

#ifdef RVLCRD_L1
                        std::cout << "sono qui" << std::endl;
			for(index_of_period = 0; index_of_period < m_nd; index_of_period++, pointer_to_period_data += m_nc)
			{
				phase_range = crange_[index_of_period];

				pointer_to_tuple_data = pointer_to_period_data + m_nc / 2 - phase_range;

				pointer_to_tuple_data->minBV = pointer_to_tuple_data->B;
				pointer_to_tuple_data->phase = -phase_range;
				pointer_to_tuple_data->index_of_period = index_of_period;

				pDPPrev = pointer_to_tuple_data;

				pointer_to_tuple_data++;

				for(phase = -phase_range + 1; phase < phase_range; phase++, pointer_to_tuple_data++)
				{
					pointer_to_tuple_data->minBV = pointer_to_tuple_data->B;

					BV = pDPPrev->minBV + m_lambdac;

					if(pointer_to_tuple_data->minBV <= BV)
					{
						pointer_to_tuple_data->phase = phase;
						pointer_to_tuple_data->index_of_period = index_of_period;
					}
					else
					{
						pointer_to_tuple_data->minBV = BV;
						pointer_to_tuple_data->phase = pDPPrev->phase;
						pointer_to_tuple_data->index_of_period = pDPPrev->index_of_period;
					}

					pDPPrev = pointer_to_tuple_data;
				}

				for(; phase < m_nc; phase++, pointer_to_tuple_data++)
				{
					pointer_to_tuple_data->minBV = pDPPrev->minBV + m_lambdac;
					pointer_to_tuple_data->phase = pDPPrev->phase;
					pointer_to_tuple_data->index_of_period = pDPPrev->index_of_period;

					pDPPrev = pointer_to_tuple_data;
				}

				pDPPrev = pointer_to_period_data + m_nc / 2 + phase_range - 1;

				pointer_to_tuple_data = pDPPrev - 1;

				for(phase = phase_range - 2; phase >= -phase_range; phase--, pointer_to_tuple_data--)
				{
					BV = pDPPrev->minBV + m_lambdac;

					if(pointer_to_tuple_data->minBV > BV)
					{
						pointer_to_tuple_data->minBV = BV;
						pointer_to_tuple_data->phase = pDPPrev->phase;
					}

					pDPPrev = pointer_to_tuple_data;
				}

				for(; phase >= -m_nc/2; phase--, pointer_to_tuple_data--)
				{
					pointer_to_tuple_data->minBV = pDPPrev->minBV + m_lambdac;
					pointer_to_tuple_data->phase = pDPPrev->phase;
					pointer_to_tuple_data->index_of_period = pDPPrev->index_of_period;

					pDPPrev = pointer_to_tuple_data;
				}
			}	// for(index_of_period = 0; index_of_period < m_nd; index_of_period++)

			pointer_to_period_data = pointer_to_row_data;

			for(phase = 0; phase < m_nc; phase++, pointer_to_period_data++)
			{
				pDPPrev = pointer_to_period_data;

				pointer_to_tuple_data = pDPPrev + m_nc;

				for(index_of_period = 1; index_of_period < m_nd; index_of_period++, pointer_to_tuple_data += m_nc)
				{
					BV = pDPPrev->minBV + m_lambdad;

					if(pointer_to_tuple_data->minBV > BV)
					{
						pointer_to_tuple_data->minBV = BV;
						pointer_to_tuple_data->phase = pDPPrev->phase;
						pointer_to_tuple_data->index_of_period = pDPPrev->index_of_period;
					}

					pDPPrev = pointer_to_tuple_data;
				}

				pDPPrev = pointer_to_period_data + (m_nd - 1) * m_nc;

				pointer_to_tuple_data = pDPPrev - m_nc;

				for(index_of_period = m_nd - 2; index_of_period >= 0; index_of_period--, pointer_to_tuple_data -= m_nc)
				{
					BV = pDPPrev->minBV + m_lambdad;

					if(pointer_to_tuple_data->minBV > BV)
					{
						pointer_to_tuple_data->minBV = BV;
						pointer_to_tuple_data->phase = pDPPrev->phase;
						pointer_to_tuple_data->index_of_period = pDPPrev->index_of_period;
					}

					pDPPrev = pointer_to_tuple_data;
				}
			}	// for(phase = 0; phase < m_nc; phase++, pointer_to_row_data++)
#else
			maxz = (1.0 + m_lambdac * (double)(number_of_phases * number_of_phases)) / (2.0 * m_lambdac);

			for(index_of_period = 0; index_of_period < number_of_periods; index_of_period++, pointer_to_period_data += number_of_phases)
			{
#ifdef RVLCRD_DEBUG
				if(index_of_period == 186)
					int debug = 0;
#endif
				phase_range = crange_[index_of_period];

				// felzenszwalb_TC12	RVL
				// ============================================
				// rightmost_parabola					rightmost_parabola
				// v					parab_center_period
				// intersection_points					intersection_points
				// q					phase
				// s					s
				// f(q)					pointer_to_tuple_data->B
				// f(v(rightmost_parabola))				pointer_to_period_data[parab_center_period[rightmost_parabola]+m_nc/2].B

				rightmost_parabola = 0;
				//OLD phase_range
				//parab_center_period[0] = -phase_range;
				parab_center_period[0] = -number_of_phases*0.5;
				intersection_points[0] = -maxz;
				intersection_points[1] = maxz;

				//OLD phase_range
				//pointer_to_tuple_data = pointer_to_period_data + m_nc / 2 - phase_range + 1;
				pointer_to_tuple_data = pointer_to_period_data + 1;

				//for(phase = -phase_range + 1; phase < phase_range; phase++, pointer_to_tuple_data++)
				for(phase = -number_of_phases/2 + 1; phase < number_of_phases/2; phase++, pointer_to_tuple_data++)
				{
					while(true)
					{
						s = ((pointer_to_tuple_data->B + m_lambdac * (double)(phase * phase)) -
								(pointer_to_period_data[parab_center_period[rightmost_parabola] + number_of_phases / 2].B + m_lambdac * (double)(parab_center_period[rightmost_parabola] * parab_center_period[rightmost_parabola]))) /
							(2.0 * m_lambdac * (double)(phase - parab_center_period[rightmost_parabola]));

						//debugCounter1++;

						if(s <= intersection_points[rightmost_parabola])
							rightmost_parabola--;
						else
							break;
					}

					rightmost_parabola++;

					parab_center_period[rightmost_parabola] = phase;
					intersection_points[rightmost_parabola] = s;
					intersection_points[rightmost_parabola + 1] = maxz;
				}

				rightmost_parabola = 0;

				pointer_to_tuple_data = pointer_to_period_data;

				for(phase = -number_of_phases / 2; phase < number_of_phases / 2; phase++, pointer_to_tuple_data++)
				{
					while(intersection_points[rightmost_parabola + 1] < (double)phase)
					{
						//debugCounter2++;

						rightmost_parabola++;
					}

					iTmp = (phase - parab_center_period[rightmost_parabola]);
					pointer_to_tuple_data->minBV = pointer_to_period_data[parab_center_period[rightmost_parabola] + number_of_phases / 2].B + m_lambdac * (double)(iTmp * iTmp);
					pointer_to_tuple_data->c = parab_center_period[rightmost_parabola];
					pointer_to_tuple_data->id = index_of_period;
				}
			}	// for(index_of_period = 0; index_of_period < m_nd; index_of_period++)

			maxz = (1.0 + m_lambdad * (double)(number_of_periods * number_of_periods)) / (2.0 * m_lambdad);

			pointer_to_period_data = pointer_to_row_data;

			for(phase = 0; phase < number_of_phases; phase++, pointer_to_period_data++)
			{
#ifdef RVLCRD_DEBUG
				if(phase == m_nc / 2)
					int debug = 0;
#endif
				rightmost_parabola = 0;
				parab_center_period[0] = 0; //for saving index_of_period
				v__[0] = (double)m_mind; //for saving d
				intersection_points[0] = -maxz;
				intersection_points[1] = maxz;

				pointer_to_tuple_data = pointer_to_period_data + number_of_phases;
				
				//NEW d_development
				//d = (double)m_mind*m_dstep;
				d = (double)m_mind;

				for(index_of_period = 1; index_of_period < number_of_periods; index_of_period++, pointer_to_tuple_data += number_of_phases)
				{
					//NEW d_development
					d *= m_dstep;

					while(true)
					{
						//OLD d
						//s = ((pointer_to_tuple_data->minBV + m_lambdad * (double)(index_of_period * index_of_period)) - (pointer_to_period_data[m_nc * parab_center_period[rightmost_parabola]].minBV + m_lambdad * (double)(parab_center_period[rightmost_parabola] * parab_center_period[rightmost_parabola]))) /
						//	(2.0 * m_lambdad * (double)(index_of_period - parab_center_period[rightmost_parabola]));

						s = ((pointer_to_tuple_data->minBV + m_lambdad * (double)(d * d)) - (pointer_to_period_data[number_of_phases * parab_center_period[rightmost_parabola]].minBV + m_lambdad * (double)(v__[rightmost_parabola] * v__[rightmost_parabola]))) /
							(2.0 * m_lambdad * (double)(d - v__[rightmost_parabola]));

						//debugCounter3++;

						if(s <= intersection_points[rightmost_parabola])
							rightmost_parabola--;
						else
							break;
					}

					rightmost_parabola++;

					//NEW d_development
					parab_center_period[rightmost_parabola] = index_of_period;
					v__[rightmost_parabola] = d;

					intersection_points[rightmost_parabola] = s;
					intersection_points[rightmost_parabola + 1] = maxz;
				}

				rightmost_parabola = 0;

				pointer_to_tuple_data = pointer_to_period_data;

				//NEW d_development
				d = (double)m_mind;

				for(index_of_period = 0; index_of_period < number_of_periods; index_of_period++, pointer_to_tuple_data += number_of_phases)
				{
					//NEW d_development
					//while(intersection_points[rightmost_parabola + 1] < (double)index_of_period)
					while(intersection_points[rightmost_parabola + 1] < d)
					{
						//debugCounter4++;

						rightmost_parabola++;
					}

					//iTmp = (index_of_period - parab_center_period[rightmost_parabola]);
					iTmp = (d - v__[rightmost_parabola]);

					pointer_to_tuple_data->minBV = pointer_to_period_data[number_of_phases * parab_center_period[rightmost_parabola]].minBV + (m_lambdad * (double)(iTmp * iTmp));
					pointer_to_tuple_data->c = pointer_to_period_data[number_of_phases * parab_center_period[rightmost_parabola]].c;
					pointer_to_tuple_data->id = parab_center_period[rightmost_parabola];

					//NEW d_development
					d *= m_dstep;
				}
			}	//	for(phase = 0; phase < m_nc; phase++)
#endif
		}	// if(v < m_h - 1)

#ifdef RVLCRD_DEBUG
		pointer_to_tuple_data = pointer_to_row_data;

		for(index_of_period = 0; index_of_period < m_nd; index_of_period++)
			for(phase = 0; phase < m_nc; phase++, pointer_to_tuple_data++)
				if(pointer_to_tuple_data->minBV < 0)
					int debug = 0;
#endif
	}	// for(v = 0; v < m_h - 1; v++, pointer_to_row_data += sizeof_row_data)

#ifndef RVLCRD_L1
	delete[] parab_center_period;
	delete[] intersection_points;
#endif

	pointer_to_row_data = m_DPData + (m_h - 1) * sizeof_row_data;

	pointer_to_period_data = pointer_to_row_data;

	row_number = m_h - 1;

	//OLD phase_range
	//RVLCRD_DPDATA *pBestNode = pointer_to_period_data + m_nc / 2 - crange_[0];
	RVLCRD_DPDATA *pBestNode = pointer_to_period_data;

	//for saving best d
	d = (double)m_mind;

	for(index_of_period = 0; index_of_period < number_of_periods; index_of_period++, pointer_to_period_data += number_of_phases, d *= m_dstep)
	{
		phase_range = crange_[index_of_period];

		//OLD phase_range
		//pointer_to_tuple_data = pointer_to_period_data + m_nc / 2 - phase_range;
		pointer_to_tuple_data = pointer_to_period_data;

		//for(phase = -phase_range; phase < phase_range; phase++, pointer_to_tuple_data++)
		for(phase = -number_of_phases/2; phase < number_of_phases/2; phase++, pointer_to_tuple_data++)
			if(pointer_to_tuple_data->B < pBestNode->B)
			{
				pBestNode = pointer_to_tuple_data;
				bestc = phase;
				bestid = index_of_period;
				bestd = d;
			}
	}

	m_c[row_number] = bestc;
	m_id[row_number] = bestid;
	m_d[row_number] = bestd;

#ifdef RVLCRD_DEBUG
	char name[40];
	sprintf(name, "Image_ExG_%d.txt", imNum);
	FILE *f = fopen(name, "w");
	//fprintf(f, "v\tpDP->D\t\tpDP->B\t\tpDP->minBV\tpDP->phase\tpDP->index_of_period\sizeof_row_data");
	//fprintf(f, "---------------------------------------------------------------------------------\sizeof_row_data");
#endif

	m_visibleStart = 0;
	m_visibleStop = 0;

	for(row_number = m_h - 2; row_number >= 0; row_number--)
	{
		pointer_to_tuple_data = pBestNode - sizeof_row_data;

		pointer_to_row_data -= sizeof_row_data;

		pBestNode = pointer_to_row_data + pointer_to_tuple_data->id * number_of_phases + number_of_phases / 2 + pointer_to_tuple_data->c;

		m_c[row_number] = pointer_to_tuple_data->c;
		m_id[row_number] = pointer_to_tuple_data->id;
		m_d[row_number] = (double)m_mind * pow(m_dstep, (double)(m_id[row_number]));

		/*if(abs(m_idFilter[v] - m_id[v]) < 3)
		//if(pointer_to_tuple_data->D == m_bestScore[v])
		{
			//m_visibleStart = v;
			if(m_visibleStop == 0)
			{
				m_visibleStop = v;
				m_visibleStart = v;
			}
			else
			{
				m_visibleStart = v;
			}
		}*/

#ifdef RVLCRD_DEBUG
		fprintf(f, "%d\t%f\t%f\t%f\t%d\t%d\t%f\sizeof_row_data", v,pointer_to_tuple_data->D,pointer_to_tuple_data->B,pointer_to_tuple_data->minBV,pointer_to_tuple_data->phase,pointer_to_tuple_data->index_of_period,m_bestScore[v]);
#endif
	}

#ifdef RVLCRD_DEBUG
	fclose(f);
#endif

	//end Optimization timer
	end = clock();
	m_optimizationTime = ((double)end - (double)start) / (double)CLOCKS_PER_SEC;

	//timer.End();
	//m_optimizationTime = timer.GetEllapsedMS();

	//m_totalTime = m_vegetationImageTime + m_templateMatchingTime + m_optimizationTime;
	m_totalTime = ((double)end - (double)startTotal) / (double)CLOCKS_PER_SEC;

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
	start = clock();

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

