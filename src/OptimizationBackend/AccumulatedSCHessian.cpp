/**
* This file is part of DSO.
* 
* Copyright 2016 Technical University of Munich and Intel.
* Developed by Jakob Engel <engelj at in dot tum dot de>,
* for more information see <http://vision.in.tum.de/dso>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* DSO is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* DSO is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with DSO. If not, see <http://www.gnu.org/licenses/>.
*/


#include "OptimizationBackend/AccumulatedSCHessian.h"
#include "OptimizationBackend/EnergyFunctional.h"
#include "OptimizationBackend/EnergyFunctionalStructs.h"

#include "FullSystem/HessianBlocks.h"

namespace dso
{

void AccumulatedSCHessianSSE::addPoint(EFPoint* p, bool shiftPriorToZero, int tid)
{
	int ngoodres = 0;
	for(EFResidual* r : p->residualsAll) if(r->isActive()) ngoodres++;
	if(ngoodres==0)
	{
		p->HdiF=0;
		p->bdSumF=0;
		p->data->idepth_hessian=0;
		p->data->maxRelBaseline=0;
		return;
	}

	float H = p->Hdd_accAF+p->Hdd_accLF+p->priorF;
	if(H < 1e-10) H = 1e-10;

	p->data->idepth_hessian=H;

	p->HdiF = 1.0 / H;
	p->bdSumF = p->bd_accAF + p->bd_accLF;
	if(shiftPriorToZero) p->bdSumF += p->priorF*p->deltaF;
	VecCf Hcd = p->Hcd_accAF + p->Hcd_accLF;
	accHcc[tid].update(Hcd,Hcd,p->HdiF);
	accbc[tid].update(Hcd, p->bdSumF * p->HdiF);

	assert(std::isfinite((float)(p->HdiF)));

	int nFrames2 = nframes[tid]*nframes[tid];
	for(EFResidual* r1 : p->residualsAll)
	{
		if(!r1->isActive()) continue;
		int r1ht = r1->hostIDX + r1->targetIDX*nframes[tid];

		for(EFResidual* r2 : p->residualsAll)
		{
			if(!r2->isActive()) continue;

			accD[tid][r1ht+r2->targetIDX*nFrames2].update(r1->JpJdF, r2->JpJdF, p->HdiF);
		}

		accE[tid][r1ht].update(r1->JpJdF, Hcd, p->HdiF);
		accEB[tid][r1ht].update(r1->JpJdF,p->HdiF*p->bdSumF);
	}
}
void AccumulatedSCHessianSSE::stitchDoubleInternal(
		MatXX* H, VecX* b, EnergyFunctional const * const EF,
		int min, int max, Vec10* stats, int tid)
{
	int toAggregate = NUM_THREADS;
	if(tid == -1) { toAggregate = 1; tid = 0; }	// special case: if we dont do multithreading, dont aggregate.
	if(min==max) return;


	int nf = nframes[0];
	int nframes2 = nf*nf;

	for(int k=min;k<max;k++)
	{
		int i = k%nf;
		int j = k/nf;

		int iIdx = CPARS+i*8;
		int jIdx = CPARS+j*8;
		int ijIdx = i+nf*j;

		Mat8C Hpc = Mat8C::Zero();
		Vec8 bp = Vec8::Zero();

		for(int tid2=0;tid2 < toAggregate;tid2++)
		{
			accE[tid2][ijIdx].finish();
			accEB[tid2][ijIdx].finish();
			Hpc += accE[tid2][ijIdx].A1m.cast<double>();
			bp += accEB[tid2][ijIdx].A1m.cast<double>();
		}

		H[tid].block<8,CPARS>(iIdx,0) += EF->adHost[ijIdx] * Hpc;
		H[tid].block<8,CPARS>(jIdx,0) += EF->adTarget[ijIdx] * Hpc;
		b[tid].segment<8>(iIdx) += EF->adHost[ijIdx] * bp;
		b[tid].segment<8>(jIdx) += EF->adTarget[ijIdx] * bp;



		for(int k=0;k<nf;k++)
		{
			int kIdx = CPARS+k*8;
			int ijkIdx = ijIdx + k*nframes2;
			int ikIdx = i+nf*k;

			Mat88 accDM = Mat88::Zero();

			for(int tid2=0;tid2 < toAggregate;tid2++)
			{
				accD[tid2][ijkIdx].finish();
				if(accD[tid2][ijkIdx].num == 0) continue;
				accDM += accD[tid2][ijkIdx].A1m.cast<double>();
			}

			H[tid].block<8,8>(iIdx, iIdx) += EF->adHost[ijIdx] * accDM * EF->adHost[ikIdx].transpose();
			H[tid].block<8,8>(jIdx, kIdx) += EF->adTarget[ijIdx] * accDM * EF->adTarget[ikIdx].transpose();
			H[tid].block<8,8>(jIdx, iIdx) += EF->adTarget[ijIdx] * accDM * EF->adHost[ikIdx].transpose();
			H[tid].block<8,8>(iIdx, kIdx) += EF->adHost[ijIdx] * accDM * EF->adTarget[ikIdx].transpose();
		}
	}

	if(min==0)
	{
		for(int tid2=0;tid2 < toAggregate;tid2++)
		{
			accHcc[tid2].finish();
			accbc[tid2].finish();
			H[tid].topLeftCorner<CPARS,CPARS>() += accHcc[tid2].A1m.cast<double>();
			b[tid].head<CPARS>() += accbc[tid2].A1m.cast<double>();
		}
	}


//	// ----- new: copy transposed parts for calibration only.
//	for(int h=0;h<nf;h++)
//	{
//		int hIdx = 4+h*8;
//		H.block<4,8>(0,hIdx).noalias() = H.block<8,4>(hIdx,0).transpose();
//	}
}

void AccumulatedSCHessianSSE::stitchDouble(MatXX &H, VecX &b, EnergyFunctional const * const EF, int tid)
{

	int nf = nframes[0];
	int nframes2 = nf*nf;

	H = MatXX::Zero(nf*8+CPARS, nf*8+CPARS);
	b = VecX::Zero(nf*8+CPARS);


	for(int i=0;i<nf;i++)
		for(int j=0;j<nf;j++)
		{
			int iIdx = CPARS+i*8;
			int jIdx = CPARS+j*8;
			int ijIdx = i+nf*j;

			accE[tid][ijIdx].finish();
			accEB[tid][ijIdx].finish();

			Mat8C accEM = accE[tid][ijIdx].A1m.cast<double>();
			Vec8 accEBV = accEB[tid][ijIdx].A1m.cast<double>();

			H.block<8,CPARS>(iIdx,0) += EF->adHost[ijIdx] * accEM;
			H.block<8,CPARS>(jIdx,0) += EF->adTarget[ijIdx] * accEM;

			b.segment<8>(iIdx) += EF->adHost[ijIdx] * accEBV;
			b.segment<8>(jIdx) += EF->adTarget[ijIdx] * accEBV;

			for(int k=0;k<nf;k++)
			{
				int kIdx = CPARS+k*8;
				int ijkIdx = ijIdx + k*nframes2;
				int ikIdx = i+nf*k;

				accD[tid][ijkIdx].finish();
				if(accD[tid][ijkIdx].num == 0) continue;
				Mat88 accDM = accD[tid][ijkIdx].A1m.cast<double>();

				H.block<8,8>(iIdx, iIdx) += EF->adHost[ijIdx] * accDM * EF->adHost[ikIdx].transpose();

				H.block<8,8>(jIdx, kIdx) += EF->adTarget[ijIdx] * accDM * EF->adTarget[ikIdx].transpose();

				H.block<8,8>(jIdx, iIdx) += EF->adTarget[ijIdx] * accDM * EF->adHost[ikIdx].transpose();

				H.block<8,8>(iIdx, kIdx) += EF->adHost[ijIdx] * accDM * EF->adTarget[ikIdx].transpose();
			}
		}

	accHcc[tid].finish();
	accbc[tid].finish();
	H.topLeftCorner<CPARS,CPARS>() = accHcc[tid].A1m.cast<double>();
	b.head<CPARS>() = accbc[tid].A1m.cast<double>();

	// ----- new: copy transposed parts for calibration only.
	for(int h=0;h<nf;h++)
	{
		int hIdx = CPARS+h*8;
		H.block<CPARS,8>(0,hIdx).noalias() = H.block<8,CPARS>(hIdx,0).transpose();
	}
}

}
