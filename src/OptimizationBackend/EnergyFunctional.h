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


#pragma once

 
#include "util/NumType.h"
#include "util/IndexThreadReduce.h"
#include "vector"
#include <math.h>
#include "map"


namespace dso
{

class PointFrameResidual;
class CalibHessian;
class FrameHessian;
class PointHessian;


class EFResidual;
class EFPoint;
class EFFrame;
class EnergyFunctional;
class AccumulatedTopHessian;
class AccumulatedTopHessianSSE;
class AccumulatedSCHessian;
class AccumulatedSCHessianSSE;


extern bool EFAdjointsValid;
extern bool EFIndicesValid;
extern bool EFDeltaValid;



class EnergyFunctional {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	friend class EFFrame;
	friend class EFPoint;
	friend class EFResidual;
	friend class AccumulatedTopHessian;
	friend class AccumulatedTopHessianSSE;
	friend class AccumulatedSCHessian;
	friend class AccumulatedSCHessianSSE;

	EnergyFunctional();
	~EnergyFunctional();


	EFResidual* insertResidual(PointFrameResidual* r);
	EFFrame* insertFrame(FrameHessian* fh, CalibHessian* Hcalib);
	EFPoint* insertPoint(PointHessian* ph);

	void dropResidual(EFResidual* r);
	void marginalizeFrame(EFFrame* fh);
	void removePoint(EFPoint* ph);



	void marginalizePointsF();
	void dropPointsF();
	void solveSystemF(int iteration, double lambda, CalibHessian* HCalib);
	double calcMEnergyF();
	double calcLEnergyF_MT();


	void makeIDX();

	void setDeltaF(CalibHessian* HCalib);

	void setAdjointsF(CalibHessian* Hcalib);

	std::vector<EFFrame*> frames;
	int nPoints, nFrames, nResiduals;

	MatXX HM;
	VecX bM;

	int resInA, resInL, resInM;
	MatXX lastHS;
	VecX lastbS;
	VecX lastX;
	std::vector<VecX> lastNullspaces_forLogging;
	std::vector<VecX> lastNullspaces_pose;
	std::vector<VecX> lastNullspaces_scale;
	std::vector<VecX> lastNullspaces_affA;
	std::vector<VecX> lastNullspaces_affB;

	IndexThreadReduce<Vec10>* red;


	std::map<uint64_t,
	  Eigen::Vector2i,
	  std::less<uint64_t>,
	  Eigen::aligned_allocator<std::pair<const uint64_t, Eigen::Vector2i>>
	  > connectivityMap;

private:

	VecX getStitchedDeltaF() const;

	void resubstituteF_MT(VecX x, CalibHessian* HCalib, bool MT);
    void resubstituteFPt(const VecCf &xc, Mat18f* xAd, int min, int max, Vec10* stats, int tid);

	void accumulateAF_MT(MatXX &H, VecX &b, bool MT);
	void accumulateLF_MT(MatXX &H, VecX &b, bool MT);
	void accumulateSCF_MT(MatXX &H, VecX &b, bool MT);

	void calcLEnergyPt(int min, int max, Vec10* stats, int tid);

	void orthogonalize(VecX* b, MatXX* H);
	Mat18f* adHTdeltaF;

	Mat88* adHost;
	Mat88* adTarget;

	Mat88f* adHostF;
	Mat88f* adTargetF;


	VecC cPrior;
	VecCf cDeltaF;
	VecCf cPriorF;

	AccumulatedTopHessianSSE* accSSE_top_L;
	AccumulatedTopHessianSSE* accSSE_top_A;


	AccumulatedSCHessianSSE* accSSE_bot;

	std::vector<EFPoint*> allPoints;
	std::vector<EFPoint*> allPointsToMarg;

	float currentLambda;
};
}

