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
#include "OptimizationBackend/MatrixAccumulators.h"
#include "IOWrapper/Output3DWrapper.h"
#include "util/ImageAndExposure.h"
#include "util/settings.h"
#include "vector"
#include "CoarseInitializer.h"
#include <math.h>

// INclude opencv core for cv::Mat in FrameHessian
#include "opencv2/core.hpp"


namespace dso
{
struct CalibHessian;
struct FrameHessian;

class CoarseInitializerStereo {
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    CoarseInitializerStereo(int w, int h);
    ~CoarseInitializerStereo();

    bool initializeFromStereo(CalibHessian* HCalib, ImageAndExposure* image, std::vector<IOWrap::Output3DWrapper*> &wraps, int id, int idmarg);
    void setFirst(CalibHessian* HCalib, FrameHessian* rightFrame, cv::Mat dispR, double baseline);
    bool trackFrame(FrameHessian* frameL, std::vector<IOWrap::Output3DWrapper*> &wraps);
    void calcTGrads(FrameHessian* newFrameHessian);

    int frameID;
    bool fixAffine;
    bool printDebug;

    Pnt* points[PYR_LEVELS];
    int numPoints[PYR_LEVELS];
    AffLight thisToNext_aff;
    SE3 thisToNext;


    FrameHessian* firstFrame;
    FrameHessian* newFrame;
private:
    // Camera calibration info
    Mat33 K[PYR_LEVELS];
    Mat33 Ki[PYR_LEVELS];
    double fx[PYR_LEVELS];
    double fy[PYR_LEVELS];
    double fxi[PYR_LEVELS];
    double fyi[PYR_LEVELS];
    double cx[PYR_LEVELS];
    double cy[PYR_LEVELS];
    double cxi[PYR_LEVELS];
    double cyi[PYR_LEVELS];
    int w[PYR_LEVELS];
    int h[PYR_LEVELS];
    void makeK(CalibHessian* HCalib);

    bool snapped;
    int snappedAt;

    // pyramid images & levels on all levels
    Eigen::Vector3f* dINew[PYR_LEVELS];
    Eigen::Vector3f* dIFist[PYR_LEVELS];

    Eigen::DiagonalMatrix<float, 8> wM;

    // temporary buffers for H and b.
    Vec10f* JbBuffer;           // 0-7: sum(dd * dp). 8: sum(res*dd). 9: 1/(1+sum(dd*dd))=inverse hessian entry.
    Vec10f* JbBuffer_new;

    Accumulator9 acc9;
    Accumulator9 acc9SC;


    Vec3f dGrads[PYR_LEVELS];

    float alphaK;
    float alphaW;
    float regWeight;
    float couplingWeight;

    Vec3f calcResAndGS(
            int lvl,
            Mat88f &H_out, Vec8f &b_out,
            Mat88f &H_out_sc, Vec8f &b_out_sc,
            const SE3 &refToNew, AffLight refToNew_aff,
            bool plot);
    Vec3f calcEC(int lvl); // returns OLD NERGY, NEW ENERGY, NUM TERMS.
    void optReg(int lvl);

    void propagateUp(int srcLvl);
    void propagateDown(int srcLvl);
    float rescale();

    void resetPoints(int lvl);
    void doStep(int lvl, float lambda, Vec8f inc);
    void applyStep(int lvl);

    void makeGradients(Eigen::Vector3f** data);

    void debugPlot(int lvl, std::vector<IOWrap::Output3DWrapper*> &wraps);
    void makeNN();
};

}


