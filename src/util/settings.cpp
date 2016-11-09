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



#include "util/settings.h"
#include <boost/bind.hpp>


namespace dso
{
int pyrLevelsUsed = PYR_LEVELS;


float setting_keyframesPerSecond = 0;
bool setting_realTimeMaxKF = false;
float setting_maxShiftWeightT= 0.04f * (640+480);
float setting_maxShiftWeightR= 0.0f * (640+480);
float setting_maxShiftWeightRT= 0.02f * (640+480);
float setting_kfGlobalWeight = 1;
float setting_maxAffineWeight= 2;


float setting_idepthFixPrior = 50*50;
float setting_idepthFixPriorMargFac = 600*600; // 30000*30000;
float setting_initialRotPrior = 1e11;
float setting_initialTransPrior = 1e10;
float setting_initialAffBPrior = 1e14;
float setting_initialAffAPrior = 1e14;
float setting_initialCalibHessian = 5e9;





int setting_solverMode = SOLVER_FIX_LAMBDA | SOLVER_ORTHOGONALIZE_X_LATER;
double setting_solverModeDelta = 0.00001;



float setting_minIdepthH_act = 100;
float setting_minIdepthH_marg = 50;
int setting_margPointVisWindow=0;


float setting_maxPixSearch = 0.027;
float setting_desiredImmatureDensity = 1500;
float setting_desiredPointDensity = 2000;
float setting_minPointsRemaining = 0.05;
float setting_maxLogAffFacInWindow = 0.7;

int   setting_minFrames = 5;
int   setting_maxFrames = 7;
int   setting_minFrameAge = 1;
int   setting_maxOptIterations=6;
int   setting_minOptIterations=1;
float setting_thOptIterations=1.2;
float setting_outlierTH = 12*12;					// higher -> less strict
float setting_outlierSmoothnessTH = 0; 				// higher -> more strict
float setting_outlierTHSumComponent = 50*50; 		// higher -> less strong gradient-based reweighting .
int   setting_killOverexposed = 1000;
int   setting_killOverexposedMode = 0;
int setting_pattern = 8;						// 0=.; 1=+; 2=x; 3=full; 4=spread
float setting_margWeightFac = 0.5*0.5;
int setting_discreteSeachItsOnPointActivation = 0;
int setting_GNItsOnPointActivation = 3;


float setting_SmoothnessErrorPixelTH = 1000;
float setting_SmoothnessEMinInlierPercentage = 0.25;
float setting_SmoothnessEGoodInlierPercentage = 0.66;

float setting_minTraceQuality = 3;
int setting_minTraceTestRadius = 2;
float setting_reTrackThreshold1 = 1.5;
float setting_reTrackThreshold2 = 1.5;
float setting_reTrackThreshold3 = 1.5;


int   setting_minGoodActiveResForMarg=3;	// irrelevant
int   setting_minGoodResForMarg=4;			// irrelevant
int   setting_minInlierVotesForMarg=0;		// irrelevant
float setting_minRelBSForMarg=0.00;			// irrelevant




// 0 = nothing.
// 1 = apply inv. response.
// 2 = apply inv. response & remove V.
int setting_photometricCalibration = 2;
bool setting_useExposure = true;
float setting_affineOptModeA = 1e12; //-1: fix. >=0: optimize (with prior, if > 0).
float setting_affineOptModeB = 1e8; //-1: fix. >=0: optimize (with prior, if > 0).
float setting_affineOptModeA_huberTH = 10000;
float setting_affineOptModeB_huberTH = 10000;
int setting_gammaWeightsPixelSelect = 1;



bool setting_relinAlways = true;
bool setting_fixCalib = false;


bool setting_activateAllOnMarg = false;
bool setting_forceAceptStep = true;



float setting_useDepthWeightsCoarse = -1;
bool setting_dilateDoubleCoarse = false;


float setting_huberTH = 9;	  	// done



float setting_frameEnergyTHConstWeight = 0.5;
float setting_frameEnergyTHN = 0.7f;
float setting_frameEnergyTHFacMean = 0.0;
float setting_frameEnergyTHFacMedian = 1.5;
float setting_overallEnergyTHWeight = 1;
float setting_coarseCutoffTH = 20;

float setting_minGradHistCut = 0.5;
float setting_minGradHistAdd = 7;
float setting_fixGradTH = -1;
float setting_gradDownweightPerLevel = 0.75;
bool  setting_selectDirectionDistribution = true;
int setting_pixelSelectionUseFast=0;



float setting_trace_stepsize = 1.0;				// stepsize for initial discrete search.
int setting_trace_GNIterations = 3;				// max # GN iterations
float setting_trace_GNThreshold = 0.1;				// GN stop after this stepsize.
float setting_trace_extraSlackOnTH = 1.2;			// for energy-based outlier check, be slightly more relaxed by this factor.
float setting_trace_slackInterval = 1.5;			// if pixel-interval is smaller than this, leave it be.
float setting_trace_minImprovementFactor = 2;		// if pixel-interval is smaller than this, leave it be.



float benchmarkSetting_fxfyfac = 0;
int benchmarkSetting_width = 0;
int benchmarkSetting_height = 0;
float benchmark_varNoise = 0;
float benchmark_varBlurNoise = 0;
float benchmark_initializerSlackFactor = 1;
int benchmark_noiseGridsize = 3;


float freeDebugParam1 = 1;
float freeDebugParam2 = 1;
float freeDebugParam3 = 1;
float freeDebugParam4 = 1;
float freeDebugParam5 = 1;



bool disableReconfigure=false;
bool debugSaveImages = false;
bool multiThreading = true;
bool disableAllDisplay = false;
bool setting_onlyLogKFPoses = true;
bool setting_logStuff = true;


int benchmarkSpecialOption = 0;
bool goStepByStep = false;


bool setting_render_displayCoarseTrackingFull=false;
bool setting_render_renderWindowFrames=true;
bool setting_render_plotTrackingFull = false;
bool setting_render_display3D = true;
bool setting_render_displayResidual = true;
bool setting_render_displayVideo = true;
bool setting_render_displayDepth = true;

bool setting_fullResetRequested = false;

int sparsityFactor = 5;	// not actually a setting, only some legacy stuff for coarse initializer.


void handleKey(char k)
{
	char kkk = k;
	switch(kkk)
	{
	case 'd': case 'D':
		freeDebugParam5 = ((int)(freeDebugParam5+1))%10;
		printf("new freeDebugParam5: %f!\n", freeDebugParam5);
		break;
	case 's': case 'S':
		freeDebugParam5 = ((int)(freeDebugParam5-1+10))%10;
		printf("new freeDebugParam5: %f!\n", freeDebugParam5);
		break;
	}

}




int staticPattern[10][40][2] = {
		{{0,0}, 	  {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100},	// .
		 {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100},
		 {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100},
		 {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}},

		{{0,-1},	  {-1,0},	   {0,0},	    {1,0},	     {0,1}, 	  {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100},	// +
		 {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100},
		 {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100},
		 {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}},

		{{-1,-1},	  {1,1},	   {0,0},	    {-1,1},	     {1,-1}, 	  {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100},	// x
		 {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100},
		 {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100},
		 {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}},

		{{-1,-1},	  {-1,0},	   {-1,1},		{-1,0},		 {0,0},		  {0,1},	   {1,-1},		{1,0},		 {1,1},       {-100,-100},	// full-tight
		 {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100},
		 {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100},
		 {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}},

		{{0,-2},	  {-1,-1},	   {1,-1},		{-2,0},		 {0,0},		  {2,0},	   {-1,1},		{1,1},		 {0,2},       {-100,-100},	// full-spread-9
		 {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100},
		 {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100},
		 {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}},

		{{0,-2},	  {-1,-1},	   {1,-1},		{-2,0},		 {0,0},		  {2,0},	   {-1,1},		{1,1},		 {0,2},       {-2,-2},   // full-spread-13
		 {-2,2},      {2,-2},      {2,2},       {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100},
		 {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100},
		 {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}},

		{{-2,-2},     {-2,-1}, {-2,-0}, {-2,1}, {-2,2}, {-1,-2}, {-1,-1}, {-1,-0}, {-1,1}, {-1,2}, 										// full-25
		 {-0,-2},     {-0,-1}, {-0,-0}, {-0,1}, {-0,2}, {+1,-2}, {+1,-1}, {+1,-0}, {+1,1}, {+1,2},
		 {+2,-2}, 	  {+2,-1}, {+2,-0}, {+2,1}, {+2,2}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100},
		 {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}},

		{{0,-2},	  {-1,-1},	   {1,-1},		{-2,0},		 {0,0},		  {2,0},	   {-1,1},		{1,1},		 {0,2},       {-2,-2},   // full-spread-21
		 {-2,2},      {2,-2},      {2,2},       {-3,-1},     {-3,1},      {3,-1}, 	   {3,1},       {1,-3},      {-1,-3},     {1,3},
		 {-1,3},      {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100},
		 {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}},

		{{0,-2},	  {-1,-1},	   {1,-1},		{-2,0},		 {0,0},		  {2,0},	   {-1,1},		{0,2},		 {-100,-100}, {-100,-100},	// 8 for SSE efficiency
		 {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100},
		 {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100},
		 {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}},

		{{-4,-4},     {-4,-2}, {-4,-0}, {-4,2}, {-4,4}, {-2,-4}, {-2,-2}, {-2,-0}, {-2,2}, {-2,4}, 										// full-45-SPREAD
		 {-0,-4},     {-0,-2}, {-0,-0}, {-0,2}, {-0,4}, {+2,-4}, {+2,-2}, {+2,-0}, {+2,2}, {+2,4},
		 {+4,-4}, 	  {+4,-2}, {+4,-0}, {+4,2}, {+4,4}, {-200,-200}, {-200,-200}, {-200,-200}, {-200,-200}, {-200,-200},
		 {-200,-200}, {-200,-200}, {-200,-200}, {-200,-200}, {-200,-200}, {-200,-200}, {-200,-200}, {-200,-200}, {-200,-200}, {-200,-200}},
};

int staticPatternNum[10] = {
		1,
		5,
		5,
		9,
		9,
		13,
		25,
		21,
		8,
		25
};

int staticPatternPadding[10] = {
		1,
		1,
		1,
		1,
		2,
		2,
		2,
		3,
		2,
		4
};


}
