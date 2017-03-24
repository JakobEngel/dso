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
#define MAX_ACTIVE_FRAMES 100

 
#include "util/globalCalib.h"
#include "vector"
 
#include <iostream>
#include <fstream>
#include "util/NumType.h"
#include "FullSystem/Residuals.h"
#include "util/ImageAndExposure.h"


namespace dso
{


inline Vec2 affFromTo(const Vec2 &from, const Vec2 &to)	// contains affine parameters as XtoWorld.
{
	return Vec2(from[0] / to[0], (from[1] - to[1]) / to[0]);
}


struct FrameHessian;
struct PointHessian;

class ImmaturePoint;
class FrameShell;

class EFFrame;
class EFPoint;

#define SCALE_IDEPTH 1.0f		// scales internal value to idepth.
#define SCALE_XI_ROT 1.0f
#define SCALE_XI_TRANS 0.5f
#define SCALE_F 50.0f
#define SCALE_C 50.0f
#define SCALE_W 1.0f
#define SCALE_A 10.0f
#define SCALE_B 1000.0f

#define SCALE_IDEPTH_INVERSE (1.0f / SCALE_IDEPTH)
#define SCALE_XI_ROT_INVERSE (1.0f / SCALE_XI_ROT)
#define SCALE_XI_TRANS_INVERSE (1.0f / SCALE_XI_TRANS)
#define SCALE_F_INVERSE (1.0f / SCALE_F)
#define SCALE_C_INVERSE (1.0f / SCALE_C)
#define SCALE_W_INVERSE (1.0f / SCALE_W)
#define SCALE_A_INVERSE (1.0f / SCALE_A)
#define SCALE_B_INVERSE (1.0f / SCALE_B)


struct FrameFramePrecalc
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	// static values
	static int instanceCounter;
	FrameHessian* host;	// defines row
	FrameHessian* target;	// defines column

	// precalc values
	Mat33f PRE_RTll;
	Mat33f PRE_KRKiTll;
	Mat33f PRE_RKiTll;
	Mat33f PRE_RTll_0;

	Vec2f PRE_aff_mode;
	float PRE_b0_mode;

	Vec3f PRE_tTll;
	Vec3f PRE_KtTll;
	Vec3f PRE_tTll_0;

	float distanceLL;


    inline ~FrameFramePrecalc() {}
    inline FrameFramePrecalc() {host=target=0;}
	void set(FrameHessian* host, FrameHessian* target, CalibHessian* HCalib);
};





struct FrameHessian
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	EFFrame* efFrame;

	// constant info & pre-calculated values
	//DepthImageWrap* frame;
	FrameShell* shell;

	Eigen::Vector3f* dI;				 // trace, fine tracking. Used for direction select (not for gradient histograms etc.)
	Eigen::Vector3f* dIp[PYR_LEVELS];	 // coarse tracking / coarse initializer. NAN in [0] only.
	float* absSquaredGrad[PYR_LEVELS];  // only used for pixel select (histograms etc.). no NAN.






	int frameID;						// incremental ID for keyframes only!
	static int instanceCounter;
	int idx;

	// Photometric Calibration Stuff
	float frameEnergyTH;	// set dynamically depending on tracking residual
	float ab_exposure;

	bool flaggedForMarginalization;

	std::vector<PointHessian*> pointHessians;				// contains all ACTIVE points.
	std::vector<PointHessian*> pointHessiansMarginalized;	// contains all MARGINALIZED points (= fully marginalized, usually because point went OOB.)
	std::vector<PointHessian*> pointHessiansOut;		// contains all OUTLIER points (= discarded.).
	std::vector<ImmaturePoint*> immaturePoints;		// contains all OUTLIER points (= discarded.).


	Mat66 nullspaces_pose;
	Mat42 nullspaces_affine;
	Vec6 nullspaces_scale;

	// variable info.
	SE3 worldToCam_evalPT;
	Vec10 state_zero;
	Vec10 state_scaled;
	Vec10 state;	// [0-5: worldToCam-leftEps. 6-7: a,b]
	Vec10 step;
	Vec10 step_backup;
	Vec10 state_backup;


    EIGEN_STRONG_INLINE const SE3 &get_worldToCam_evalPT() const {return worldToCam_evalPT;}
    EIGEN_STRONG_INLINE const Vec10 &get_state_zero() const {return state_zero;}
    EIGEN_STRONG_INLINE const Vec10 &get_state() const {return state;}
    EIGEN_STRONG_INLINE const Vec10 &get_state_scaled() const {return state_scaled;}
    EIGEN_STRONG_INLINE const Vec10 get_state_minus_stateZero() const {return get_state() - get_state_zero();}


	// precalc values
	SE3 PRE_worldToCam;
	SE3 PRE_camToWorld;
	std::vector<FrameFramePrecalc,Eigen::aligned_allocator<FrameFramePrecalc>> targetPrecalc;
	MinimalImageB3* debugImage;


    inline Vec6 w2c_leftEps() const {return get_state_scaled().head<6>();}
    inline AffLight aff_g2l() const {return AffLight(get_state_scaled()[6], get_state_scaled()[7]);}
    inline AffLight aff_g2l_0() const {return AffLight(get_state_zero()[6]*SCALE_A, get_state_zero()[7]*SCALE_B);}



	void setStateZero(const Vec10 &state_zero);
	inline void setState(const Vec10 &state)
	{

		this->state = state;
		state_scaled.segment<3>(0) = SCALE_XI_TRANS * state.segment<3>(0);
		state_scaled.segment<3>(3) = SCALE_XI_ROT * state.segment<3>(3);
		state_scaled[6] = SCALE_A * state[6];
		state_scaled[7] = SCALE_B * state[7];
		state_scaled[8] = SCALE_A * state[8];
		state_scaled[9] = SCALE_B * state[9];

		PRE_worldToCam = SE3::exp(w2c_leftEps()) * get_worldToCam_evalPT();
		PRE_camToWorld = PRE_worldToCam.inverse();
		//setCurrentNullspace();
	};
	inline void setStateScaled(const Vec10 &state_scaled)
	{

		this->state_scaled = state_scaled;
		state.segment<3>(0) = SCALE_XI_TRANS_INVERSE * state_scaled.segment<3>(0);
		state.segment<3>(3) = SCALE_XI_ROT_INVERSE * state_scaled.segment<3>(3);
		state[6] = SCALE_A_INVERSE * state_scaled[6];
		state[7] = SCALE_B_INVERSE * state_scaled[7];
		state[8] = SCALE_A_INVERSE * state_scaled[8];
		state[9] = SCALE_B_INVERSE * state_scaled[9];

		PRE_worldToCam = SE3::exp(w2c_leftEps()) * get_worldToCam_evalPT();
		PRE_camToWorld = PRE_worldToCam.inverse();
		//setCurrentNullspace();
	};
	inline void setEvalPT(const SE3 &worldToCam_evalPT, const Vec10 &state)
	{

		this->worldToCam_evalPT = worldToCam_evalPT;
		setState(state);
		setStateZero(state);
	};



	inline void setEvalPT_scaled(const SE3 &worldToCam_evalPT, const AffLight &aff_g2l)
	{
		Vec10 initial_state = Vec10::Zero();
		initial_state[6] = aff_g2l.a;
		initial_state[7] = aff_g2l.b;
		this->worldToCam_evalPT = worldToCam_evalPT;
		setStateScaled(initial_state);
		setStateZero(this->get_state());
	};

	void release();

	inline ~FrameHessian()
	{
		assert(efFrame==0);
		release(); instanceCounter--;
		for(int i=0;i<pyrLevelsUsed;i++)
		{
			delete[] dIp[i];
			delete[]  absSquaredGrad[i];

		}



		if(debugImage != 0) delete debugImage;
	};
	inline FrameHessian()
	{
		instanceCounter++;
		flaggedForMarginalization=false;
		frameID = -1;
		efFrame = 0;
		frameEnergyTH = 8*8*patternNum;



		debugImage=0;
	};


    void makeImages(float* color, CalibHessian* HCalib);

	inline Vec10 getPrior()
	{
		Vec10 p =  Vec10::Zero();
		if(frameID==0)
		{
			p.head<3>() = Vec3::Constant(setting_initialTransPrior);
			p.segment<3>(3) = Vec3::Constant(setting_initialRotPrior);
			if(setting_solverMode & SOLVER_REMOVE_POSEPRIOR) p.head<6>().setZero();

			p[6] = setting_initialAffAPrior;
			p[7] = setting_initialAffBPrior;
		}
		else
		{
			if(setting_affineOptModeA < 0)
				p[6] = setting_initialAffAPrior;
			else
				p[6] = setting_affineOptModeA;

			if(setting_affineOptModeB < 0)
				p[7] = setting_initialAffBPrior;
			else
				p[7] = setting_affineOptModeB;
		}
		p[8] = setting_initialAffAPrior;
		p[9] = setting_initialAffBPrior;
		return p;
	}


	inline Vec10 getPriorZero()
	{
		return Vec10::Zero();
	}

};

struct CalibHessian
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	static int instanceCounter;

	VecC value_zero;
	VecC value_scaled;
	VecCf value_scaledf;
	VecCf value_scaledi;
	VecC value;
	VecC step;
	VecC step_backup;
	VecC value_backup;
	VecC value_minus_value_zero;

    inline ~CalibHessian() {instanceCounter--;}
	inline CalibHessian()
	{

		VecC initial_value = VecC::Zero();
		initial_value[0] = fxG[0];
		initial_value[1] = fyG[0];
		initial_value[2] = cxG[0];
		initial_value[3] = cyG[0];

		setValueScaled(initial_value);
		value_zero = value;
		value_minus_value_zero.setZero();

		instanceCounter++;
		for(int i=0;i<256;i++)
			Binv[i] = B[i] = i;		// set gamma function to identity
	};


	// normal mode: use the optimized parameters everywhere!
    inline float& fxl() {return value_scaledf[0];}
    inline float& fyl() {return value_scaledf[1];}
    inline float& cxl() {return value_scaledf[2];}
    inline float& cyl() {return value_scaledf[3];}
    inline float& fxli() {return value_scaledi[0];}
    inline float& fyli() {return value_scaledi[1];}
    inline float& cxli() {return value_scaledi[2];}
    inline float& cyli() {return value_scaledi[3];}



	inline void setValue(const VecC &value)
	{
		// [0-3: Kl, 4-7: Kr, 8-12: l2r]
		this->value = value;
		value_scaled[0] = SCALE_F * value[0];
		value_scaled[1] = SCALE_F * value[1];
		value_scaled[2] = SCALE_C * value[2];
		value_scaled[3] = SCALE_C * value[3];

		this->value_scaledf = this->value_scaled.cast<float>();
		this->value_scaledi[0] = 1.0f / this->value_scaledf[0];
		this->value_scaledi[1] = 1.0f / this->value_scaledf[1];
		this->value_scaledi[2] = - this->value_scaledf[2] / this->value_scaledf[0];
		this->value_scaledi[3] = - this->value_scaledf[3] / this->value_scaledf[1];
		this->value_minus_value_zero = this->value - this->value_zero;
	};

	inline void setValueScaled(const VecC &value_scaled)
	{
		this->value_scaled = value_scaled;
		this->value_scaledf = this->value_scaled.cast<float>();
		value[0] = SCALE_F_INVERSE * value_scaled[0];
		value[1] = SCALE_F_INVERSE * value_scaled[1];
		value[2] = SCALE_C_INVERSE * value_scaled[2];
		value[3] = SCALE_C_INVERSE * value_scaled[3];

		this->value_minus_value_zero = this->value - this->value_zero;
		this->value_scaledi[0] = 1.0f / this->value_scaledf[0];
		this->value_scaledi[1] = 1.0f / this->value_scaledf[1];
		this->value_scaledi[2] = - this->value_scaledf[2] / this->value_scaledf[0];
		this->value_scaledi[3] = - this->value_scaledf[3] / this->value_scaledf[1];
	};


	float Binv[256];
	float B[256];


	EIGEN_STRONG_INLINE float getBGradOnly(float color)
	{
		int c = color+0.5f;
		if(c<5) c=5;
		if(c>250) c=250;
		return B[c+1]-B[c];
	}

	EIGEN_STRONG_INLINE float getBInvGradOnly(float color)
	{
		int c = color+0.5f;
		if(c<5) c=5;
		if(c>250) c=250;
		return Binv[c+1]-Binv[c];
	}
};


// hessian component associated with one point.
struct PointHessian
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	static int instanceCounter;
	EFPoint* efPoint;

	// static values
	float color[MAX_RES_PER_POINT];			// colors in host frame
	float weights[MAX_RES_PER_POINT];		// host-weights for respective residuals.



	float u,v;
	int idx;
	float energyTH;
	FrameHessian* host;
	bool hasDepthPrior;

	float my_type;

	float idepth_scaled;
	float idepth_zero_scaled;
	float idepth_zero;
	float idepth;
	float step;
	float step_backup;
	float idepth_backup;

	float nullspaces_scale;
	float idepth_hessian;
	float maxRelBaseline;
	int numGoodResiduals;

	enum PtStatus {ACTIVE=0, INACTIVE, OUTLIER, OOB, MARGINALIZED};
	PtStatus status;

    inline void setPointStatus(PtStatus s) {status=s;}


	inline void setIdepth(float idepth) {
		this->idepth = idepth;
		this->idepth_scaled = SCALE_IDEPTH * idepth;
    }
	inline void setIdepthScaled(float idepth_scaled) {
		this->idepth = SCALE_IDEPTH_INVERSE * idepth_scaled;
		this->idepth_scaled = idepth_scaled;
    }
	inline void setIdepthZero(float idepth) {
		idepth_zero = idepth;
		idepth_zero_scaled = SCALE_IDEPTH * idepth;
		nullspaces_scale = -(idepth*1.001 - idepth/1.001)*500;
    }


	std::vector<PointFrameResidual*> residuals;					// only contains good residuals (not OOB and not OUTLIER). Arbitrary order.
	std::pair<PointFrameResidual*, ResState> lastResiduals[2]; 	// contains information about residuals to the last two (!) frames. ([0] = latest, [1] = the one before).


	void release();
	PointHessian(const ImmaturePoint* const rawPoint, CalibHessian* Hcalib);
    inline ~PointHessian() {assert(efPoint==0); release(); instanceCounter--;}


	inline bool isOOB(const std::vector<FrameHessian*>& toKeep, const std::vector<FrameHessian*>& toMarg) const
	{

		int visInToMarg = 0;
		for(PointFrameResidual* r : residuals)
		{
			if(r->state_state != ResState::IN) continue;
			for(FrameHessian* k : toMarg)
				if(r->target == k) visInToMarg++;
		}
		if((int)residuals.size() >= setting_minGoodActiveResForMarg &&
				numGoodResiduals > setting_minGoodResForMarg+10 &&
				(int)residuals.size()-visInToMarg < setting_minGoodActiveResForMarg)
			return true;





		if(lastResiduals[0].second == ResState::OOB) return true;
		if(residuals.size() < 2) return false;
		if(lastResiduals[0].second == ResState::OUTLIER && lastResiduals[1].second == ResState::OUTLIER) return true;
		return false;
	}


	inline bool isInlierNew()
	{
		return (int)residuals.size() >= setting_minGoodActiveResForMarg
                    && numGoodResiduals >= setting_minGoodResForMarg;
	}

};





}

