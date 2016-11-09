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
#include "FullSystem/FullSystem.h"
#include "FullSystem/HessianBlocks.h"
#include "util/settings.h"

namespace dso
{


EIGEN_STRONG_INLINE float derive_idepth(
		const Vec3f &t, const float &u, const float &v,
		const int &dx, const int &dy, const float &dxInterp,
		const float &dyInterp, const float &drescale)
{
	return (dxInterp*drescale * (t[0]-t[2]*u)
			+ dyInterp*drescale * (t[1]-t[2]*v))*SCALE_IDEPTH;
}



EIGEN_STRONG_INLINE bool projectPoint(
		const float &u_pt,const float &v_pt,
		const float &idepth,
		const Mat33f &KRKi, const Vec3f &Kt,
		float &Ku, float &Kv)
{
	Vec3f ptp = KRKi * Vec3f(u_pt,v_pt, 1) + Kt*idepth;
	Ku = ptp[0] / ptp[2];
	Kv = ptp[1] / ptp[2];
	return Ku>1.1f && Kv>1.1f && Ku<wM3G && Kv<hM3G;
}



EIGEN_STRONG_INLINE bool projectPoint(
		const float &u_pt,const float &v_pt,
		const float &idepth,
		const int &dx, const int &dy,
		CalibHessian* const &HCalib,
		const Mat33f &R, const Vec3f &t,
		float &drescale, float &u, float &v,
		float &Ku, float &Kv, Vec3f &KliP, float &new_idepth)
{
	KliP = Vec3f(
			(u_pt+dx-HCalib->cxl())*HCalib->fxli(),
			(v_pt+dy-HCalib->cyl())*HCalib->fyli(),
			1);

	Vec3f ptp = R * KliP + t*idepth;
	drescale = 1.0f/ptp[2];
	new_idepth = idepth*drescale;

	if(!(drescale>0)) return false;

	u = ptp[0] * drescale;
	v = ptp[1] * drescale;
	Ku = u*HCalib->fxl() + HCalib->cxl();
	Kv = v*HCalib->fyl() + HCalib->cyl();

	return Ku>1.1f && Kv>1.1f && Ku<wM3G && Kv<hM3G;
}




}

