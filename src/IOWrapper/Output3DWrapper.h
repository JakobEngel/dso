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
#include <string>

#include "util/NumType.h"
#include "util/MinimalImage.h"
#include "map"

namespace cv {
	class Mat;
}

namespace dso
{

class FrameHessian;
class CalibHessian;
class FrameShell;

namespace IOWrap
{

class Output3DWrapper
{
public:
	Output3DWrapper() {};
	virtual ~Output3DWrapper() {};
	virtual void publishGraph(const std::map<long,Eigen::Vector2i> &connectivity) {};
	virtual void publishKeyframes(std::vector<FrameHessian*> &frames, bool final, CalibHessian* HCalib) {};
	virtual void publishCamPose(FrameShell* frame, CalibHessian* HCalib) {};

	virtual void pushLiveFrame(FrameHessian* image) {};
	virtual void pushResidualImage(MinimalImageB3* image) {};
	virtual void pushDepthImage(MinimalImageB3* image) {};

	virtual void join() {};
	virtual void reset() {};
};

}
}
