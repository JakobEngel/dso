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
#include <vector>
#include "util/NumType.h"
#include "util/MinimalImage.h"


namespace dso
{

namespace IOWrap
{

void displayImage(const char* windowName, const MinimalImageB* img, bool autoSize = false);
void displayImage(const char* windowName, const MinimalImageB3* img, bool autoSize = false);
void displayImage(const char* windowName, const MinimalImageF* img, bool autoSize = false);
void displayImage(const char* windowName, const MinimalImageF3* img, bool autoSize = false);
void displayImage(const char* windowName, const MinimalImageB16* img, bool autoSize = false);


void displayImageStitch(const char* windowName, const std::vector<MinimalImageB*> images, int cc=0, int rc=0);
void displayImageStitch(const char* windowName, const std::vector<MinimalImageB3*> images, int cc=0, int rc=0);
void displayImageStitch(const char* windowName, const std::vector<MinimalImageF*> images, int cc=0, int rc=0);
void displayImageStitch(const char* windowName, const std::vector<MinimalImageF3*> images, int cc=0, int rc=0);

int waitKey(int milliseconds);
void closeAllWindows();

}

}
