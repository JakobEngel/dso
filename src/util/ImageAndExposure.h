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
#include <cstring>
#include <iostream>


namespace dso
{


class ImageAndExposure
{
public:
	float* image;			// irradiance. between 0 and 256
	int w,h;				// width and height;
	double timestamp;
	bool* overexposedMap;	// true if source pixel was overexposed.
	float exposure_time;	// exposure time in ms.

	float* imageOriginal;			// irradiance. between 0 and 256
	int wOrg,hOrg;				// width and height;
	inline ImageAndExposure(int w_, int h_, double timestamp_=0) : w(w_), h(h_), timestamp(timestamp_)
	{
		image = new float[w*h];
		overexposedMap = new bool[w*h];
		exposure_time=1;

		imageOriginal=0;
		wOrg=hOrg=0;
	}
	inline ~ImageAndExposure()
	{
		delete[] image;
		delete[] overexposedMap;

		if(imageOriginal != 0) delete[] imageOriginal;
	}


	inline void copyMetaTo(ImageAndExposure &other)
	{
		other.exposure_time = exposure_time;
	}

	inline ImageAndExposure* getDeepCopy()
	{
		ImageAndExposure* img = new ImageAndExposure(w,h,timestamp);
		img->wOrg = wOrg;
		img->hOrg = hOrg;
		img->exposure_time = exposure_time;


		memcpy(img->image, image, w*h*sizeof(float));

		if(imageOriginal != 0)
		{
			img->imageOriginal = new float[wOrg*hOrg];
			memcpy(img->imageOriginal, imageOriginal, wOrg*hOrg*sizeof(float));
		}

		return img;
	}
};


}
