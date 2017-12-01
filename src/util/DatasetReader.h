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
#include "util/settings.h"
#include "util/globalFuncs.h"
#include "util/globalCalib.h"

#include <sstream>
#include <fstream>
#include <dirent.h>
#include <algorithm>

#include "util/Undistort.h"
#include "IOWrapper/ImageRW.h"

#if HAS_ZIPLIB
    #include "zip.h"
#endif

#include <boost/thread.hpp>
#include <boost/lexical_cast.hpp>

using namespace dso;



inline int getdir (std::string dir, std::vector<std::string> &files)
{
    DIR *dp;
    struct dirent *dirp;
    if((dp  = opendir(dir.c_str())) == NULL)
    {
        return -1;
    }

    while ((dirp = readdir(dp)) != NULL) {
        std::string name = std::string(dirp->d_name);

        if(name != "." && name != "..")
            files.push_back(name);
    }
    closedir(dp);


    std::sort(files.begin(), files.end());

    if(dir.at( dir.length() - 1 ) != '/') dir = dir+"/";
    for(unsigned int i=0;i<files.size();i++)
    {
        if(files[i].at(0) != '/')
            files[i] = dir + files[i];
    }

    return files.size();
}


struct PrepImageItem
{
    int id;
    bool isQueud;
    ImageAndExposure* pt;

    inline PrepImageItem(int _id)
    {
        id=_id;
        isQueud = false;
        pt=0;
    }

    inline void release()
    {
        if(pt!=0) delete pt;
        pt=0;
    }
};




class ImageFolderReader
{
public:
    ImageFolderReader(std::string pathL, std::string pathR, std::string calibFile, std::string gammaFile, std::string vignetteFile)
    {

        this->pathL = pathL;
        this->pathR = pathR;
        this->calibfile = calibFile;

#if HAS_ZIPLIB
        ziparchiveL=0;
        ziparchiveR=0;
        databufferL=0;
        databufferR=0;
#endif

        bool isZippedL = (pathL.length()>4 && pathL.substr(pathL.length()-4) == ".zip");
        bool isZippedR = (pathR.length()>4 && pathR.substr(pathR.length()-4) == ".zip");

        // LEFT IMAGE FOLDER
        if(isZippedL)
        {
#if HAS_ZIPLIB
            int ziperror=0;
            ziparchiveL = zip_open(path.c_str(),  ZIP_RDONLY, &ziperror);
            if(ziperror!=0)
            {
                printf("ERROR %d reading archive %s!\n", ziperror, pathL.c_str());
                exit(1);
            }

            filesL.clear();
            int numEntries = zip_get_num_entries(ziparchiveL, 0);
            for(int k=0;k<numEntries;k++)
            {
                const char* name = zip_get_name(ziparchiveL, k,  ZIP_FL_ENC_STRICT);
                std::string nstr = std::string(name);
                if(nstr == "." || nstr == "..") continue;
                filesL.push_back(name);
            }

            printf("got %d entries and %d files!\n", numEntries, (int)filesR.size());
            std::sort(filesL.begin(), filesL.end());
#else
            printf("ERROR: cannot read .zip archive, as compile without ziplib!\n");
            exit(1);
#endif
        }
        else
            getdir(pathL, filesL);

        // RIGHT IMAGE FOLDER
        if(isZippedR)
        {
#if HAS_ZIPLIB
            int ziperror=0;
            ziparchiveR = zip_open(path.c_str(),  ZIP_RDONLY, &ziperror);
            if(ziperror!=0)
            {
                printf("ERROR %d reading archive %s!\n", ziperror, pathR.c_str());
                exit(1);
            }

            filesR.clear();
            int numEntries = zip_get_num_entries(ziparchiveR, 0);
            for(int k=0;k<numEntries;k++)
            {
                const char* name = zip_get_name(ziparchiveR, k,  ZIP_FL_ENC_STRICT);
                std::string nstr = std::string(name);
                if(nstr == "." || nstr == "..") continue;
                filesR.push_back(name);
            }

            printf("got %d entries and %d files!\n", numEntries, (int)filesR.size());
            std::sort(filesR.begin(), filesR.end());
#else
            printf("ERROR: cannot read .zip archive, as compile without ziplib!\n");
            exit(1);
#endif
        }
        else
            getdir(pathR, filesR);

        // Make sure we have equal amount left and right
        if(filesL.size() != filesR.size()) {
            printf("ERROR: Left images do not match right images! Left %d and right %d\n",(int)filesL.size(),(int)filesR.size());
            exit(1);
        }


        undistort = Undistort::getUndistorterForFile(calibFile, gammaFile, vignetteFile);

        widthOrg = undistort->getOriginalSize()[0];
        heightOrg = undistort->getOriginalSize()[1];
        width = undistort->getSize()[0];
        height = undistort->getSize()[1];

        // Open our config file so we can get the transform matrix
        printf("STEREO: Loading config transform information\n");
        std::ifstream f(calibFile.c_str());
        if (!f.good()) {
            printf("ERROR: Cannot operate without calibration, shutting down.\n");
            f.close();
            exit(1);
        }

        // Loop through and get the last line in the config
        std::string l1;
        while(f.good()) {
            std::getline(f,l1);
        }
        f.close();

        // Done, lets read in the matrix
        float ic[1];
        std::sscanf(l1.c_str(),"%f",&ic[0]);
        baselinefx = ic[0];

        // Nice printing for the user
        printf("STEREO: Using following baseline:\n");
        printf("%.4f (baselinetimes fx)\n", baselinefx);
        printf("%.4f (meters)\n", baselinefx/undistort->getK()(0,0));


        // Load timestamps if possible.
        loadTimestamps();
        printf("ImageFolderReader: LEFT got %d files in %s!\n", (int)filesL.size(), pathL.c_str());
        printf("ImageFolderReader: RIGHT got %d files in %s!\n", (int)filesR.size(), pathR.c_str());

    }
    ~ImageFolderReader()
    {
#if HAS_ZIPLIB
        if(ziparchiveL!=0) zip_close(ziparchiveL);
        if(ziparchiveR!=0) zip_close(ziparchiveR);
        if(databufferL!=0) delete databufferL;
        if(databufferR!=0) delete databufferR;
#endif

        delete undistort;
    };

    Eigen::VectorXf getOriginalCalib()
    {
        return undistort->getOriginalParameter().cast<float>();
    }
    Eigen::Vector2i getOriginalDimensions()
    {
        return  undistort->getOriginalSize();
    }

    void getCalibMono(Eigen::Matrix3f &K, int &w, int &h)
    {
        K = undistort->getK().cast<float>();
        w = undistort->getSize()[0];
        h = undistort->getSize()[1];
    }

    void setGlobalCalibration()
    {
        int w_out, h_out;
        Eigen::Matrix3f K;
        getCalibMono(K, w_out, h_out);
        setGlobalCalib(w_out, h_out, K);
    }

    int getNumImages()
    {
        return filesL.size();
    }

    double getTimestamp(int id)
    {
        if(timestamps.size()==0) return id*0.1f;
        if(id >= (int)timestamps.size()) return 0;
        if(id < 0) return 0;
        return timestamps[id];
    }

    
    void prepImage(int id, bool as8U=false)
    {

    }

    MinimalImageB* getImageRawL(int id)
    {
            return getImageRaw_internalLEFT(id,0);
    }

    MinimalImageB* getImageRawR(int id)
    {
        return getImageRaw_internalRIGHT(id,0);
    }

    ImageAndExposure* getImage(int id, bool forceLoadDirectly=false)
    {
        return getImage_internal(id, 0);
    }


    inline float* getPhotometricGamma()
    {
        if(undistort==0 || undistort->photometricUndist==0) return 0;
        return undistort->photometricUndist->getG();
    }


    // undistorter. [0] always exists, [1-2] only when MT is enabled.
    // note we only need one since our stereo images are rectified
    Undistort* undistort;
private:

    /**
     * Epoch time conversion
     * http://www.epochconverter.com/programming/functions-c.php
     */
    double parseTime(std::string timestamp)
    {
        // example: 2011-09-26 13:21:35.134391552
        //          01234567891111111111222222222
        //                    0123456789012345678
        struct tm t = {0};  // Initalize to all 0's
        std::cout << timestamp << std::endl;
        t.tm_year = boost::lexical_cast<int>(timestamp.substr(0, 4)) - 1900;
        t.tm_mon  = boost::lexical_cast<int>(timestamp.substr(5, 2)) - 1;
        t.tm_mday = boost::lexical_cast<int>(timestamp.substr(8, 2));
        t.tm_hour = boost::lexical_cast<int>(timestamp.substr(11, 2));
        t.tm_min  = boost::lexical_cast<int>(timestamp.substr(14, 2));
        t.tm_sec  = boost::lexical_cast<int>(timestamp.substr(17, 2));
        t.tm_isdst = -1;
        time_t timeSinceEpoch = mktime(&t);
        return timeSinceEpoch;
    }


    MinimalImageB* getImageRaw_internalLEFT(int id, int unused)
    {
        if(!isZippedL)
        {
            // CHANGE FOR ZIP FILE
            return IOWrap::readImageBW_8U(filesL[id]);
        }
        else
        {
#if HAS_ZIPLIB
            if(databufferL==0) databuffer = new char[widthOrg*heightOrg*6+10000];
            zip_file_t* fle = zip_fopen(ziparchiveL, filesL[id].c_str(), 0);
            long readbytes = zip_fread(fle, databufferL, (long)widthOrg*heightOrg*6+10000);

            if(readbytes > (long)widthOrg*heightOrg*6)
            {
                printf("read %ld/%ld bytes for file %s. increase buffer!!\n", readbytes,(long)widthOrg*heightOrg*6+10000, filesL[id].c_str());
                delete[] databufferL;
                databufferL = new char[(long)widthOrg*heightOrg*30];
                fle = zip_fopen(ziparchiveL, filesL[id].c_str(), 0);
                readbytes = zip_fread(fle, databufferL, (long)widthOrg*heightOrg*30+10000);

                if(readbytes > (long)widthOrg*heightOrg*30)
                {
                    printf("buffer still to small (read %ld/%ld). abort.\n", readbytes,(long)widthOrg*heightOrg*30+10000);
                    exit(1);
                }
            }

            return IOWrap::readStreamBW_8U(databufferL, readbytes);
#else
            printf("ERROR: cannot read .zip archive, as compile without ziplib!\n");
            exit(1);
#endif
        }
    }

    MinimalImageB* getImageRaw_internalRIGHT(int id, int unused)
    {
        if(!isZippedL)
        {
            // CHANGE FOR ZIP FILE
            return IOWrap::readImageBW_8U(filesR[id]);
        }
        else
        {
#if HAS_ZIPLIB
            if(databufferR==0) databuffer = new char[widthOrg*heightOrg*6+10000];
            zip_file_t* fle = zip_fopen(ziparchiveR, filesR[id].c_str(), 0);
            long readbytes = zip_fread(fle, databufferR, (long)widthOrg*heightOrg*6+10000);

            if(readbytes > (long)widthOrg*heightOrg*6)
            {
                printf("read %ld/%ld bytes for file %s. increase buffer!!\n", readbytes,(long)widthOrg*heightOrg*6+10000, filesR[id].c_str());
                delete[] databufferR;
                databufferR = new char[(long)widthOrg*heightOrg*30];
                fle = zip_fopen(ziparchiveR, filesR[id].c_str(), 0);
                readbytes = zip_fread(fle, databufferR, (long)widthOrg*heightOrg*30+10000);

                if(readbytes > (long)widthOrg*heightOrg*30)
                {
                    printf("buffer still to small (read %ld/%ld). abort.\n", readbytes,(long)widthOrg*heightOrg*30+10000);
                    exit(1);
                }
            }

            return IOWrap::readStreamBW_8U(databufferR, readbytes);
#else
            printf("ERROR: cannot read .zip archive, as compile without ziplib!\n");
            exit(1);
#endif
        }
    }


    ImageAndExposure* getImage_internal(int id, int unused)
    {
        MinimalImageB* minimgL = getImageRaw_internalLEFT(id, 0);
        MinimalImageB* minimgR = getImageRaw_internalRIGHT(id, 0);
        ImageAndExposure* ret2 = undistort->undistort<unsigned char>(
                minimgL,
                minimgR,
                (exposures.size() == 0 ? 1.0f : exposures[id]),
                (timestamps.size() == 0 ? 0.0 : timestamps[id]));
        // Delete our incoming buffers
        delete minimgL;
        delete minimgR;
        // Set the stereo configuration values here
        ret2->baselinefx = baselinefx;
        return ret2;
    }

    inline void loadTimestamps()
    {
        std::ifstream tr;
        std::string timesFile = pathL.substr(0,pathL.find_last_of('/')) + "/../times.txt";
        tr.open(timesFile.c_str());
        while(!tr.eof() && tr.good()) {
            // Read in line from file
            std::string line;
            std::getline(tr,line);
            // Skip if empty
            if(line == "" || line == "\n" || line == " \n")
                continue;
            // TODO: Look into finding a non-kitti dataset that has exposure times
            // Convert data from RAW datasets
            //double stamp = parseTime(line);
            // Simple timestamp for ODOMETRY datasets
            double stamp = std::stod(line);
            timestamps.push_back(stamp);
            //exposures.push_back(0);
        }
        tr.close();

        // Check to make sure our timestamps match our images
        if((int)getNumImages() != (int)timestamps.size()) {
            printf("DATASET: Set timestamps and exposures to zero! (numImages = %d numTimes = %d)\n",(int)getNumImages(),(int)timestamps.size());
            exposures.clear();
            timestamps.clear();
        }

        // TODO: Look into finding a non-kitti dataset that has exposure times
        bool exposuresGood = false;
        if((int)getNumImages() != (int)exposures.size() || !exposuresGood)
        {
            printf("DATASET: Set EXPOSURES to zero! (numImages = %d numExpo = %d)\n",(int)getNumImages(),(int)exposures.size());
            exposures.clear();
        }

        printf("DATASET: got %d images and %d timestamps and %d exposures.!\n", (int)getNumImages(), (int)timestamps.size(), (int)exposures.size());
    }

    std::vector<ImageAndExposure*> preloadedImages;
    std::vector<std::string> filesL;
    std::vector<std::string> filesR;
    std::vector<double> timestamps;
    std::vector<float> exposures;

    // Image size information
    int width, height;
    int widthOrg, heightOrg;

    // Stereo config information
    float baselinefx;

    std::string pathL;
    std::string pathR;
    std::string calibfile;

    bool isZippedL;
    bool isZippedR;

#if HAS_ZIPLIB
    zip_t* ziparchiveL;
    zip_t* ziparchiveR;
    char* databufferL;
    char* databufferR;
#endif
};

