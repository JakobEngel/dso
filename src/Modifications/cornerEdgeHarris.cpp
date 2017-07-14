/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                        Intel License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000, Intel Corporation, all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of Intel Corporation may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

#include <cstdio>
#include <vector>
#include <iostream>
#include <functional>

#include <opencv/cv.h>
#include <cv.hpp>
#include <chrono>

namespace ORB_SLAM2
{

    struct greaterThanPtr :
            public std::binary_function<const float *, const float *, bool>
    {
        bool operator () (const float * a, const float * b) const
        // Ensure a fully deterministic result of the sort
        { return (*a > *b) ? true : (*a < *b) ? false : (a > b); }
    };


    void cornerEdgeHarrisExtractor(cv::InputArray _image, cv::OutputArray _corners, cv::OutputArray _lambdasVectors,
                                   int maxCorners, double qualityLevel, double minDistance,
                                   cv::InputArray _mask, int blockSize, double qualityThreshold) {

        CV_Assert(qualityLevel > 0 && minDistance >= 0 && maxCorners >= 0);
        CV_Assert(_mask.empty() || (_mask.type() == CV_8UC1 && _mask.sameSize(_image)));

        cv::Mat image = _image.getMat(), eig, tmp;
        if (image.empty()) {
            _corners.release();
            return;
        }


        cv::Mat myHarris_dst = cv::Mat::zeros( image.size(), CV_32FC(6) );
        eig = cv::Mat::zeros( image.size(), CV_32FC1 );

        cornerEigenValsAndVecs( image, myHarris_dst, blockSize, 3, cv::BORDER_DEFAULT );

        int countCorner = 0, countEdge = 0;
        for( int j = 0; j < image.rows; j++ )
        { for( int i = 0; i < image.cols; i++ )
            {
                float lambda_1 = fabs(myHarris_dst.at<cv::Vec6f>(j, i)[0]);
                float lambda_2 = fabs(myHarris_dst.at<cv::Vec6f>(j, i)[1]);

                float maxLambda = std::max(lambda_1, lambda_2);
                float minLambda = std::min(lambda_1, lambda_2);

                // If both lambdas are above the threshold -> it is a corner
                if ( minLambda > qualityThreshold) {
                    eig.at<float>(j, i) = maxLambda;
                    countCorner++;
                }
                    // If only one lambda is above the threshold -> it is an edge
                else if ( maxLambda > qualityThreshold ) {

                    // We will sort by lambdas and we want edges to be sorted according to the difference in lambdas
                    // We also want to prefer corners so edge priority will be
                    // (maxLambda - minLambda)
//                    eig.at<float>(j,i) = (maxLambda - minLambda); /// qualityThreshold;

                    // TODO: Idea to verify that strong edge is as good as strong corner
                    eig.at<float>(j,i) = maxLambda;
                    countEdge++;
                }

            }
        }

        // Originally they did the threshold according to max and also threshold. We do not threshold our points
//        double minVal, maxVal = 0;
//        minMaxLoc(eig, &minVal, &maxVal, 0, 0, _mask);
//        threshold(eig, eig, maxVal * qualityLevel, 0, cv::THRESH_TOZERO);
        dilate(eig, tmp, cv::Mat());


        cv::Size imgsize = image.size();
        std::vector<const float *> tmpCorners;

        // collect list of pointers to features - put them into temporary image
        cv::Mat mask = _mask.getMat();
        for (int y = 1; y < imgsize.height - 1; y++) {
            const float *eig_data = (const float *) eig.ptr(y);
            const float *tmp_data = (const float *) tmp.ptr(y);
            const uchar *mask_data = mask.data ? mask.ptr(y) : 0;

            for (int x = 1; x < imgsize.width - 1; x++) {
                float val = eig_data[x];
                if (val != 0 && val == tmp_data[x] && (!mask_data || mask_data[x]))
                    tmpCorners.push_back(eig_data + x);
            }
        }

        std::vector<cv::Point2f> corners;
        size_t i, j, total = tmpCorners.size(), ncorners = 0;

        if (total == 0) {
            _corners.release();
            return;
        }

        std::sort(tmpCorners.begin(), tmpCorners.end(), greaterThanPtr());

        std::vector<cv::Vec6f> lambdasVectors;
        if (minDistance >= 1) {
            // Partition the image into larger grids
            int w = image.cols;
            int h = image.rows;

            const int cell_size = cvRound(minDistance);
            const int grid_width = (w + cell_size - 1) / cell_size;
            const int grid_height = (h + cell_size - 1) / cell_size;

            std::vector<std::vector<cv::Point2f> > grid(grid_width * grid_height);

            minDistance *= minDistance;

            for (i = 0; i < total; i++) {
                int ofs = (int) ((const uchar *) tmpCorners[i] - eig.ptr());
                int y = (int) (ofs / eig.step);
                int x = (int) ((ofs - y * eig.step) / sizeof(float));

                bool good = true;

                int x_cell = x / cell_size;
                int y_cell = y / cell_size;

                int x1 = x_cell - 1;
                int y1 = y_cell - 1;
                int x2 = x_cell + 1;
                int y2 = y_cell + 1;

                // boundary check
                x1 = std::max(0, x1);
                y1 = std::max(0, y1);
                x2 = std::min(grid_width - 1, x2);
                y2 = std::min(grid_height - 1, y2);

                for (int yy = y1; yy <= y2; yy++) {
                    for (int xx = x1; xx <= x2; xx++) {
                        std::vector<cv::Point2f> &m = grid[yy * grid_width + xx];

                        if (m.size()) {
                            for (j = 0; j < m.size(); j++) {
                                float dx = x - m[j].x;
                                float dy = y - m[j].y;

                                if (dx * dx + dy * dy < minDistance) {
                                    good = false;
                                    goto break_out;
                                }
                            }
                        }
                    }
                }

                break_out:

                if (good) {
                    grid[y_cell * grid_width + x_cell].push_back(cv::Point2f((float) x, (float) y));

                    corners.push_back(cv::Point2f((float) x, (float) y));
                    ++ncorners;

                    if (maxCorners > 0 && (int) ncorners == maxCorners)
                        break;
                }
            }
        } else {
            for (i = 0; i < total; i++) {
                int ofs = (int) ((const uchar *) tmpCorners[i] - eig.ptr());
                int y = (int) (ofs / eig.step);
                int x = (int) ((ofs - y * eig.step) / sizeof(float));


                // Copying also the lambda results
                cv::Vec6f v = myHarris_dst.at<cv::Vec6f>(y, x);
                lambdasVectors.push_back(v);

                corners.push_back(cv::Point2f((float) x, (float) y));
                ++ncorners;
                if (maxCorners > 0 && (int) ncorners == maxCorners)
                    break;
            }
        }

        cv::Mat(corners).convertTo(_corners, _corners.fixedType() ? _corners.type() : CV_32F);
        cv::Mat(lambdasVectors).convertTo(_lambdasVectors, _lambdasVectors.fixedType() ? _lambdasVectors.type() : CV_32F);
    }
}

// TODO: REMOVE AFTER TESTS
//    static void eigen2x2( const float* cov, float* dst, int n )
//    {
//        for( int j = 0; j < n; j++ )
//        {
//            double a = cov[j*3];
//            double b = cov[j*3+1];
//            double c = cov[j*3+2];
//
//            double u = (a + c)*0.5;
//            double v = std::sqrt((a - c)*(a - c)*0.25 + b*b);
//            double l1 = u + v;
//            double l2 = u - v;
//
//            double x = b;
//            double y = l1 - a;
//            double e = fabs(x);
//
//            if( e + fabs(y) < 1e-4 )
//            {
//                y = b;
//                x = l1 - c;
//                e = fabs(x);
//                if( e + fabs(y) < 1e-4 )
//                {
//                    e = 1./(e + fabs(y) + FLT_EPSILON);
//                    x *= e, y *= e;
//                }
//            }
//
//            double d = 1./std::sqrt(x*x + y*y + DBL_EPSILON);
//            dst[6*j] = (float)l1;
//            dst[6*j + 2] = (float)(x*d);
//            dst[6*j + 3] = (float)(y*d);
//
//            x = b;
//            y = l2 - a;
//            e = fabs(x);
//
//            if( e + fabs(y) < 1e-4 )
//            {
//                y = b;
//                x = l2 - c;
//                e = fabs(x);
//                if( e + fabs(y) < 1e-4 )
//                {
//                    e = 1./(e + fabs(y) + FLT_EPSILON);
//                    x *= e, y *= e;
//                }
//            }
//
//            d = 1./std::sqrt(x*x + y*y + DBL_EPSILON);
//            dst[6*j + 1] = (float)l2;
//            dst[6*j + 4] = (float)(x*d);
//            dst[6*j + 5] = (float)(y*d);
//        }
//    }


//    static void calcEigenValsVecs( const cv::Mat& _cov, cv::Mat& _dst )
//    {
//        cv::Size size = _cov.size();
//        if( _cov.isContinuous() && _dst.isContinuous() )
//        {
//            size.width *= size.height;
//            size.height = 1;
//        }
//
//        for( int i = 0; i < size.height; i++ )
//        {
//            const float* cov = _cov.ptr<float>(i);
//            float* dst = _dst.ptr<float>(i);
//
//            eigen2x2(cov, dst, size.width);
//        }
//    }

//    static void calcHarris( const cv::Mat& _cov, cv::Mat& _dst, double k )
//    {
//        int i, j;
//        cv::Size size = _cov.size();
//
//        if( _cov.isContinuous() && _dst.isContinuous() )
//        {
//            size.width *= size.height;
//            size.height = 1;
//        }
//
//        for( i = 0; i < size.height; i++ )
//        {
//            const float* cov = _cov.ptr<float>(i);
//            float* dst = _dst.ptr<float>(i);
//            j = 0;
//
//
//            for( ; j < size.width; j++ )
//            {
//                float a = cov[j*3];
//                float b = cov[j*3+1];
//                float c = cov[j*3+2];
//                 dst[j] = fabs((float)(a*c - b*b - k*(a + c)*(a + c)));
////                dst[j] = (float)(a*c - b*b - k*(a + c)*(a + c));
//            }
//        }
//    }


//    static void
//    cornerEigenValsVecs(const cv::Mat &src, cv::Mat &eigenv, int block_size,
//                        int aperture_size, double k = 0.,
//                        int borderType = cv::BORDER_DEFAULT) {
//
//        int depth = src.depth();
//        double scale = (double) (1 << ((aperture_size > 0 ? aperture_size : 3) - 1)) * block_size;
//        if (aperture_size < 0)
//            scale *= 2.0;
//        if (depth == CV_8U)
//            scale *= 255.0;
//        scale = 1.0 / scale;
//
//        CV_Assert(src.type() == CV_8UC1 || src.type() == CV_32FC1);
//
//        cv::Mat Dx, Dy;
//        if (aperture_size > 0) {
//            cv::Sobel(src, Dx, CV_32F, 1, 0, aperture_size, scale, 0, borderType);
//            cv::Sobel(src, Dy, CV_32F, 0, 1, aperture_size, scale, 0, borderType);
//        } else {
//            cv::Scharr(src, Dx, CV_32F, 1, 0, scale, 0, borderType);
//            cv::Scharr(src, Dy, CV_32F, 0, 1, scale, 0, borderType);
//        }
//
//        cv::Size size = src.size();
//        cv::Mat cov(size, CV_32FC3);
//        int i, j;
//
//        for (i = 0; i < size.height; i++) {
//            float *cov_data = cov.ptr<float>(i);
//            const float *dxdata = Dx.ptr<float>(i);
//            const float *dydata = Dy.ptr<float>(i);
//            j = 0;
//
//
//            for (; j < size.width; j++) {
//                float dx = dxdata[j];
//                float dy = dydata[j];
//
//                cov_data[j * 3] = dx * dx;
//                cov_data[j * 3 + 1] = dx * dy;
//                cov_data[j * 3 + 2] = dy * dy;
//            }
//        }
//
//        cv::boxFilter(cov, cov, cov.depth(), cv::Size(block_size, block_size),
//                      cv::Point(-1, -1), false, borderType);
//
//
//         calcHarris(cov, eigenv, k);
//         //calcEigenValsVecs( cov, eigenv );
//
//    }


//    void cornerHarrisOur(cv::InputArray _src, cv::OutputArray _dst, int blockSize, int ksize, double k, int borderType) {
//
//        cv::Mat src = _src.getMat();
//        _dst.create(src.size(), CV_32FC1);
//        cv::Mat dst = _dst.getMat();
//
//
//        cornerEigenValsVecs(src, dst, blockSize, ksize, k, borderType);
//    }

//    void cornerMinEigenValOur( cv::InputArray _src, cv::OutputArray _dst, int blockSize, int ksize, int borderType )
//    {
//        cv::Mat src = _src.getMat();
//        _dst.create( src.size(), CV_32FC1 );
//        cv::Mat dst = _dst.getMat();
//
//        cornerEigenValsVecs( src, dst, blockSize, ksize, 0, borderType );
//    }