//
// Created by michalnowicki on 29.06.17.
//

#ifndef DSO_CORNEREDGEHARRIS_H
#define DSO_CORNEREDGEHARRIS_H

namespace ORB_SLAM2 {
    void cornerEdgeHarrisExtractor(cv::InputArray _image, cv::OutputArray _corners, cv::OutputArray _lambdasVectors,
                                   int maxCorners, double qualityLevel, double minDistance,
                                   cv::InputArray _mask, int blockSize, double harrisK);
}
#endif //DSO_CORNEREDGEHARRIS_H
