#include "IOWrapper/ImageRW.h"
#include "util/DatasetReader.h"

#include <opencv2/opencv.hpp>

using namespace dso;

int main(int argc, char *argv[]) {
    ImageFolderReader reader(argv[1], argv[2], argv[3], argv[4]);
    for (int i = 0; i < reader.getNumImages(); ++i) {
        ImageAndExposure* output = reader.getImage(i);
        MinimalImage<float> outImage(output->w, output->h, output->image);
        char fname[256];
        sprintf(fname, "%06d.png", i);
        IOWrap::writeImage(fname, &outImage);
    }

    return 0;
}
