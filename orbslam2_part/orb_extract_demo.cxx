#include<opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgcodecs/imgcodecs_c.h>
#include <cv.hpp>
//#include <opencv2/features2d/features2d.hpp>
#include "ORBextractor.h"

#define EDGE_THRESHOLD 80

int main(int argc, char * argv[])
{
    //int nfeatures, float scaleFactor, int nlevels,
    //                 int iniThFAST, int minThFAST
    ORB_SLAM2::ORBextractor extractor(2000, 1.2, 8, 20, 7);
    std::vector<cv::KeyPoint> mvKeys;
    cv::Mat mDescriptors;
    auto im = cv::imread("/media/wegatron/data/data/tmu/rgbd_dataset_freiburg1_desk/rgb/1305031469.159703.png", CV_LOAD_IMAGE_UNCHANGED);
    cvtColor(im,im,CV_RGB2GRAY);
//    cv::ORB::create()
//    cv::Size sz(im.cols, im.rows);
//    cv::Size wholeSize(sz.width + EDGE_THRESHOLD*2, sz.height + EDGE_THRESHOLD*2);
//    cv::Mat temp(wholeSize, im.type()), masktemp;
//    cv::Mat mvImagePyramid = temp(cv::Rect(EDGE_THRESHOLD, EDGE_THRESHOLD, sz.width, sz.height));
//    copyMakeBorder(im, temp, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD,
//                   cv::BORDER_REFLECT_101+cv::BORDER_ISOLATED);

    extractor(im,cv::Mat(),mvKeys,mDescriptors);
//    cv::imshow("img", mvImagePyramid);
//    cv::waitKey(0);
    return 0;
}