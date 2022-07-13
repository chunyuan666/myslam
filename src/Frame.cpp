#include "Frame.h"
#include "Common.h"

namespace myslam{
    unsigned long Frame::nLastId = 0;

    Frame::Frame(const cv::Mat &imgleft, const cv::Mat &imgright, const double &timestamp,
                 const Mat33d &k){
        mImgLeft = imgleft.clone();
        mImgRight = imgright.clone();
        mTimeStamp = timestamp;
        mPose = SE3_Identity;
        mK = k;
        mFrameId = nLastId++;
    }


}