#ifndef MYSLAM_FRAME_H
#define MYSLAM_FRAME_H

#include "Common.h"
#include "Feature.h"

namespace myslam{

class Frame{
public:
    Frame()=default;
    Frame(const cv::Mat &imgleft, const cv::Mat &imgright, const double &timestamp,
          const Mat33d &k);
    void SetPose(const SE3 &pose){
        mPose = pose;
    }

    SE3 GetPose(){
        return mPose;
    }

public:
    typedef std::shared_ptr<Frame> Ptr;
    unsigned long mFrameId;
    static unsigned long nLastId;
    cv::Mat mImgLeft, mImgRight;
    SE3 mPose;
    double mTimeStamp;
    std::vector<Feature::Ptr> mvpFeatureLeft, mvpFeatureRight;
    Mat33d mK;
};

}
#endif