#ifndef MYSLAM_KEYFRAME_H
#define MYSLAM_KEYFRAME_H

#include "Common.h"
#include "Frame.h"

namespace myslam{

class KeyFrame:public Frame{
public:
    KeyFrame(const Frame::Ptr &frame);

public:
    unsigned long mKeyFrameId;
    static unsigned long nNextId;
    typedef std::shared_ptr<KeyFrame> Ptr;
};

}

#endif