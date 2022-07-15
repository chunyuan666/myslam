#ifndef MYSLAM_OPTIMIZER_H
#define MYSLAM_OPTIMIZER_H
#include "Common.h"
#include "Frame.h"
#include "g2o_types.h"
#include "Map.h"

namespace myslam{
    class Optimizer{
    public:
        static unsigned long PoseOptimization(const Frame::Ptr &Frame);
        static unsigned long OptimizeActivateMap();
    };
}

#endif //MYSLAM_OPTIMIZER_H
