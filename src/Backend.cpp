#include "Backend.h"
#include "KeyFrame.h"
#include "Map.h"

namespace myslam{

    Backend::Backend(){
        mbNeedOptimize = false;
        std::thread([this] { Running(); });
    }

    void Backend::Running(){
        while(mbNeedOptimize){
            while(CheckNewKeyFrame()){
                ProcessKeyFrame();
            }
            //　局部优化位姿和地图
            // Optimizer::OptimizeActivateMap(mMap);
        }
    }

    void Backend::ProcessKeyFrame(){
        mCurrentKeyFrame = mlpNewkeyFrames.front();
        mlpNewkeyFrames.pop_front();
        mMap->InserKeyFrame(mCurrentKeyFrame);
    }
}