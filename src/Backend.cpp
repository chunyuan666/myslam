#include "Backend.h"
#include "KeyFrame.h"
#include "Map.h"

namespace myslam{

    Backend::Backend(){
        mbNeedOptimize = false;
        mbBackendIsRunning.store(true);
        std::thread([this] { Running(); });
    }


    void Backend::Running(){
        while(mbBackendIsRunning.load()){
            while(CheckNewKeyFrame()){
                ProcessKeyFrame();
            }
            //　局部优化位姿和地图
            if(mbNeedOptimize){
                Optimizer::OptimizeActivateMap(mMap, mCameraLeft);
                mbNeedOptimize = false;
            }

        }
    }

    void Backend::ProcessKeyFrame(){
        mCurrentKeyFrame = mlpNewkeyFrames.front();
        mlpNewkeyFrames.pop_front();
        mMap->InserKeyFrame(mCurrentKeyFrame);
    }
}