#include "Backend.h"

namespace myslam{

    Backend::Backend(){
        std::thread([this] { Running(); });
    }

    void Backend::Running(){
        while(1){
            if(CheckNewKeyFrame()){
                ProcessKeyFrame();
            }
        }
    }

    void Backend::ProcessKeyFrame(){
        mCurrentKeyFrame = mlpNewkeyFrames.front();
        mlpNewkeyFrames.pop_front();
    }
        

}