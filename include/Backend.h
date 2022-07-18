#ifndef _BACKEND_H
#define _BACKEND_H

#include "Common.h"
#include "KeyFrame.h"
#include "Map.h"
#include "Optimizer.h"


namespace myslam{
 

class Backend{
public:
    Backend();
    void Running();

    void InserKeyFrame(const KeyFrame::Ptr &kf){
        mlpNewkeyFrames.push_back(kf);
        mbNeedOptimize = true;
    }

    bool CheckNewKeyFrame(){
        if(!mlpNewkeyFrames.empty()){
            return true;
        }
        return false;
    }

    void ProcessKeyFrame();

    void SetMap(const Map::Ptr &map){
        mMap = map;
    }

    void StopBackend(){
        mbNeedOptimize = false;
        mbBackendIsRunning = false;
    }

    void SetCamera(const Camera::Ptr &cam){
        mCameraLeft = cam;
    }
    

public:
    typedef std::shared_ptr<Backend> Ptr;
    
private:
    std::list<KeyFrame::Ptr> mlpNewkeyFrames;
    bool mbNeedOptimize = false;
    KeyFrame::Ptr mCurrentKeyFrame;
    Camera::Ptr mCameraLeft;
    Map::Ptr mMap;

    std::atomic<bool> mbBackendIsRunning{};


};


}

#endif