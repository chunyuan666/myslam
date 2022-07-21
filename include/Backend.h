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

    void InsertKeyFrame(const KeyFrame::Ptr &kf){
        std::unique_lock<std::mutex> lock(mMutexNewKFs);
        mlpNewkeyFrames.push_back(kf);
        mbNeedOptimize = true;
    }

    bool CheckNewKeyFrame(){
        std::unique_lock<std::mutex> lock(mMutexNewKFs);
        return (!mlpNewkeyFrames.empty());
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
    bool mbNeedOptimize = false;
    
private:
    std::list<KeyFrame::Ptr> mlpNewkeyFrames;
    KeyFrame::Ptr mCurrentKeyFrame;
    Camera::Ptr mCameraLeft;
    Map::Ptr mMap;

    std::atomic<bool> mbBackendIsRunning;

    std::mutex mMutexNewKFs;

    std::thread mBackendHandle;
};


}

#endif