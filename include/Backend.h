<<<<<<< HEAD
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

    void Stop(){
        mbBackendIsRunning.store(false);
        mBackendHandle.join();
    }

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

=======
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

>>>>>>> 739e668a32adf562c3226cf7614b4d82d1b4d69f
#endif