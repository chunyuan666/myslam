#ifndef _BACKEND_H
#define _BACKEND_H

#include "Common.h"
#include "KeyFrame.h"

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
    

public:
    typedef std::shared_ptr<Backend> Ptr;
    
private:
    std::list<KeyFrame::Ptr> mlpNewkeyFrames;
    bool mbNeedOptimize = false;
    KeyFrame::Ptr mCurrentKeyFrame;


};


}

#endif