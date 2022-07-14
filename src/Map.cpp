#include "Common.h"
#include "Map.h"

namespace myslam{
    
    void Map::InserMapPoint(const MapPoints::Ptr &MapPoint){
        //还没有存在地图中
        if(mmAllMapPoints.find(MapPoint->mid) == mmAllMapPoints.end()){
            mmAllMapPoints.insert({MapPoint->mid, MapPoint});
        }else{  // 在地图里了
            mmAllMapPoints[MapPoint->mid] = MapPoint;
        }
    }

    void Map::InserKeyFrame(const KeyFrame::Ptr &kf){
        //还没有存在地图中
        if(mmAllKeyFrames.find(kf->mKeyFrameId) == mmAllKeyFrames.end()){
            mmAllKeyFrames.insert(std::make_pair(kf->mKeyFrameId, kf));
            mmActivateKeyFrames.insert(std::make_pair(kf->mKeyFrameId, kf));
        }else{ // 在地图里了
            mmAllKeyFrames[kf->mKeyFrameId] = kf;
            mmActivateKeyFrames[kf->mKeyFrameId] = kf;
        }

        //　插入地图点
        for(auto &fea : kf->mvpFeatureLeft){
            if(mmAllMapPoints.find())
        }

        // 如果活跃帧大于阈值7．则删除一些活跃帧
        if(mmActivateKeyFrames.size() > mNumActivateMap){
            CullActivateKF();
        }
    }

    void Map::CullActivateKF(){
        
    }

}