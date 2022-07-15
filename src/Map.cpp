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

    void Map::InserActivateMapPoints(const MapPoints::Ptr &MapPoint){
        //还没有存在地图中
        if(mmActivateMapPoints.find(MapPoint->mid) == mmAllMapPoints.end()){
            mmActivateMapPoints.insert({MapPoint->mid, MapPoint});
        }else{  // 在地图里了
            mmActivateMapPoints[MapPoint->mid] = MapPoint;
        }
    }

    void Map::InserKeyFrame(const KeyFrame::Ptr &kf){
        mpCurrentKeyFrame = kf;
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
            auto map = fea->mpMapPoint.lock();
            if(map){
                map->AddActivateObservation(kf->mKeyFrameId, fea);
                InserActivateMapPoints(map);
            }
        }

        // 如果活跃帧大于阈值7．则删除一些活跃帧
        if(mmActivateKeyFrames.size() > mNumActivateMap){
            CullOldActivateKF();
            CullOldActivateMapPoint();
        }
    }

    void Map::CullOldActivateKF(){
        if(!mpCurrentKeyFrame)
            return;
        auto T_wc = mpCurrentKeyFrame->GetPose().inverse();
        double maxDistance = 0; unsigned long maxID = 0;
        double minDistance = 999.9; unsigned long minID = 0;
        for(auto &kf : mmActivateKeyFrames){
            if(kf.first == mpCurrentKeyFrame->mKeyFrameId)
                continue;
            auto distance = (kf.second->GetPose() * T_wc).log().norm();
            if(distance > maxDistance){
                maxDistance = distance;
                maxID = kf.first;
            }else if(distance < minDistance){
                minDistance = distance;
                minID = kf.first;
            }
        }
        unsigned long toDelId;
        double min_th = 0.2;
        if(minDistance < min_th){
            toDelId = minID;
        }else{
            toDelId = maxID;
        }
        // 取消地图点对该帧的观测
        for(auto &feat : mmActivateKeyFrames[toDelId]->mvpFeatureLeft){
            auto map = feat->mpMapPoint.lock();
            if(map){
                map->RemoveActiveObservation(toDelId, feat);
            }
        }
        // 删除关键帧
        mmActivateKeyFrames.erase(toDelId);
    }
    //　删除观测为0的局部地图点
    void Map::CullOldActivateMapPoint() {
        for(auto iter = mmActivateMapPoints.begin(); iter!=mmActivateMapPoints.end();){
            if(iter->second->GetActivateObsCnt() <= 0){
                mmActivateMapPoints.erase(iter);
            }else{
                iter++;
            }
        }
    }

}