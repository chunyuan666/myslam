#ifndef MYSLAM_MAP_H
#define MYSLAM_MAP_H

#include "Common.h"
#include "MapPoints.h"
#include "KeyFrame.h"

namespace myslam{

class Map{
public:
    Map(){}
    /**
     * @brief 插入地图点
    */
    void InserMapPoint(const MapPoints::Ptr &MapPoint);

    /**
     * ＠brief 插入关键帧
    */
    void InserKeyFrame(const KeyFrame::Ptr &kf);

    /**
     * @brief 插入外点的ID
     * 
    */
    void InserOutLier(const unsigned long &map_id){
        mlOutlierMapPoints.push_back(map_id);
    }
    /**
     * @brief
     * */
    void SetNumActivateMap(const int &num){
        mNumActivateMap = num;
    }

    std::map<unsigned long, KeyFrame::Ptr> GetActivateKeyFrames(){
        return mmActivateKeyFrames;
    }

    std::map<unsigned long, KeyFrame::Ptr> GetAllKeyFrames(){
        return mmAllKeyFrames;
    }

    std::map<unsigned long, MapPoints::Ptr> GetActivateMapPoints(){
        return mmActivateMapPoints;
    }

    void CullOldActivateKF();

    void CullOldActivateMapPoint();

    void InserActivateMapPoints(const MapPoints::Ptr &MapPoint);

    void RemoveOutlierMapPoints();


public:
    typedef std::shared_ptr<Map> Ptr;

private:
    //　所有地图点，first:地图点的ID secend:地图点
    std::map<unsigned long, MapPoints::Ptr> mmAllMapPoints;
    // 所有关键帧,first:关键帧的ID secend:关键帧
    std::map<unsigned long, KeyFrame::Ptr> mmAllKeyFrames;
    // 滑动窗口关键帧
    std::map<unsigned long, KeyFrame::Ptr> mmActivateKeyFrames;
    // 滑动窗口地图点
    std::map<unsigned long, MapPoints::Ptr> mmActivateMapPoints;
    // 地图中的外点　由估计位姿时产生
    std::list<unsigned long> mlOutlierMapPoints;

    int mNumActivateMap;

    KeyFrame::Ptr mpCurrentKeyFrame;
};


}

#endif