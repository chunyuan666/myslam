//
// Created by yuan on 2022/7/8.
//

#ifndef MYSLAM_MAPPOINTS_H
#define MYSLAM_MAPPOINTS_H

#include "Common.h"
//#include "Frame.h"


namespace myslam{
    class MapPoints {
    public:

        MapPoints(){
            mid = nLastId++;
        }

        MapPoints(const Vector3d& pose){
            mCoordinate = pose;
            mid = nLastId++;
        }

        void SetPose(const Vector3d &pose){
            mCoordinate = pose;
        }

        Vector3d GetPose(){
            return mCoordinate;
        }

        
        

    public:
        typedef std::shared_ptr<MapPoints> Ptr;
        unsigned long mid;
        static unsigned long nLastId;

        //　估计位姿时生成判断该点是否为异常点
        bool mbIsOutlier = false;
    private:
        Vector3d mCoordinate = Vector3d::Zero();
    };
}



#endif //MYSLAM_MAPPOINTS_H
