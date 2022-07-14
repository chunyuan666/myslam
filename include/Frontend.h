#ifndef _FRONTEND_H
#define _FRONTEND_H

#include <utility>

#include "Common.h"
#include "Backend.h"
#include "Frame.h"
#include "ORBExtractor.h"
#include "Camera.h"
#include "MapPoints.h"
#include "Map.h"
#include "KeyFrame.h"
#include "Optimizer.h"

namespace myslam{

// 跟踪状态
enum class TrackStatus {INIT, GOOD, BAD, LOST};


class Frontend{

public:

    /**
     * @ 构造函数
    */
    Frontend()= default;

    /**
     * @brief 前端主线程
    */
   bool Running(const cv::Mat &leftImg, const cv::Mat &rightImg, const double timestamp);

   /**
    * @brief 设置跟踪参数状态
   */
    void SetTrackingPara(const int &initGood, const int &trackingGood, const int &trackingBad){
        mnFeaturesTrackingGood = trackingGood;
        mnFeaturesTrackingBad=trackingBad;
        mnFeaturesInitGood=initGood;
    }

   /**
    * @brief 返回系统的跟踪状态
   */
   TrackStatus GetStatus(){
       return mStatus;
   }

   /**
    * @brief 初始状态
    *        
   */
    bool StereoInit();

    /**
     * @brief 提取右图特征点
     */
    unsigned long MatchFeaturesInRight();

    /**
     * @brief 提取特征点
    */
    unsigned long DetectFeature();

    /**
     * @brief 设置特征提取器
    */
    void SetORBExtractor(const ORBExtractor::Ptr& orb_extor_init,const ORBExtractor::Ptr& orb_extor){
        mORBExtractor = orb_extor;
        mORBExtractorInit = orb_extor_init;
    }

    /**
     * @brief 设置相机
     * */
    void SetCamera(const Camera::Ptr &cam_left, const Camera::Ptr &cam_right){
        mCameraLeft = cam_left;
        mCameraRight = cam_right;
    }

    /**
     * @beirf 设置地图
    */
    void SetMap(const Map::Ptr &map){
        mMap = map;
    }

    /**
     * ＠brief 初始化地图
     * */
     unsigned long  MapInit();

    /**
     * ＠brief 三角化函数
     * ＠param[in] 相机1的投影矩阵
     * ＠param[in] 相机2的投影矩阵
     * ＠param[in] 图像1的特征点
     * ＠param[in] 图像2的特征点
     * ＠param[out] 三角化后地图点的坐标
     * */
    static void Triangulation(const SE3 &P1, const SE3 &P2,
                       const cv::KeyPoint &Kp1, const cv::KeyPoint &Kp2, Vector3d &X3D);

    /**
     * @brief 检查是否需要插入关键帧
    */
    bool CheckIsInserKF();

    /**
     * @brief 插入关键帧
     * 
    */
    bool InserKeyFrame();

    /**
     * @brief　追踪函数
    */
    bool Tracking();

    /**
     * @brief 光流匹配
     * 
    */
    unsigned long MatchLastFrameByLKFlow();

    /**
     * @brief 估计当前帧位姿
    */
    unsigned long EstimatePose();

    /**
     * @brief 创建新的地图点
    */
    unsigned long CrateNewMapPoints();


public:
    typedef std::shared_ptr<Frontend> Ptr;
    Backend::Ptr mBackend;
    Frame::Ptr mCurrentFrame;
    Frame::Ptr mLastFrame;
    ORBExtractor::Ptr mORBExtractor, mORBExtractorInit;
    Camera::Ptr mCameraLeft, mCameraRight;
    Map::Ptr mMap;
    

private:
    TrackStatus mStatus = TrackStatus::INIT; 
     // params for deciding the tracking status
    int mnFeaturesTrackingGood;
    int mnFeaturesTrackingBad;
    int mnFeaturesInitGood;

    SE3 mMotion; 


};

}

#endif