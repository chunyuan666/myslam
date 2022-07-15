#include "Frontend.h"
#include "Common.h"
#include "g2o_types.h"

namespace myslam {


    bool Frontend::Running(const cv::Mat &leftImg, const cv::Mat &rightImg, const double timestamp) {
        mCurrentFrame = std::make_shared<Frame>(leftImg, rightImg, timestamp, mCameraLeft->GetK());
        /***********TODO*****************/
        switch (mStatus) {
            case TrackStatus::INIT:
                if(StereoInit()){
                    mStatus = TrackStatus::GOOD;
                }
                break;
            case TrackStatus::GOOD:
            case TrackStatus::BAD:
                Tracking();
                break;
            case TrackStatus::LOST:
                /**TODO*/
                break;
        }

        mLastFrame = mCurrentFrame;
        return true;
    }

    /**
     * @brief 双目初始化
     * @details 
     * 1. 提取左目图像的特征点
    */
    bool Frontend::StereoInit() {
        // 提取左目图像的特征点
        DetectFeature();
        // 匹配右图特征点
        unsigned long cnt = MatchFeaturesInRight();
        if(cnt < mnFeaturesInitGood )
            return false;
        //　初始化地图
        unsigned long num_maps = MapInit();
        if( num_maps < mnFeaturesTrackingBad )
            return false;
        // 将初始化帧作为关键帧插入
        InserKeyFrame();
        return true;
    }

    /**
     * @brief 提取特征点
    */
    unsigned long Frontend::DetectFeature() {
        // 制作掩膜
        cv::Mat mask(mCurrentFrame->mImgLeft.size(), CV_8UC1, 255);
        for (auto &feature: mCurrentFrame->mvpFeatureLeft) {
            cv::rectangle(mask, feature->mKeyPoint.pt - cv::Point2f(20, 20),
                          feature->mKeyPoint.pt + cv::Point2f(20, 20), 0, CV_FILLED);
        }

        std::vector<cv::KeyPoint> KeyPoints;
        if (mStatus == TrackStatus::INIT) {
            mORBExtractorInit->Detect(mCurrentFrame->mImgLeft, mask, KeyPoints);
        } else {
            mORBExtractor->Detect(mCurrentFrame->mImgLeft, mask, KeyPoints);
        }
        /*
        cv::Mat outimg;
        cv::drawKeypoints(mCurrentFrame->mImgLeft, KeyPoints, outimg,
                          cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
        LOG(INFO) << "sum OBR: " << KeyPoints.size();
        cv::imshow("ORB", outimg);
        cv::waitKey(0);
        cv::destroyAllWindows();
        */
        unsigned long cnt = 0;
        for (auto &KeyPoint: KeyPoints) {
            Feature::Ptr feature = std::make_shared<Feature>(KeyPoint);
            mCurrentFrame->mvpFeatureLeft.push_back(feature);
            cnt++;
        }
        return cnt;
    }

    unsigned long Frontend::MatchFeaturesInRight() {
        std::vector<cv::Point2f> vPointFeaLeft, vPointFeaRight;
        vPointFeaLeft.reserve(mCurrentFrame->mvpFeatureLeft.size());
        vPointFeaRight.reserve(mCurrentFrame->mvpFeatureLeft.size());
        // 准备左右目的特征点
        for (auto &feature: mCurrentFrame->mvpFeatureLeft) {
            vPointFeaLeft.push_back(feature->mKeyPoint.pt);
            auto map = feature->mpMapPoint.lock();
            if (map) {
                cv::Point2f PointFeaRight = mCameraRight->World2Pixel(map->GetPose());
                vPointFeaRight.push_back(PointFeaRight);
            } else {
                vPointFeaRight.push_back(feature->mKeyPoint.pt);
            }
        }
        //　利用光流匹配右目
        std::vector<uchar> status;
        cv::Mat error;
        cv::calcOpticalFlowPyrLK(mCurrentFrame->mImgLeft, mCurrentFrame->mImgRight,
                                 vPointFeaLeft, vPointFeaRight, status, error, cv::Size(11, 11),
                                 3, cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
                                 cv::OPTFLOW_USE_INITIAL_FLOW);
        unsigned long nGoodPoints = 0;
        for (int i = 0; i < status.size(); i++) {
            if (status[i]) {
                cv::KeyPoint new_kp(vPointFeaRight[i], 7);
                auto feature = std::make_shared<Feature>(new_kp);
                mCurrentFrame->mvpFeatureRight.push_back(feature);
                nGoodPoints++;
            } else {
                mCurrentFrame->mvpFeatureRight.push_back(nullptr);
            }
        }
        return nGoodPoints;
    }

    //　初始化地图
    unsigned long  Frontend::MapInit() {
        // 创造新的地图点
        auto nGoodPoints = CrateNewMapPoints();
        // 设第一帧的位姿为[R|t] = [I | 0]
        mCurrentFrame->SetPose(SE3_Identity);
        return nGoodPoints;
    }                                                                                                                                   

    // 恒速运动模式追踪
    bool Frontend::Tracking(){
        mCurrentFrame->SetPose( mMotion * mLastFrame->GetPose());

        // 匹配特征点,用光流法匹配，追踪上一帧和当前帧
        MatchLastFrameByLKFlow();
        //　估计当前帧位姿
        unsigned long numTrackingInliers = EstimatePose();
        
        if( numTrackingInliers >= mnFeaturesInitGood ){
            mStatus = TrackStatus::GOOD;
        }else if( numTrackingInliers >= mnFeaturesTrackingBad && numTrackingInliers < mnFeaturesTrackingGood){
            mStatus = TrackStatus::BAD;
        }else{
            mStatus = TrackStatus::LOST;
        }
        mMotion = mCurrentFrame->GetPose() * mLastFrame->GetPose().inverse();

        if(mStatus == TrackStatus::BAD){
            // 查找新的特征点
            DetectFeature();
            // 匹配右目特征点
            MatchFeaturesInRight();
            //　三角化新的地图点
            CrateNewMapPoints();
            //　插入关键帧
            InserKeyFrame();
        }
    }

    // 创造新的地图点
    unsigned long Frontend::CrateNewMapPoints(){
        // 1. 三角化形成地图点
        cv::KeyPoint Kp1, Kp2;
        unsigned long nGoodPoints = 0;
        for(unsigned long i = 0; i < mCurrentFrame->mvpFeatureLeft.size(); i++){
            if(!mCurrentFrame->mvpFeatureRight[i])
                continue;
            auto map = mCurrentFrame->mvpFeatureLeft[i]->mpMapPoint;
            if(!map.expired())  // 该特征点已有地图点，不再添加
                continue;
            Kp1 = mCurrentFrame->mvpFeatureLeft[i]->mKeyPoint;
            Kp2 = mCurrentFrame->mvpFeatureRight[i]->mKeyPoint;
            SE3 p1 = mCameraLeft->GetCameraPose();
            SE3 p2 = mCameraRight->GetCameraPose();
            Vector3d X3D;
            Triangulation(p1, p2, Kp1, Kp2, X3D);
            // LOG(INFO) << "X3D: \n" << X3D;
            if(X3D[2] < 0)
                continue;
            nGoodPoints++;
            // 2. 创建新的地图点
            MapPoints::Ptr NewPoint =  std::make_shared<MapPoints>(X3D);
            mCurrentFrame->mvpFeatureLeft[i]->mpMapPoint = NewPoint;
            mCurrentFrame->mvpFeatureRight[i]->mpMapPoint = NewPoint;
            NewPoint->AddObservation(mCurrentFrame->mFrameId, mCurrentFrame->mvpFeatureLeft[i]);
            mMap->InserMapPoint(NewPoint);
        }
        return nGoodPoints;
    }

    //　估计当前帧位姿
    unsigned long Frontend::EstimatePose(){
        // 计算位姿
        auto InLiers = Optimizer::PoseOptimization(mCurrentFrame);
        //　剔除外点
        for(auto &fea : mCurrentFrame->mvpFeatureLeft){
            if(!fea->IsOutLier)
                continue;
            auto mapPoint = fea->mpMapPoint.lock();
            // 
            if(mapPoint){
                mapPoint->mbIsOutlier = true;
                mMap->InserOutLier(mapPoint->mid); //插入地图中的外点，等到局部优化时再删除
                mapPoint.reset();
            }
            fea->IsOutLier = false;
        }
        return InLiers;
    }

    // 光流匹配
    unsigned long Frontend::MatchLastFrameByLKFlow(){
        std::vector<cv::Point2f> vFeaPointsLast, vFeaPointsCurrent;
        for(auto &fea : mLastFrame->mvpFeatureLeft){
            vFeaPointsLast.push_back(fea->mKeyPoint.pt);
            auto mapPoint = fea->mpMapPoint.lock();
            if(mapPoint && !mapPoint->mbIsOutlier){
                cv::Point2f pointCurrent = mCameraLeft->World2Pixel(mapPoint->GetPose(),mCurrentFrame->GetPose());
                vFeaPointsCurrent.push_back(pointCurrent);
            }else{
                vFeaPointsCurrent.push_back(fea->mKeyPoint.pt);
            }
        }
        std::vector<uchar> status;
        cv::Mat error;
        cv::calcOpticalFlowPyrLK(mLastFrame->mImgLeft, mCurrentFrame->mImgLeft,
                                 vFeaPointsLast, vFeaPointsCurrent, status, error, cv::Size(11, 11),
                                 3, cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
                                 cv::OPTFLOW_USE_INITIAL_FLOW);
        unsigned long nGoodPoints = 0;
        for (int i = 0; i < status.size(); i++) {
            if (status[i]) {
                cv::KeyPoint new_kp(vFeaPointsCurrent[i], 7);
                auto feature = std::make_shared<Feature>(new_kp);
                mCurrentFrame->mvpFeatureLeft.push_back(feature);
                nGoodPoints++;
            }
        }
        return nGoodPoints;
    }

    
    /**
     * ＠brief 三角化函数
     * ＠param[in] 相机1的投影矩阵
     * ＠param[in] 相机2的投影矩阵
     * ＠param[in] 图像1的特征点
     * ＠param[in] 图像2的特征点
     * ＠param[out] 三角化后地图点的坐标
     * */
    void Frontend::Triangulation(const SE3 &P1, const SE3 &P2,
                                 const cv::KeyPoint &Kp1, const cv::KeyPoint &Kp2, 
                                 Vector3d &X3D) {
        cv::Mat A(4, 4, CV_32F);
        cv::Mat p1, p2;
        cv::eigen2cv(P1.matrix3x4(), p1);
        cv::eigen2cv(P2.matrix3x4(), p2);
        static_cast<cv::Mat>(Kp1.pt.x * p1.row(2) - p1.row(0)).copyTo(A.row(0));
        static_cast<cv::Mat>(Kp1.pt.y * p1.row(2) - p1.row(1)).copyTo(A.row(1));
        static_cast<cv::Mat>(Kp2.pt.x * p2.row(2) - p2.row(0)).copyTo(A.row(2));
        static_cast<cv::Mat>(Kp2.pt.y * p2.row(2) - p2.row(1)).copyTo(A.row(3));
        //奇异值分解的结果
        cv::Mat u,w,vt;
        //对系数矩阵A进行奇异值分解
        cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);
        //根据前面的结论，奇异值分解右矩阵的最后一行其实就是解，原理类似于前面的求最小二乘解，四个未知数四个方程正好正定
        //别忘了我们更习惯用列向量来表示一个点的空间坐标
        cv::Mat x3D = vt.row(3).t();
        //为了符合其次坐标的形式，使最后一维为1
        x3D = x3D.rowRange(0,3)/x3D.at<float>(3);
        cv::cv2eigen(x3D, X3D);
    }

    // 检查是否插入关键帧
    bool Frontend::CheckIsInserKF(){
        /***TODO**/
    }

    //　插入关键帧
    bool Frontend::InserKeyFrame(){
        KeyFrame::Ptr newKeyFrame = std::make_shared<KeyFrame>(mCurrentFrame);
        mBackend->InserKeyFrame(newKeyFrame);// 插入后端
        return true;
    }
}