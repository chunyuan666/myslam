#include "Optimizer.h"

namespace myslam{
    unsigned long Optimizer::PoseOptimization(Frame::Ptr &Frame) {
        // 用G2O来估计位姿
        // Step 1：构造g2o优化器, BlockSolver_6_3表示：位姿 _PoseDim 为6维，路标点 _LandmarkDim 是3维
        typedef g2o::BlockSolver_6_3 BlockSolverType;
        typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;
        auto solver = new g2o::OptimizationAlgorithmLevenberg(
                g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);

        //　2.添加顶点
        auto *vertex = new VertexPose();
        vertex->setId(0);
        vertex->setEstimate(Frame->GetPose());
        optimizer.addVertex(vertex);

        //　3.添加边
        int index = 1;
        std::vector<EdgeProjectionPoseOnly *> edges;
        std::vector<Feature::Ptr> features;
        //edges.reserve(Frame->mvpFeatureLeft.size());
        //features.reserve(Frame->mvpFeatureLeft.size());
        for(auto &fea : Frame->mvpFeatureLeft){
            auto map = fea->mpMapPoint.lock();
            if(map && !map->mbIsOutlier){
                auto *edge = new EdgeProjectionPoseOnly(map->GetPose(), Frame->mK);
                edge->setId(index);
                edge->setVertex(0, vertex);
                edge->setMeasurement(cvPoint2Vec2(fea->mKeyPoint.pt));
                edge->setInformation(Eigen::Matrix2d::Identity());
                edge->setRobustKernel(new g2o::RobustKernelHuber);
                features.push_back(fea);
                edges.push_back(edge);
                optimizer.addEdge(edge);
                index++;
            }
        }
        //　4.迭代
        double chi2_th = 5.991; //95%的置信区间
        int nOutLiers = 0;
        int iterations = 4;
        for(int iter = 0; iter < iterations; iter++){
            optimizer.initializeOptimization(0);
            optimizer.optimize(10);
            nOutLiers = 0;
            for(int i = 0; i < edges.size(); i++) {
                auto e = edges[i];
                auto fea = features[i];
                double chi2 = e->chi2();
                // 如果这条误差边是来自于outlier
                if (fea->IsOutLier) {
                    e->computeError();
                }
                // 该点不可靠
                if (chi2 > chi2_th) {
                    fea->IsOutLier = true;
                    e->setLevel(1); //下次不优化
                    nOutLiers++;
                } else {
                    fea->IsOutLier = false;
                    e->setLevel(0);
                }
                if(iter==iterations/2)
                    e->setRobustKernel(nullptr);
            }
        }
        // 5.计算位姿
        Frame->SetPose(vertex->estimate());
        //LOG(INFO) << "\n" << "第" << Frame->mFrameId << "帧" << "估计后位姿：\n" << vertex->estimate().matrix();
        return edges.size()-nOutLiers;
    }

    unsigned long Optimizer::OptimizeActivateMap(Map::Ptr &Map, const Camera::Ptr &camera) {
        typedef g2o::BlockSolver_6_3 BlockSolverType;
        typedef g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType> LinearSolverType;
        auto solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<BlockSolverType>(
        g2o::make_unique<LinearSolverType>()));
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);

        auto KFs = Map->GetActivateKeyFrames();
        auto MPs = Map->GetActivateMapPoints();

        for(auto &kf : KFs){
            LOG(INFO) << "actiKFs里的帧id: " << kf.first << ", ";
        }

        std::map<unsigned long, VertexPose *> Vertex_KFs;
        std::map<unsigned long, VertexXYZ *> Vertex_Mps;

        // 增加顶点，相机位姿
        int maxKFid = 0;
        for(auto &Keyframe : KFs){
            auto kf = Keyframe.second;
            auto vertex_pose = new VertexPose();
            vertex_pose->setId(static_cast<int>(kf->mKeyFrameId));
            //LOG(INFO) << "\nkf_pose: \n" << kf->GetPose().matrix();
            vertex_pose->setEstimate(kf->GetPose());
            optimizer.addVertex(vertex_pose);
            if(kf->mKeyFrameId > maxKFid){
                maxKFid = static_cast<int>(kf->mKeyFrameId);
            }
            Vertex_KFs.insert(std::make_pair(kf->mKeyFrameId, vertex_pose));
        }

        int index = 1;
        double chi2_th = 7.851;
        std::map<Feature::Ptr, EdgeProjection *> FeatsAndEdges;
        // 增加顶点，地图点的坐标
        for(auto &MapPoint : MPs){
            auto mp_id = MapPoint.first;
            auto mp = MapPoint.second;
            if(mp==nullptr || mp->mbIsOutlier)
                continue;
            auto vertex_XYZ = new VertexXYZ();
            vertex_XYZ->setId(static_cast<int>(maxKFid +1 + mp->mid));
            //LOG(INFO) << "\nmp_pose: \n" << mp->GetPose().matrix();
            vertex_XYZ->setEstimate(mp->GetPose());
            vertex_XYZ->setMarginalized(true);

            optimizer.addVertex(vertex_XYZ);
            Vertex_Mps.insert(std::make_pair(mp->mid, vertex_XYZ));
            auto observations = mp->GetObservation();
            for(auto &obs : observations){
                auto kfId = obs.first;
                auto feat = obs.second.lock();
                if(feat==nullptr || feat->IsOutLier)
                    continue;
                //assert(KFs.find(kfId) != KFs.end());
                if(KFs.find(kfId) == KFs.end()) {
                    LOG(INFO) << "第 "<<kfId<<"帧不在actiKFs里"; 
                }
                auto *e = new EdgeProjection(camera->GetK(), camera->GetCameraPose());
                e->setId(index);
                e->setVertex(0, Vertex_KFs[kfId]);
                e->setVertex(1, Vertex_Mps[mp_id]);
                e->setMeasurement(cvPoint2Vec2(feat->mKeyPoint.pt));
                e->setInformation(Mat22d::Identity());
                auto rk = new g2o::RobustKernelHuber();
                rk->setDelta(sqrt(chi2_th));
                e->setRobustKernel(rk);
                optimizer.addEdge(e);
                index++;
                FeatsAndEdges.insert(std::make_pair(feat, e));
            }
        }

/*        for(auto &MapPoint : MPs){
            auto mp_id = MapPoint.first;
            auto mp = MapPoint.second;
            if(mp==nullptr || mp->mbIsOutlier) continue;
            
            auto observations = mp->GetObservation();
            for(auto &obs : observations){
                auto kfId = obs.first;
                auto feat = obs.second.lock();
                if(feat==nullptr) continue;

            }
        }
*/
        int cntOutlier = 0, cntInlier = 0;

        optimizer.initializeOptimization(0);
        optimizer.optimize(5);

        for(auto &fe : FeatsAndEdges){
            auto e = fe.second;
            double chi2 = e->chi2();
            if(e->chi2() > chi2_th){
                e->setLevel(1);
            }else{
                e->setLevel(0);
            }
             e->setRobustKernel(0);
        }

        optimizer.initializeOptimization(0);
        optimizer.optimize(5);

        for(auto &fe : FeatsAndEdges){
            auto feat = fe.first;
            auto e = fe.second;
            double chi2 = e->chi2();
            if(e->chi2() > chi2_th){
                cntOutlier ++;
                feat->IsOutLier = true;
            }else{
                cntInlier++;
                feat->IsOutLier = false;
            }
        }
                
        //LOG(INFO) << "OUTLIERS nums is:  " << cntOutlier;
       // LOG(INFO) << "INLIERS nums is:  " << cntInlier;
        // 处理外点
        // 遍历当前边和特征点
        for(auto &fe : FeatsAndEdges){
            // 找出外点的特征
            auto feat = fe.first;
            auto mp = feat->mpMapPoint.lock();
            if(feat->IsOutLier){
                // 取消Feature对该点的观测
                //mp->RemoveActiveObservation(feat);
                mp->RemoveObservation(feat);
                // 释放该地图点的指针,feat不再持有该地图点的指针
                feat->mpMapPoint.reset();
                /*
                if(mp->GetObsCnt()==0){ //该点已经没有观测
                    mp->mbIsOutlier = true; //标记为外点
                }
                */
            }
        }
        //设置当前帧的位姿
        for(auto &v : Vertex_KFs){
            KFs[v.first]->SetPose(v.second->estimate());
            LOG(INFO) << "矫正后的位姿： id: " << KFs[v.first]->mFrameId << "\n" << KFs[v.first]->GetPose().matrix3x4(); 
        }
        for(auto &m : Vertex_Mps){
             MPs[m.first]->SetPose(m.second->estimate());
        }
           

        //删除外点
        //Map->RemoveOutlierMapPoints();
        Map->CullOldActivateMapPoint();
    }
}










