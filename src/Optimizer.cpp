#include "Optimizer.h"

namespace myslam{
    unsigned long Optimizer::PoseOptimization(const Frame::Ptr &Frame) {
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
        edges.reserve(Frame->mvpFeatureLeft.size());
        features.reserve(Frame->mvpFeatureLeft.size());
        for(auto &fea : Frame->mvpFeatureLeft){
            auto map = fea->mpMapPoint.lock();
            if(map && !map->mbIsOutlier){
                LOG(INFO) << "begin";
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
                if(iter>=iterations/2)
                    e->setRobustKernel(nullptr);
            }
        }
        // 5.计算位姿
        Frame->SetPose(vertex->estimate());
        return edges.size()-nOutLiers;
    }
}