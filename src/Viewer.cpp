#include "Common.h"
#include "Viewer.h"

namespace myslam{
    

    void Viewer::LoopRunning(){
        pangolin::CreateWindowAndBind("MYSLAM", 1024,768);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
        pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",true,true);
        pangolin::Var<bool> menuShowPoints("menu.Show Points",true,true);
        pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames",true,true);

        pangolin::OpenGlRenderState vis_camera(
        pangolin::ProjectionMatrix(1024, 768, mViewpointF, mViewpointF, 512, 389, 0.1, 1000),
        pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0, 0, 0, 0.0, -1.0, 0.0));

        pangolin::View& vis_display = pangolin::CreateDisplay()
                    .SetBounds(0.0,1.0,pangolin::Attach::Pix(175),1.0,-1024.0f/768.0f)
                   .SetHandler(new pangolin::Handler3D(vis_camera));

        bool bFollow = true;

        while(!pangolin::ShouldQuit() && mViewerIsRunning){
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
            std::unique_lock<std::mutex> lock(mViewerDataMutex);
            if(mCurrentFrame){
                if (menuFollowCamera && bFollow){
                    FollowCurrentFrame(vis_camera);
                }else if(!menuFollowCamera && bFollow){
                    bFollow = false;
                }else if(menuFollowCamera && !bFollow){
                    FollowCurrentFrame(vis_camera);
                    vis_camera.SetModelViewMatrix(
                        pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0));
                    bFollow = true;
                }
                cv::Mat img = PlotFrameImage();
                cv::imshow("frame", img); 
                cv::waitKey(1);         
            }

            vis_display.Activate(vis_camera);
            if(mCurrentFrame){
                DrawFrame(mCurrentFrame, green);
            }
            if (mMap){
                DrawKFsAndMPs(menuShowKeyFrames, menuShowPoints);
            }
            
            pangolin::FinishFrame();
            usleep(1000);
        }
        LOG(INFO) << "Stop viewer";
    }

    void Viewer::FollowCurrentFrame(pangolin::OpenGlRenderState& vis_camera){
        SE3 Twc = mCurrentFrame->GetPose().inverse();
        pangolin::OpenGlMatrix m(Twc.matrix());
        vis_camera.Follow(m, true);

    }

    cv::Mat Viewer::PlotFrameImage(){
        cv::Mat img_out;
        cv::cvtColor(mCurrentFrame->mImgLeft, img_out, CV_GRAY2BGR);
        for (size_t i = 0, N = mCurrentFrame->mvpFeatureLeft.size(); i < N; ++i){
                auto feat = mCurrentFrame->mvpFeatureLeft[i];
                cv::circle(img_out, feat->mKeyPoint.pt, 2, cv::Scalar(0,250,0), 2);
        }
        return img_out;
    }

    void Viewer::DrawFrame(Frame::Ptr frame, const float* color){
        SE3 Twc = frame->GetPose().inverse();
        const float sz = 1.0;
        const int line_width = 2.0;
        const float fx = 400;
        const float fy = 400;
        const float cx = 512;
        const float cy = 384;
        const float width = 1080;
        const float height = 768;

        glPushMatrix();

        Sophus::Matrix4f m = Twc.matrix().template cast<float>();
        glMultMatrixf((GLfloat*)m.data());

        if (color == nullptr) {
            glColor3f(1, 0, 0);
        } else
            glColor3f(color[0], color[1], color[2]);

        glLineWidth(line_width);
        glBegin(GL_LINES);
        glVertex3f(0, 0, 0);
        glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
        glVertex3f(0, 0, 0);
        glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
        glVertex3f(0, 0, 0);
        glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
        glVertex3f(0, 0, 0);
        glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

        glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
        glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

        glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
        glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

        glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
        glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);

        glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
        glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

        glEnd();
        glPopMatrix();
    }

    void Viewer::DrawKFsAndMPs(const bool menuShowKeyFrames, const bool menuShowPoints){
        if (menuShowKeyFrames){
            for (auto& kf: mMap->GetAllKeyFrames()){
                DrawFrame(kf.second, blue);
            }
        }
        
        glPointSize(2);
        glBegin(GL_POINTS);

        if(menuShowPoints){
            for (auto& mp : mMap->GetAllMapPoints()) {
                auto pos = mp.second->GetPose();
                glColor3f(red[0], red[1], red[2]);
                glVertex3d(pos[0], pos[1], pos[2]);
            }
        }
        glEnd();
    }
}