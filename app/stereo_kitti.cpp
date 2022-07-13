#include "System.h"
#include <iostream>
#include "Common.h"

using namespace std;
using namespace myslam;

int main(int argc, char **argv){

//    if(argc != 3){
//        LOG(INFO) << "argc: " << argc;
//        for(int i = 0; i < argc; i++){
//            LOG(INFO) << "argv[" << i << "]: " << argv[i];
//        }
//        std::cerr << endl << "Usage:  ./bin/run_kitti_stereo   path_to_config   path_to_sequence" << std::endl;
//        return 1;
//    }

    LOG(INFO) << "argc: " << argc;
        for(int i = 0; i < argc; i++){
            LOG(INFO) << "argv[" << i << "]: " << argv[i];
        }
    std::string strConfigPath = argv[1];
    std::string strSequencePath = argv[2];
    // load sequence frames
    std::vector<std::string> vstrImageLeft, vstrImageRight;
    std::vector<double> vTimestamps;

    System::Ptr slam(new System(strConfigPath));

    slam->LoadImages(strSequencePath, vstrImageLeft, vstrImageRight, vTimestamps);
    const unsigned long nImages = vstrImageLeft.size();

   LOG(INFO) << "nImages: " << nImages << endl;

   for(int ni = 0; ni < nImages; ni++){
       cv::Mat ImgLeft = cv::imread(vstrImageLeft[ni], cv::IMREAD_GRAYSCALE);
       cv::Mat ImgRight = cv::imread(vstrImageRight[ni], cv::IMREAD_GRAYSCALE);
       double TimeStamp = vTimestamps[ni];

       if(ImgLeft.empty()){
            std::cerr << std::endl << "Failed to load image at: "
                 << string(vstrImageLeft[ni]) << std::endl;
            return 1;
        }

        //　开始运行
        slam->Run(ImgLeft, ImgRight, TimeStamp);
   }


    


    return 0;

}