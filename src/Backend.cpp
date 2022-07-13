#include "Backend.h"

namespace myslam{

    Backend::Backend(){
        std::thread([this] { Running(); });
    }

    void Backend::Running(){
        while(1){
            
        }
    }
        

}