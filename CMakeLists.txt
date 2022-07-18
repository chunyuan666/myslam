cmake_minimum_required(VERSION 3.0)
project(myslam)

set(CMAKE_CXX_FLAGS "-std=c++14  -Wall -g")
set(CMAKE_CXX_FLAGS_DEBUG "-std=c++14  -fopenmp -pthread")

# 第三方编译库
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
message(STATUS ${PROJECT_SOURCE_DIR})

# 设置可执行文件路径、库文件路径
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(SYSTEM "/foo/bar")

#Eigen
#include_directories("/usr/include/eigen3")
find_package(Eigen3 3.1.0 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

#Sophus
find_package(Sophus REQUIRED)
include_directories(${Sophus_Include_DIRS})

#OpenCV
find_package(OpenCV 3 REQUIRED)
include_directories(${OpenCV_Include_DIRS})

# glog: 实现应用级别的logging
find_package(Glog REQUIRED)
include_directories(${GLOG_INCLUDE_DIRS})

# gtest
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

# gflags
find_package(GFlags REQUIRED)
include_directories(${GFLAGS_INCLUDE_DIRS})

# G2O
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

# csparse
find_package(CSparse REQUIRED)
include_directories(${CSPARSE_INCLUDE_DIR})



# 设置第三方库
set(THIRD_PARTY_LIBS
        ${OpenCV_LIBS}
        ${Sophus_LIBRARIES}
        ${GTEST_BOTH_LIBRARIES}
        ${GLOG_LIBRARIES}
        ${GFLAGS_LIBRARIES}
        fmt
        g2o_core g2o_stuff g2o_types_sba g2o_solver_csparse g2o_csparse_extension
        ${CSPARSE_LIBRARY}
        )

## 设置源文件路径、头文件路径
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/src)

add_library(${PROJECT_NAME} STATIC
        src/System.cpp
        src/Camera.cpp
        src/Frame.cpp
        src/Frontend.cpp
        src/ORBExtractor.cpp
        src/MapPoints.cpp
        src/Map.cpp
        src/KeyFrame.cpp
        src/Optimizer.cpp include/Optimizer.h
        src/Backend.cpp)

enable_testing()

add_executable(stereo_kitti app/stereo_kitti.cpp)
target_link_libraries(stereo_kitti ${PROJECT_NAME} ${THIRD_PARTY_LIBS})



