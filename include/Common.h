#ifndef MYSLAM_COMMON_H
#define MYSLAM_COMMON_H

#include <iostream>

#include <atomic>

#include <memory>
#include <string>
#include <list>
#include <thread>

// glog
#include <gflags/gflags.h>
#include <glog/logging.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

namespace myslam{
    typedef Sophus::SE3d SE3;
    typedef Sophus::SO3d SO3;


    typedef Eigen::Vector3d Vector3d;
    typedef Eigen::Matrix<double, 2, 1> Vec2d;
    typedef Eigen::Matrix<double, 3, 1> Vec3d;
    typedef Eigen::Matrix<double, 6, 1> Vec6d;
    typedef Eigen::Matrix<double, 3, 3> Mat33d;
    typedef Eigen::Matrix<double, 2, 2> Mat22d;

#define SE3_Identity SE3(SO3(), Vector3d::Zero())

    inline Vec2d cvPoint2Vec2(const cv::Point2f& point){
        return Vec2d(point.x, point.y);
    }
}




#endif