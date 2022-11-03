#pragma once
#ifndef MYSLAM_COMMON_INCLUDE_H
#define MYSLAM_COMMON_INCLUDE_H
/*
 * #ifndef的方式依赖于宏名字不能冲突，这不光可以保证同一个文件不会被包含多次，也能保证内容完全相同的两个文件不会被不小心同时包含。
 * 但是无法避免宏名撞车
 * #pragma once则由编译器提供保证：同一个文件不会被包含多次。注意这里所说的“同一个文件”是指物理上的一个文件，而不是指内容相同的两个文件。
 * 无法避免头文件拷贝
 */

// 为了多线程
// https://iowiki.com/cpp_standard_library/atomic.html
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>

#include <iostream>
#include <list>
#include <map>
#include <set>
#include <unordered_map>
#include <vector>

#include <memory>
#include <typeinfo>

#include <Eigen/Core>
#include <Eigen/Geometry>

// double matrix
typedef Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> MatXXd;
typedef Eigen::Matrix<double, 10, 10> Mat1010d;
typedef Eigen::Matrix<double, 13, 13> Mat1313d;
typedef Eigen::Matrix<double, 8, 10> Mat810d;
typedef Eigen::Matrix<double, 8, 3> Mat83d;
typedef Eigen::Matrix<double, 6, 6> Mat66d;
typedef Eigen::Matrix<double, 5, 3> Mat53d;
typedef Eigen::Matrix<double, 4, 3> Mat43d;
typedef Eigen::Matrix<double, 4, 2> Mat42d;
typedef Eigen::Matrix<double, 3, 3> Mat33d;
typedef Eigen::Matrix<double, 2, 2> Mat22d;
typedef Eigen::Matrix<double, 8, 8> Mat88d;
typedef Eigen::Matrix<double, 7, 7> Mat77d;
typedef Eigen::Matrix<double, 4, 9> Mat49d;
typedef Eigen::Matrix<double, 8, 9> Mat89d;
typedef Eigen::Matrix<double, 9, 4> Mat94d;
typedef Eigen::Matrix<double, 9, 8> Mat98d;
typedef Eigen::Matrix<double, 8, 1> Mat81d;
typedef Eigen::Matrix<double, 1, 8> Mat18d;
typedef Eigen::Matrix<double, 9, 1> Mat91d;
typedef Eigen::Matrix<double, 1, 9> Mat19d;
typedef Eigen::Matrix<double, 8, 4> Mat84d;
typedef Eigen::Matrix<double, 4, 8> Mat48d;
typedef Eigen::Matrix<double, 4, 4> Mat44d;
typedef Eigen::Matrix<double, 3, 4> Mat34d;
typedef Eigen::Matrix<double, 14, 14> Mat1414d;

// float matrix
typedef Eigen::Matrix<float, 3, 3> Mat33f;
typedef Eigen::Matrix<float, 10, 3> Mat103f;
typedef Eigen::Matrix<float, 2, 2> Mat22f;
typedef Eigen::Matrix<float, 3, 1> Vec3f;
typedef Eigen::Matrix<float, 2, 1> Vec2f;
typedef Eigen::Matrix<float, 6, 1> Vec6f;
typedef Eigen::Matrix<float, 1, 8> Mat18f;
typedef Eigen::Matrix<float, 6, 6> Mat66f;
typedef Eigen::Matrix<float, 8, 8> Mat88f;
typedef Eigen::Matrix<float, 8, 4> Mat84f;
typedef Eigen::Matrix<float, 6, 6> Mat66f;
typedef Eigen::Matrix<float, 4, 4> Mat44f;
typedef Eigen::Matrix<float, 12, 12> Mat1212f;
typedef Eigen::Matrix<float, 13, 13> Mat1313f;
typedef Eigen::Matrix<float, 10, 10> Mat1010f;
typedef Eigen::Matrix<float, 9, 9> Mat99f;
typedef Eigen::Matrix<float, 4, 2> Mat42f;
typedef Eigen::Matrix<float, 6, 2> Mat62f;
typedef Eigen::Matrix<float, 1, 2> Mat12f;
typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> MatXXf;
typedef Eigen::Matrix<float, 14, 14> Mat1414f;

// double vectors
typedef Eigen::Matrix<double, 14, 1> Vec14d;
typedef Eigen::Matrix<double, 13, 1> Vec13d;
typedef Eigen::Matrix<double, 10, 1> Vec10d;
typedef Eigen::Matrix<double, 9, 1> Vec9d;
typedef Eigen::Matrix<double, 8, 1> Vec8d;
typedef Eigen::Matrix<double, 7, 1> Vec7d;
typedef Eigen::Matrix<double, 6, 1> Vec6d;
typedef Eigen::Matrix<double, 5, 1> Vec5d;
typedef Eigen::Matrix<double, 4, 1> Vec4d;
typedef Eigen::Matrix<double, 3, 1> Vec3d;
typedef Eigen::Matrix<double, 2, 1> Vec2d;
typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VecXd;

// float vectors
typedef Eigen::Matrix<float, 12, 1> Vec12f;
typedef Eigen::Matrix<float, 8, 1> Vec8f;
typedef Eigen::Matrix<float, 10, 1> Vec10f;
typedef Eigen::Matrix<float, 4, 1> Vec4f;
typedef Eigen::Matrix<float, 12, 1> Vec12f;
typedef Eigen::Matrix<float, 13, 1> Vec13f;
typedef Eigen::Matrix<float, 9, 1> Vec9f;
typedef Eigen::Matrix<float, Eigen::Dynamic, 1> VecXf;
typedef Eigen::Matrix<float, 14, 1> Vec14f;


#include <sophus/se3.hpp> // 其实这个文件已经包含了so3
#include <sophus/so3.hpp>

typedef Sophus::SE3d SE3d;
typedef Sophus::SO3d SO3d;

#include <opencv2/core/core.hpp>

using cv::Mat; // 这里是有什么用吗? 感觉只是声明了一下,似乎声明可以省略名字
// https://zhuanlan.zhihu.com/p/127020516

#include <glog/logging.h>

#endif //MYSLAM_COMMON_INCLUDE_H