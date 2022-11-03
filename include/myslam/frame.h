#pragma once
#ifndef MYSLAM_FRAME_H
#define MYSLAM_FRAME_H

#include "camera.h"
#include "common_include.h"

namespace myslam {

    // 就像是头文件的实现放到cpp去一样，所以不用写具体结构体形式
    struct MapPoint;
    struct Feature;

    struct Frame {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Frame> Ptr;

        unsigned long id_ = 0;
        unsigned long keyframe_id_ = 0;
        bool  is_keyframe = false;
        double time_stamp_;
        SE3d pose_;
        std::mutex pose_mutex_;
        cv::Mat left_img_, right_img_;

        std::vector<std::shared_ptr<Feature>> feature_left_;
        std::vector<std::shared_ptr<Feature>> feature_right_;

    public:
        Frame() {}

        Frame(long id, double time_stamp, const SE3d &pose, const cv::Mat &left, const cv::Mat &right);

        SE3d pose() {
            std::unique_lock<std::mutex> lck(pose_mutex_);
            return pose_;
        }

        void setPose(const SE3d &pose) {
            std::unique_lock<std::mutex> lck(pose_mutex_);
            pose_ = pose;
        }

        void setKeyFrame();

        static std::shared_ptr<Frame> createFrame();



    };

} // namespace myslam

#endif //MYSLAM_FRAME_H