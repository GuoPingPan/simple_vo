#pragma once
#ifndef MYSLAM_FRONTEND_H
#define MYSLAM_FRONTEND_H

#include <opencv2/features2d.hpp>
#include "frame.h"
#include "map.h"
#include "common_include.h"


namespace myslam {

    class Backend;
    class Viewer;

    enum class FrontendStatus { INITING, TRACKING_GOOD, TRACKING_BAD, LOST };

    class Frontend {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Frontend> Ptr;

        Frontend();

        bool addFrame(Frame::Ptr frame);
        void setMap(Map::Ptr map) { map_ = map; }
        void setViewer(std::shared_ptr<Viewer> viewer) { viewer_ = viewer; }
        void setBackEnd(std::shared_ptr<Backend> backend) { backend_ = backend; }


        FrontendStatus getStatus() const { return status_; }

        void setCameras(Camera::Ptr left, Camera::Ptr right){
            camera_left_ = left;
            camera_right_ = right;
        }

    private:
        bool Track();
        bool Reset();

        /**
         * Track with last frame
         * @return  num of match points
         */
        int  trackLastFrame();

        /**
         * Estimate current frame pose
         * @return num of inliers
         */
        int estimateCurrentPose();

        bool insertKeyFrame();

        bool StereoInit();

        /**
         * Detect features in current frame, and will keep in current frame
         * @return
         */
        int detectFeature();

        /**
         * Find corresponding feature in right image of current frame
         * @return num of features found
         */
        int findFeatureInRight();

        bool buildInitMap();

        int triangulateNewPoints();

        /**
         * todo 是将当前帧的 features 作为 mappoint 的 new observation？
         * 即更新 observation
         */
        void setObservationsForKeyFrame();

        FrontendStatus status_ = FrontendStatus::INITING;

        Frame::Ptr current_frame_ = nullptr;
        Frame::Ptr last_frame_ = nullptr;
        Camera::Ptr camera_left_ = nullptr;
        Camera::Ptr camera_right_ = nullptr;

        Map::Ptr map_ = nullptr;
        std::shared_ptr<Backend> backend_ = nullptr;
        std::shared_ptr<Viewer> viewer_ = nullptr;

        SE3d relative_motion_; // 相对上一帧的运动

        int tracking_inliers_; // 用于测试新的关键帧

        int num_features_ = 200;
        int num_features_init_ = 100;
        int num_features_tracking_ = 50;
        int num_features_tracking_bad_ = 20;
        int num_features_needed_for_keyframe_ = 80;

        // todo 自己改成 ORB Descriptor 试试
        cv::Ptr<cv::GFTTDetector> gftt_; // opencv 特征点检测器

    };



} // namespace myslam



#endif //MYSLAM_FRONTEND_H