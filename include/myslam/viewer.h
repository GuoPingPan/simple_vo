#pragma once
#ifndef MYSLAM_VIEWER_H
#define MYSLAM_VIEWER_H

#include <thread>
#include <pangolin/pangolin.h>

#include "common_include.h"
#include "map.h"
#include "frame.h"

namespace myslam {

    class Viewer {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Viewer> Ptr;

        Viewer();

        void setMap(Map::Ptr map){ map_ = map; }
        void close();
        void addCurrentFrame(Frame::Ptr current_frame);
        void updateMap();

    private:
        void threadLoop();

        void drawFrame(Frame::Ptr frame, const float *color);

        void drawMapPoint();

        void followCurrentFrame(pangolin::OpenGlRenderState& vis_camera);

        /// 将当前帧的特征画到一张图片上
        cv::Mat plotFrameImage();

        Frame::Ptr current_frame_ = nullptr;
        Map::Ptr map_ = nullptr;

        std::thread viewer_thread_;
        bool viewer_running_ = true;

        std::unordered_map<unsigned long, Frame::Ptr> active_keyframes_;
        std::unordered_map<unsigned long, MapPoint::Ptr> active_landmarks_;
        bool map_updated_ = false;

        std::mutex viewer_data_mutex_;

    };



} // namespace myslam




#endif // MYSLAM_VIEWER_H