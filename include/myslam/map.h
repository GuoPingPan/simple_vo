#pragma once
#ifndef MAP_H
#define MAP_H

#include "common_include.h"
#include "frame.h"
#include "mappoint.h"

namespace myslam {
    /**
     * @brief 地图
     * @function 插入关键帧、增加地图点、删除地图点、获得所有地图点、获取所有关键帧、获取激活地图点、获取激活关键帧、清除map中观测为0的点、将旧关键帧设置为不活跃状态
     */

    class Map {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Map> Ptr;
        typedef std::unordered_map<unsigned long, MapPoint::Ptr> LandmarksType;
        typedef std::unordered_map<unsigned long, Frame::Ptr> KeyframesType;

        Map() {}

        void insertKeyFrame(Frame::Ptr frame);
        void insertMapPoint(MapPoint::Ptr map_point);

        /// 获取所有地图点
        LandmarksType getAllMapPoints() {
            std::unique_lock<std::mutex> lck(data_mutex_);
            return landmarks_;
        }
        /// 获取所有关键帧
        KeyframesType getAllKeyFrames() {
            std::unique_lock<std::mutex> lck(data_mutex_);
            return keyframes_;
        }

        /// 获取激活地图点
        LandmarksType getActiveMapPoints() {
            std::unique_lock<std::mutex> lck(data_mutex_);
            return active_landmarks_;
        }

        /// 获取激活关键帧
        KeyframesType getActiveKeyFrames() {
            std::unique_lock<std::mutex> lck(data_mutex_);
            return active_keyframes_;
        }

        /// 清理map中观测数量为零的点
        void cleanMap();


    private:
        // 只能被类内调用
        void removeOldKeyframe();

        std::mutex data_mutex_;
        LandmarksType landmarks_;
        LandmarksType active_landmarks_;
        KeyframesType keyframes_;
        KeyframesType active_keyframes_;

        Frame::Ptr current_frame_ = nullptr;

        int num_active_keyframes_ = 7;  // 激活的关键帧数量

    };


} // namespace myslam

#endif // MAP_H