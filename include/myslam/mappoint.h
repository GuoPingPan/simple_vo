#pragma once
#ifndef MYSLAM_MAPPOINT_H
#define MYSLAM_MAPPOINT_H

#include "common_include.h"

namespace myslam {

    struct Feature;
    struct Frame;

    struct MapPoint {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<MapPoint> Ptr;

        unsigned long id_ = 0;
        bool is_outlier_ = false;
        Vec3d pos_ = Vec3d::Zero();  // Position in world

        std::mutex data_mutex_;
        int observed_times_ = 0;
        std::list<std::weak_ptr<Feature>> observations_;

        MapPoint() {}

        MapPoint(long id, Vec3d position);

        Vec3d pos() {
            std::unique_lock<std::mutex> lck(data_mutex_);
            return pos_;
        }

        void setPos(const Vec3d &pos) {
            std::unique_lock<std::mutex> lck(data_mutex_);
            pos_ = pos;
        };

        void addObservation(std::shared_ptr<Feature> feature) {
            std::unique_lock<std::mutex> lck(data_mutex_);
            observations_.push_back(feature);
            observed_times_++;
        }

        void removeObservation(std::shared_ptr<Feature> feat);

        std::list<std::weak_ptr<Feature>> getObservation() {
            std::unique_lock<std::mutex> lck(data_mutex_);
            return observations_;
        }

        static MapPoint::Ptr createNewMappoint();

    };





} // namespace myslam

#endif // MYSLAM_MAPPOINT_H