#include "../include/myslam/mappoint.h"
#include "../include/myslam/feature.h"


namespace myslam {

    MapPoint::MapPoint(long id, Vec3d position) : id_(id), pos_(position) {}

    MapPoint::Ptr MapPoint::createNewMappoint() {
        static long factory_id = 0;
        MapPoint::Ptr new_mappoint(new MapPoint);
        new_mappoint->id_ = factory_id++;
        return new_mappoint;
    }

    /**
     * 删除一个特征点
     * @param feat
     */
    void MapPoint::removeObservation(std::shared_ptr<Feature> feat) {

        std::unique_lock<std::mutex> lck(data_mutex_);
        for(auto iter = observations_.begin(); iter != observations_.end(); iter++){
            if(iter->lock() == feat) {
                observations_.erase(iter);
                feat->map_point_.reset();
                --observed_times_;
                break;
            }
        }
    }



} // namespace myslam