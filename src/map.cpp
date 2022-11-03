#include "../include/myslam/map.h"
#include "../include/myslam/feature.h"

namespace myslam {

    /**
     * 这里是在地图中插入关键帧
     * @param frame
     */
    void Map::insertKeyFrame(Frame::Ptr frame) {

        current_frame_ = frame;

        if(keyframes_.find(frame->keyframe_id_) == keyframes_.end()) {
            keyframes_.insert(std::make_pair(frame->keyframe_id_, frame)); // 注意是 keyframe_id_
            active_keyframes_.insert(std::make_pair(frame->keyframe_id_, frame));
        }
        // todo 怎么会存在插入过的关键帧？
        else {
            keyframes_[frame->keyframe_id_] = frame;
            active_keyframes_[frame->keyframe_id_] = frame;
        }
    }


    void Map::insertMapPoint(MapPoint::Ptr map_point) {

        if(landmarks_.find(map_point->id_) == landmarks_.end()){
            landmarks_.insert(std::make_pair(map_point->id_,map_point)); // mappoint id
            active_landmarks_.insert(std::make_pair(map_point->id_,map_point));
        }
        else{
            landmarks_[map_point->id_] = map_point;
            active_landmarks_[map_point->id_] = map_point;
        }
    }

    void Map::removeOldKeyframe() {
        if(current_frame_ == nullptr) return;
        // 寻找最近和最远的两个活跃关键帧
        double max_dis = 0, min_dis = 9999;
        double max_kf_id = 0, min_kf_id = 0;
        auto T_wc = current_frame_->pose().inverse();
        for(auto &kf : active_keyframes_){
            if(kf.second == current_frame_) continue;
            auto dist = (kf.second->pose() * T_wc).log().norm();
            if(dist < min_dis){
                min_dis = dist;
                min_kf_id = kf.first; // keyframe id
            }
            if(dist > max_dis){
                max_dis = dist;
                max_kf_id = kf.first;
            }
        }

        const double min_dis_th = 0.2; // 阈值为20cm
        Frame::Ptr frame_to_remove = nullptr;
        // todo 这里为什么是根据 keyframes_去找呢，而不是根据activate_keyframes_;
        // 其实keyframe和activate keyframe的索引都是keyframe_id，是一样的
        // 这里没有直接调用 active_keyframes_.earse(min_kf+id);
        // 是为了后面要删除特征点
        if (min_dis < min_dis_th)
            frame_to_remove = keyframes_.at(min_kf_id);
        else
            frame_to_remove = keyframes_.at(max_kf_id);

        LOG(INFO) << "remove keyframe " << frame_to_remove->keyframe_id_;
        // 这里关键帧应该还没有彻底释放
        active_keyframes_.erase(frame_to_remove->keyframe_id_);

        // 删除帧对应的特征点，因为feature只存在于frame里面，与mappoint的联系在于observation
        for(auto feat : frame_to_remove->feature_left_) {
            auto mp = feat->map_point_.lock();
            if(mp) {
                mp->removeObservation(feat);

            }
        }

        for(auto feat : frame_to_remove->feature_right_) {
            auto mp = feat->map_point_.lock();
            if(mp) {
                mp->removeObservation(feat);
            }
        }

        cleanMap();
    }


    void Map::cleanMap() {
        int cnt_landmark_removed = 0;
        for(auto iter = active_landmarks_.begin(); iter != active_landmarks_.end();) {
            if(iter->second->observed_times_ == 0) {
                iter = active_landmarks_.erase(iter); // 返回删除后的位置
                ++cnt_landmark_removed;
            }
            else{
                ++iter;
            }
        }
        LOG(INFO) << "Removed " << cnt_landmark_removed << " active landmarks";
    }


} // namespace myslam