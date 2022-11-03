#include "../include/myslam/frame.h"

namespace myslam {

    Frame::Frame(long id, double time_stamp, const SE3d &pose, const Mat &left, const Mat &right)
    :id_(id), time_stamp_(time_stamp), pose_(pose), left_img_(left), right_img_(right) {}

    Frame::Ptr Frame::createFrame() {
        static long factory_id = 0; // 这个是静态变量，一旦初始化了就不会发生改变
        Frame::Ptr new_frame(new Frame);
        new_frame->id_ = factory_id++;
        return new_frame;
    }

    void Frame::setKeyFrame() {
        static long keyframe_factory_id = 0;
        is_keyframe = true;
        keyframe_id_ = keyframe_factory_id++;
    }


} // namespace myslam