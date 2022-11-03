#ifndef MYSLAM_DATASET_H
#define MYSLAM_DATASET_H
#include "camera.h"
#include "common_include.h"
#include "frame.h"

namespace myslam{

    class Dataset {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Dataset> Ptr;

        Dataset(const std::string &dataset_path);

        bool init();

        Frame::Ptr getNextFrame();

        Camera::Ptr getCamera(int camera_id) const {
            return cameras_.at(camera_id); // at 是vector的函数
        }

    private:
        std::string dataset_path_;
        int current_image_index_ = 0;

        std::vector<Camera::Ptr> cameras_;

    };


} // namespace myslam

#endif // MYSLAM_DATASET_H