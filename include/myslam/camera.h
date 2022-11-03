#pragma once
#ifndef MYSLAM_CAMERA_H
#define MYSLAM_CAMERA_H

#include "common_include.h"

namespace myslam {

    class Camera {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        // https://blog.csdn.net/shyjhyp11/article/details/123208279
        // https://blog.csdn.net/shyjhyp11/article/details/123204444
        typedef std::shared_ptr<Camera> Ptr;

        double fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0, baseline_ = 0;  // Camera intrinsics
        SE3d pose_;
        SE3d pose_inv_;

        Camera() {}
        Camera(double fx, double fy, double cx, double cy, double baseline, const SE3d &pose)
            : fx_(fx), fy_(fy), cx_(cx), cy_(cy), baseline_(baseline), pose_(pose) {
            pose_inv_ = pose_.inverse();
        }

        // const 不改变类内成员
        SE3d pose() const { return pose_; }

        Mat33d K() const {
            Mat33d k;
            k << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1;
            return k;
        }

        // coordinate transform: world, camera, pixel
        // transform 均用 T_c_w表达
        Vec3d world2camera(const Vec3d &p_w, const SE3d &T_c_w);

        Vec3d camera2world(const Vec3d &p_c, const SE3d &T_c_w);

        Vec2d camera2pixel(const Vec3d &p_c);

        Vec3d pixel2camera(const Vec2d &p_p, double depth = 1);

        Vec3d pixel2world(const Vec2d &p_p, const SE3d &T_c_w, double depth = 1);

        Vec2d world2pixel(const Vec3d &p_w, const SE3d &T_c_w);

    };

} // namespace myslam

#endif //MYSLAM_CAMERA_H