#include "../include/myslam/camera.h"

namespace myslam {


    Vec3d Camera::world2camera(const Vec3d &p_w, const SE3d &T_c_w) {
        // 这里的pose应该可以理解为相机相对与第一个相机的pose
        return pose_ * T_c_w * p_w;
    }

    Vec3d Camera::camera2world(const Vec3d &p_c, const SE3d &T_c_w) {
        return T_c_w.inverse() * pose_inv_ * p_c;
    }

    Vec2d Camera::camera2pixel(const Vec3d &p_c) {
        return Vec2d(
                fx_  * p_c(0,0) / p_c(2,0)  + cx_,
                fy_  * p_c(1,0) / p_c(2,0)  + cy_
                );
    }

    Vec3d Camera::pixel2camera(const Vec2d &p_p, double depth) {
        return Vec3d (
                (p_p(0,0) - cx_) * depth / fx_,
                (p_p(1,0) - cy_) * depth / fy_,
                depth
                );
    }

    Vec2d Camera::world2pixel(const Vec3d &p_w, const SE3d &T_c_w) {
        return camera2pixel(world2camera(p_w,T_c_w));
    }

    Vec3d Camera::pixel2world(const Vec2d &p_p, const SE3d &T_c_w, double depth) {
        return camera2world(pixel2camera(p_p,depth),T_c_w);
    }

} // namespace myslam