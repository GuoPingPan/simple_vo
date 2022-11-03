#pragma once
#ifndef MYSLAM_ALGORITHM_H
#define MYSLAM_ALGORITHM_H

#include "common_include.h"

namespace myslam {

    /**
     * 三角化过程，通过 x_normal = pose * pw -> x_normal x (pose * pw) = 0 进而通过叉乘公式展开，每个点获得两个有效线性方程
     * https://blog.csdn.net/YunLaowang/article/details/89640279
     * @param poses
     * @param points 归一化平面坐标
     * @param pt_world
     * @return
     */
    inline bool triangulation(const std::vector<SE3d> &poses, const std::vector<Vec3d> &points, Vec3d &pt_world) {

        MatXXd A(2*poses.size(),4);
        VecXd b(2*poses.size());
        b.setZero();
        for(size_t i = 0; i < poses.size(); ++i){
            Mat34d pose = poses[i].matrix3x4();
            A.block<1,4>(i*2,0) = points[i][0] * pose.row(2) - pose.row(0);
            A.block<1,4>(i*2+1,0) = points[i][1] * pose.row(2) - pose.row(1);
        }
        auto svd = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
        pt_world = (svd.matrixV().col(3) / svd.matrixV()(3,3)).head<3>();

        if (svd.singularValues()[3] / svd.singularValues()[2] < 1e-2) {
            // 解质量不好，放弃
            return true;
        }
        return false;
    }

// converters
    inline Vec2d toVec2(const cv::Point2f p) { return Vec2d(p.x, p.y); }



} // namespace myslam


#endif // MYSLAM_ALGORITHM_H