#pragma once
#ifndef MYSLAM_G2O_TYPES_H
#define MYSLAM_G2O_TYPES_H

#include "common_include.h"

#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_vertex.h>

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>

#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/solver.h>
#include <g2o/core/sparse_optimizer.h>

#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/base_multi_edge.h>

namespace myslam {

    // 顶点的维度和类型
    class VertexPose : public g2o::BaseVertex<6,SE3d> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        virtual void setToOriginImpl() override { _estimate = SE3d(); }

        virtual void oplusImpl(const double *update) override {
            Vec6d update_eigen;
            update_eigen << update[0], update[1], update[2], update[3], update[4], update[5];
            _estimate = SE3d::exp(update_eigen) * _estimate;
        }

        virtual bool read(std::istream &in) override { return true; }
        virtual bool write(std::ostream &out) const override { return true; }

    };

    class VertexXYZ : public g2o::BaseVertex<3,Vec3d> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        virtual void setToOriginImpl() override { _estimate = Vec3d::Zero(); }

        virtual void oplusImpl(const double *update) override {
            _estimate[0] += update[0];
            _estimate[1] += update[1];
            _estimate[2] += update[2];
        }

        virtual bool read(std::istream &in) override { return true; }
        virtual bool write(std::ostream &out) const override { return true; }
    };

    // 观测值的维度，类型；顶点类型1,2,...
    class EdgeProjectionPoseOnly : public g2o::BaseUnaryEdge<2,Vec2d,VertexPose> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        EdgeProjectionPoseOnly(const Vec3d &pose, const Mat33d &K) : _K(K),_pose(pose) {}

        virtual void computeError() override {
            const VertexPose *v = static_cast<VertexPose *>(_vertices[0]);
            SE3d T = v->estimate();
            Vec3d pos_pixel = _K * (T * _pose);
            pos_pixel /= pos_pixel[2];
            _error = _measurement - pos_pixel.head<2>();
        }


        virtual void linearizeOplus() override {
            const VertexPose *v = static_cast<VertexPose *>(_vertices[0]);
            SE3d T = v->estimate();
            Vec3d pose_cam = T * _pose;
            double fx = _K(0,0);
            double fy = _K(1,1);

            double X = pose_cam[0];
            double Y = pose_cam[1];
            double Z = pose_cam[2];
            double Zinv = 1.0 / (Z + 1e-18);
            double Zinv2 = Zinv * Zinv;

            _jacobianOplusXi << -fx * Zinv, 0, fx * X * Zinv2, fx * X * Y * Zinv2,
                    -fx - fx * X * X * Zinv2, fx * Y * Zinv, 0, -fy * Zinv,
                    fy * Y * Zinv2, fy + fy * Y * Y * Zinv2, -fy * X * Y * Zinv2,
                    -fy * X * Zinv;
        }

        virtual bool read(std::istream &in) override { return true; }
        virtual bool write(std::ostream &out) const override { return true; }


    private:
        Vec3d _pose;
        Mat33d _K;

    };

    // 观测值的维度，类型；顶点类型1,2,...
    // todo bug 这里的一个bug就是将Vertex的位置弄反了
    class EdgeProjection : public g2o::BaseBinaryEdge<2,Vec2d,VertexPose,VertexXYZ> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        EdgeProjection(const Mat33d &K, const SE3d &cam_ext) : _K(K) {
            _cam_ext = cam_ext; //之所以这么赋值是因为SE3d加括号就是函数了
        }

        virtual void computeError() override {
            const VertexPose *p = static_cast<VertexPose *>(_vertices[0]);
            const VertexXYZ *v = static_cast<VertexXYZ *>(_vertices[1]);
            SE3d T = p->estimate();
            Vec3d pos_pixel = _K * (_cam_ext * (T * v->estimate()));
            /// todo 这个bug太恶心了,就是pos_pixel / pos_pixel[2]看不出来
            pos_pixel /= pos_pixel[2];
            _error = _measurement - pos_pixel.head<2>();
        }


        virtual void linearizeOplus() override {
            const VertexPose *p = static_cast<VertexPose *>(_vertices[0]);
            const VertexXYZ *v = static_cast<VertexXYZ *>(_vertices[1]);
            SE3d T = p->estimate();
            Vec3d pw = v->estimate();
            Vec3d pose_cam = _cam_ext * T * pw;
            double fx = _K(0,0);
            double fy = _K(1,1);

            double X = pose_cam[0];
            double Y = pose_cam[1];
            double Z = pose_cam[2];
            double Zinv = 1.0 / (Z + 1e-18);
            double Zinv2 = Zinv * Zinv;

            _jacobianOplusXi << -fx * Zinv, 0, fx * X * Zinv2, fx * X * Y * Zinv2,
                    -fx - fx * X * X * Zinv2, fx * Y * Zinv, 0, -fy * Zinv,
                    fy * Y * Zinv2, fy + fy * Y * Y * Zinv2, -fy * X * Y * Zinv2,
                    -fy * X * Zinv;

            _jacobianOplusXj = _jacobianOplusXi.block<2, 3>(0, 0) *
                               _cam_ext.rotationMatrix() * T.rotationMatrix();
        }

        virtual bool read(std::istream &in) override { return true; }
        virtual bool write(std::ostream &out) const override { return true; }


    private:
        SE3d _cam_ext;
        Mat33d _K;

    };

} // namespace myslam

#endif //MYSLAM_G2O_TYPES_H
