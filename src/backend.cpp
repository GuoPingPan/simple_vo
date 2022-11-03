#include <opencv2/opencv.hpp>

#include "../include/myslam/algorithm.h"
#include "../include/myslam/backend.h"
#include "../include/myslam/config.h"
#include "../include/myslam/feature.h"
#include "../include/myslam/frontend.h"
#include "../include/myslam/g2o_types.h"
#include "../include/myslam/map.h"
#include "../include/myslam/viewer.h"

namespace myslam {

    Backend::Backend() {
        /// atomic_bool
        // https://blog.csdn.net/sjc_0910/article/details/118861539?spm=1001.2014.3001.5506
        backend_running_.store(true);
        backend_thread_ = std::thread(std::bind(&Backend::backendLoop, this));
    }

    void Backend::updateMap() {
        std::unique_lock<std::mutex> lock(data_mutex_);
        /// condition_variable
        map_update_.notify_one();
    }

    void Backend::stop() {
        backend_running_.store(false);
        map_update_.notify_one();
        backend_thread_.join();
    }

    void Backend::backendLoop() {
        while(backend_running_.load()) {
            std::unique_lock<std::mutex> lock(data_mutex_);
            map_update_.wait(lock);

            /// 后端仅优化激活的Frames和Landmarks
            Map::KeyframesType active_kfs = map_->getActiveKeyFrames();
            Map::LandmarksType active_landmarks = map_->getActiveMapPoints();
            optimize(active_kfs, active_landmarks);
        }
    }



    void Backend::optimize(Map::KeyframesType &keyframes, Map::LandmarksType &landmarks) {
        // setup g2o
        // https://blog.csdn.net/shoufei403/article/details/127344670
        // 这个3即不是measure的维度，也不是error的维度，而是输入观测的维度，可以是节点，也可以不是
        typedef g2o::BlockSolver_6_3 BlockSolverType;
        typedef g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType>
                LinearSolverType;
        auto solver = new g2o::OptimizationAlgorithmLevenberg(
                g2o::make_unique<BlockSolverType>(
                        g2o::make_unique<LinearSolverType>()));
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);

        // pose 顶点,使用Keyframe id
        std::map<unsigned long, VertexPose *> vertices;  // keyframe_id , pose
        unsigned long max_kf_id = 0;
        for (auto &keyframe : keyframes) {
            auto kf = keyframe.second;
            VertexPose *vertex_pose = new VertexPose();  // camera vertex_pose
            vertex_pose->setId(kf->keyframe_id_);
            vertex_pose->setEstimate(kf->pose());
            optimizer.addVertex(vertex_pose); // 加入顶点
            if (kf->keyframe_id_ > max_kf_id) {
                max_kf_id = kf->keyframe_id_;
            }

            vertices.insert({kf->keyframe_id_, vertex_pose});
        }

        // 路标顶点，使用路标id索引
        std::map<unsigned long, VertexXYZ *> vertices_landmarks; // landmark_id , point

        // K 和左右外参
        Mat33d K = cam_left_->K();
        SE3d left_ext = cam_left_->pose();
        SE3d right_ext = cam_right_->pose();


        // edges
        int index = 1;
        double chi2_th = 5.991;  // robust kernel 阈值
        std::map<EdgeProjection *, Feature::Ptr> edges_and_features; // 用于后面设定outlier

        /// 每个路标
        for (auto &landmark : landmarks) {
            if (landmark.second->is_outlier_) continue;
            unsigned long landmark_id = landmark.second->id_;
            auto observations = landmark.second->getObservation();

            /// 选择最近第一个观测到的该路标的feature，与landmark对应
            for (auto &obs : observations) {
                if (obs.lock() == nullptr) continue;
                auto feat = obs.lock();
                if (feat->is_outlier_ || feat->frame_.lock() == nullptr) continue;

                auto frame = feat->frame_.lock();

                /// 根据相机的左右设定位移矩阵
                EdgeProjection *edge = nullptr;
                if (feat->is_on_left_image_) {
//                    edge = new EdgeProjection(left_ext, K);
                    edge = new EdgeProjection(K, left_ext);
                } else {
                    edge = new EdgeProjection(K, right_ext);
                }

                /// 如果landmark还没有被加入优化，则新加一个顶点
                if (vertices_landmarks.find(landmark_id) ==
                    vertices_landmarks.end()) {
                    VertexXYZ *v = new VertexXYZ;
                    v->setEstimate(landmark.second->pos());
                    v->setId(landmark_id + max_kf_id + 1);
                    v->setMarginalized(true); /// 设定边缘化
                    vertices_landmarks.insert({landmark_id, v});
                    optimizer.addVertex(v);
                }

                /// 设定边
                edge->setId(index);
                edge->setVertex(0, vertices.at(frame->keyframe_id_));    // pose
                edge->setVertex(1, vertices_landmarks.at(landmark_id));  // landmark
                edge->setMeasurement(toVec2(feat->position_.pt));
                edge->setInformation(Mat22d::Identity());
                auto rk = new g2o::RobustKernelHuber();
                rk->setDelta(chi2_th);
                edge->setRobustKernel(rk);
                edges_and_features.insert({edge, feat});

                optimizer.addEdge(edge);

                index++;
            }
        }

        // do optimization and eliminate the outliers
        optimizer.initializeOptimization();
        optimizer.optimize(10);

        int cnt_outlier = 0, cnt_inlier = 0;
        int iteration = 0;
        while (iteration < 5) {
            cnt_outlier = 0;
            cnt_inlier = 0;
            // determine if we want to adjust the outlier threshold
            for (auto &ef : edges_and_features) {
                if (ef.first->chi2() > chi2_th) {
                    cnt_outlier++;
                } else {
                    cnt_inlier++;
                }
            }

            /// 有一半的inlier就退出
            double inlier_ratio = cnt_inlier / double(cnt_inlier + cnt_outlier);
            if (inlier_ratio > 0.5) {
                break;
            } else {
                chi2_th *= 2;
                iteration++;
            }
        }

        //
        for (auto &ef : edges_and_features) {
            if (ef.first->chi2() > chi2_th) {
                ef.second->is_outlier_ = true;
                // remove the observation
                ef.second->map_point_.lock()->removeObservation(ef.second);
            } else {
                ef.second->is_outlier_ = false;
            }
        }

        LOG(INFO) << "Outlier/Inlier in optimization: " << cnt_outlier << "/"
                  << cnt_inlier;

        // Set pose and lanrmark position
        for (auto &v : vertices) {
            keyframes.at(v.first)->setPose(v.second->estimate());
        }
        for (auto &v : vertices_landmarks) {
            landmarks.at(v.first)->setPos(v.second->estimate());
        }
    }




} // namespace myslam