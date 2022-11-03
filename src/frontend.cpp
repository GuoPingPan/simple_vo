#include <opencv2/opencv.hpp>

#include "../include/myslam/config.h"
#include "../include/myslam/frontend.h"
#include "../include/myslam/feature.h"
#include "../include/myslam/map.h"
#include "../include/myslam/viewer.h"
#include "../include/myslam/algorithm.h"
#include "../include/myslam/backend.h"
#include "../include/myslam/g2o_types.h"

namespace myslam {

    Frontend::Frontend() {
        // 检测特征点数，质量？，特征点最小间隔20像素
        gftt_ = cv::GFTTDetector::create(Config::get<int>("num_features"), 0.01, 20);
        num_features_init_ = Config::get<int>("num_features_init");
        num_features_ = Config::get<int>("num_features");
    }

    /**
     * Frontend的启动点
     * @param frame
     * @return
     */

    bool Frontend::addFrame(Frame::Ptr frame) {
        current_frame_ = frame;

        switch (status_) {
            case FrontendStatus::INITING:
                StereoInit();
                break;
            case FrontendStatus::TRACKING_GOOD:
            case FrontendStatus::TRACKING_BAD:
                Track();
                break;
            case FrontendStatus::LOST:
                Reset();
                break;

        }

        last_frame_ = current_frame_;
        return true;
    }

    int Frontend::detectFeature() {
        cv::Mat mask(current_frame_->left_img_.size(), CV_8UC1, 255); // 全白

        // 将当前帧左图的所有特征点用矩形画出来
        // https://blog.csdn.net/Du_Shuang/article/details/77905698
        for(auto &feat : current_frame_->feature_left_) {
            cv::rectangle(mask, feat->position_.pt - cv::Point2f(10,10),
                          feat->position_.pt + cv::Point2f(10,10), 0 ,CV_FILLED); // 对黑色位置屏蔽
        }

        std::vector<cv::KeyPoint> keypoints;
        // 因为每个特征点距离阈值为20，故对原有特征点进行mask
        gftt_->detect(current_frame_->left_img_,keypoints,mask);

        int cnt_detected = 0;
        for(auto &kp : keypoints){
            current_frame_->feature_left_.push_back(
                    Feature::Ptr(new Feature(current_frame_, kp))
                    );
            cnt_detected++;
        }

        LOG(INFO) << "Detect " << cnt_detected << " new features";
        return cnt_detected;
    }

    int Frontend::findFeatureInRight() {
        std::vector<cv::Point2f> kps_left, kps_right;
        for(auto &kp : current_frame_->feature_left_) {
            kps_left.push_back(kp->position_.pt);
            auto mp = kp->map_point_.lock();
            // todo 为什么用mappoint结合pose映射回来呢，这里的current_frame_pose()不是以左边图像为主吗?
            if(mp) {
                auto px = camera_right_->world2pixel(mp->pos_,current_frame_->pose());
                kps_right.push_back(cv::Point2f(px[0],px[1]));
            }
            else{
                kps_right.push_back(kp->position_.pt);
            }
        }

        std::vector<uchar> status;
        Mat error;
        // 前面对与kps_right是为了提供初值吗？，好像不需要设定初值也ok
        // https://www.freesion.com/article/9342580073/
        // 源码在 https://blog.csdn.net/jihengshan/article/details/20644073
        cv::calcOpticalFlowPyrLK(
                current_frame_->left_img_, current_frame_->right_img_, kps_left,
                kps_right, status, error, cv::Size(11,11), 3,
                //  TermCriteria https://blog.csdn.net/YMWM_/article/details/117967663
                cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
                cv::OPTFLOW_USE_INITIAL_FLOW // 使用初始值 https://blog.csdn.net/jihengshan/article/details/20644073
                );

        int num_good_pts = 0;
        for(size_t i = 0; i < status.size(); ++i){
            if(status[i]) {
                cv::KeyPoint kp(kps_right[i], 7);
                Feature::Ptr feat(new Feature(current_frame_, kp));
                feat->is_on_left_image_ = false;
                current_frame_->feature_right_.push_back(feat);
                num_good_pts++;
            }
            else{
                current_frame_->feature_right_.push_back(nullptr);
            }
        }
        LOG(INFO) << "Find " << num_good_pts << " in the right image.";
        return num_good_pts;
    }

    /**
     * 该函数实现初始地图的构建
     * @brief 首先是完成左右目图像的特征点检测
     *        然后利用左右相机位姿构造和pw的映射，利用三角化，每次两个匹配点恢复出一个地图点
     * @tips 应该就初始化能够使用camera的pose，后面都是frame的pose
     * @return
     */
    bool  Frontend::buildInitMap() {
        std::vector<SE3d> poses{camera_left_->pose(),camera_right_->pose()};
        size_t cnt_init_landmarks = 0;
        for(size_t i = 0; i < current_frame_->feature_left_.size(); ++i){
            if(current_frame_->feature_right_[i] == nullptr) continue;
            std::vector<Vec3d> points{
                    camera_left_->pixel2camera(
                            Vec2d(current_frame_->feature_left_[i]->position_.pt.x,
                                  current_frame_->feature_left_[i]->position_.pt.y)
                            ),
                    camera_right_->pixel2camera(
                            Vec2d(current_frame_->feature_right_[i]->position_.pt.x,
                                  current_frame_->feature_right_[i]->position_.pt.y)
                            )
            };

            Vec3d pw = Vec3d::Zero();

            //三角化需要归一化平面点和pose来求解pw
            // todo 求解的到的pw的z轴肯定是正的？世界坐标系怎么定义
            // 这里的三角化用的在相机坐标系下的,因为两个相机之间的外参可知道,故能恢复出深度,结合当前相机的位姿能够得到世界坐标系下的地图点
            if(triangulation(poses,points,pw) && pw[2] > 0) {
                auto new_map_point = MapPoint::createNewMappoint();
                new_map_point->setPos(pw);
                new_map_point->addObservation(current_frame_->feature_left_[i]);
                new_map_point->addObservation(current_frame_->feature_right_[i]);
                current_frame_->feature_left_[i]->map_point_ = new_map_point;
                current_frame_->feature_right_[i]->map_point_ = new_map_point;
                ++cnt_init_landmarks;
                map_->insertMapPoint(new_map_point);
            }

        }

        current_frame_->setKeyFrame();
        map_->insertKeyFrame(current_frame_);
        backend_->updateMap();


        LOG(INFO) << "Initial map created with " << cnt_init_landmarks
                  << " map points";

        return true;
    }


    bool Frontend::StereoInit() {

        int num_features_left = detectFeature();
        int num_coor_features = findFeatureInRight();

        // 右目特征点比较少，初始化失败
        if(num_coor_features < num_features_init_){
            return false;
        }

        bool build_map_success = buildInitMap();

        if(build_map_success) {
            status_ = FrontendStatus::TRACKING_GOOD;
            if(viewer_) {
                viewer_->addCurrentFrame(current_frame_);
                viewer_->updateMap();
            }
            return true;
        }

        return false;
    }

    int Frontend::trackLastFrame() {
        std::vector<cv::Point2f> kps_last, kps_current;
        for(auto &kp : last_frame_->feature_left_) {
            if(kp->map_point_.lock()) {
                auto mp = kp->map_point_.lock();
                auto px = camera_left_->world2pixel(mp->pos_,current_frame_->pose());
                kps_last.push_back(kp->position_.pt);
                kps_current.push_back(cv::Point2f(px[0],px[1]));
            }
            else {
                kps_last.push_back(kp->position_.pt);
                kps_current.push_back(kp->position_.pt);
            }
        }

        std::vector<uchar> status;
        cv::Mat error;
        cv::calcOpticalFlowPyrLK(
                // todo 你这里直接copy了findright,导致错误
                last_frame_->left_img_, current_frame_->left_img_, kps_last,
                kps_current, status, error, cv::Size(11,11), 3,
                //  TermCriteria https://blog.csdn.net/YMWM_/article/details/117967663
                cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
                cv::OPTFLOW_USE_INITIAL_FLOW // 使用初始值 https://blog.csdn.net/jihengshan/article/details/20644073
        );

        int num_good_pts = 0;

        for(size_t i = 0; i < status.size(); ++i){
            if(status[i]){
                cv::KeyPoint kp(kps_current[i],7);
                Feature::Ptr feat(new Feature(current_frame_,kp));
                feat->map_point_ = last_frame_->feature_left_[i]->map_point_;
                current_frame_->feature_left_.push_back(feat);
                ++num_good_pts;
            }
        }

        LOG(INFO) << "Find " << num_good_pts << " in the last image.";
        return num_good_pts;

    }


    /**
     * 这里其实是优化 Tcw的位姿，通过当前帧所有特征点对应的地图点
     * @tips 这里优化的时候有考虑到不同级别的边，即剔除outline，保证稳定性
     * @return
     */
    int Frontend::estimateCurrentPose() {

        typedef g2o::BlockSolverPL<6,3> BlockSolverType; // solver 源码里面有一个 sparse block matrix
        typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType; // linear solver
        auto solver = new g2o::OptimizationAlgorithmLevenberg(
                g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>())
                );
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);

        /// 顶点就当前位姿
        VertexPose *vertex_pose = new VertexPose();
        vertex_pose->setId(0);
        vertex_pose->setEstimate(current_frame_->pose());
        optimizer.addVertex(vertex_pose);

        Mat33d K = camera_left_->K();


        /// 添加许多边
        int index = 1;
        std::vector<EdgeProjectionPoseOnly *> edges;
        std::vector<Feature::Ptr> features;
        for(size_t i = 0; i < current_frame_->feature_left_.size(); ++i){
            auto mp = current_frame_->feature_left_[i]->map_point_.lock();
            if(mp){
                features.push_back(current_frame_->feature_left_[i]);
                EdgeProjectionPoseOnly *edge = new EdgeProjectionPoseOnly(mp->pos_,K);
                edge->setId(index);
                edge->setVertex(0,vertex_pose);
                edge->setMeasurement(Vec2d(
                        current_frame_->feature_left_[i]->position_.pt.x,
                        current_frame_->feature_left_[i]->position_.pt.y
                ));
                edge->setInformation(Mat22d::Identity());
                edge->setRobustKernel(new g2o::RobustKernelHuber);
                edges.push_back(edge);
                optimizer.addEdge(edge);
                ++index;
            }
        }

        /// 测试位置的优良性
        /// https://blog.csdn.net/lib0000/article/details/116544677
        const double chi12_th = 5.991;
        int cnt_outlier = 0;
        for(int iteration = 0; iteration < 4 ; ++iteration){
            vertex_pose->setEstimate(current_frame_->pose());
            /// 默认优化级别为0的边
            optimizer.initializeOptimization();
            optimizer.optimize(10);

            cnt_outlier = 0;
            for(size_t i = 0; i < edges.size(); ++i){
                auto e = edges[i];
                if(features[i]->is_outlier_){
                    e->computeError();
                }
                if(e->chi2() > chi12_th){
                    features[i]->is_outlier_ = true;
                    e->setLevel(1);
                    ++cnt_outlier;
                }
                else{
                    features[i]->is_outlier_ = false;
                    e->setLevel(0);
                }

                if(iteration == 2){
                    e->setRobustKernel(nullptr);
                }
            }
        }

        LOG(INFO) << "Outlier/Inlier in pose estimating: " << cnt_outlier << "/"
                  << features.size() - cnt_outlier;

        current_frame_->setPose(vertex_pose->estimate());

        LOG(INFO) << "Current Pose = \n" << current_frame_->pose().matrix();

        for(auto &feat : features){
            if(feat->is_outlier_){
                // 将对应的weak_ptr设置为nullptr
                // https://blog.csdn.net/weixin_44980842/article/details/121828485
                feat->map_point_.reset();
                feat->is_outlier_ = false;
            }
        }

        return features.size() - cnt_outlier;

    }


    void Frontend::setObservationsForKeyFrame() {
        for(auto &kp : current_frame_->feature_left_){
            auto mp = kp->map_point_.lock();
            // 虽然addObservation里面有锁，但是这里是必要的，因为这里是测试map_point是否为nullptr
            if(mp) mp->addObservation(kp);
        }
    }


    int Frontend::triangulateNewPoints() {
        std::vector<SE3d> poses{camera_left_->pose(), camera_right_->pose()};
        /// T_w_c;
        SE3d current_cam_pose = current_frame_->pose().inverse();
        int cnt_triangulated_pts = 0;
        for(size_t i = 0; i < current_frame_->feature_left_.size(); ++i){
            if(current_frame_->feature_left_[i]->map_point_.expired() &&
            current_frame_->feature_right_[i] != nullptr) {
                std::vector<Vec3d> points{
                        camera_left_->pixel2camera(toVec2(current_frame_->feature_left_[i]->position_.pt)),
                        camera_right_->pixel2camera(toVec2(current_frame_->feature_right_[i]->position_.pt))
                };

                Vec3d pw = Vec3d::Zero();

                // 其实这里 pw[2] > 0 是可以理解的，因为其实Camera pose是全0，相当于是以第一个相机为原始点
                // 因此世界坐标假设和Camera pose是一致的，即z轴向前，然而点云的坐标在z轴一定大于0;
                if(triangulation(poses,points,pw) && pw[2] > 0) {
                    auto new_map_point = MapPoint::createNewMappoint();
                    new_map_point->pos_ = current_cam_pose * pw;
                    new_map_point->addObservation(current_frame_->feature_left_[i]);
                    new_map_point->addObservation(current_frame_->feature_right_[i]);
                    current_frame_->feature_left_[i]->map_point_ = new_map_point;
                    current_frame_->feature_right_[i]->map_point_ = new_map_point;
                    map_->insertMapPoint(new_map_point);
                    ++cnt_triangulated_pts;
                }

            }
        }
        LOG(INFO) << "new landmarks: " << cnt_triangulated_pts;
        return cnt_triangulated_pts;

    }

    bool Frontend::insertKeyFrame() {
        if(tracking_inliers_ >= num_features_needed_for_keyframe_) return false;

        current_frame_->setKeyFrame();
        map_->insertKeyFrame(current_frame_);

        LOG(INFO) << "Set frame " << current_frame_->id_ << " as keyframe "
                  << current_frame_->keyframe_id_;

        setObservationsForKeyFrame();

        detectFeature();
        findFeatureInRight();

        /// 这个三角化和初始化的三角化是相同的方法，不同的意义，初始化的时候直接并入了三角化过程
        /// 这个三角化是将新追踪到的特征点进行三角化
        triangulateNewPoints();

        backend_->updateMap();

        if(viewer_) viewer_->updateMap();

        return true;

    }

    bool Frontend::Track() {

        // 上帧是good match 那么就利用匀速运动来作为初值
        bool last_frame_exist = false;
        if(last_frame_) {
            last_frame_exist = true;
            current_frame_->setPose(relative_motion_ * last_frame_->pose());
        }
        LOG(INFO) << "last_frame_ = \n" << last_frame_exist;

        LOG(INFO) << "Before Update Pose = \n" << current_frame_->pose().matrix();


        int num_track_last = trackLastFrame();
        tracking_inliers_ = estimateCurrentPose();

        if(tracking_inliers_ > num_features_tracking_) {
            status_ = FrontendStatus::TRACKING_GOOD;
        }
        else if(tracking_inliers_ > num_features_tracking_bad_) {
            status_ = FrontendStatus::TRACKING_BAD;
        }
        else {
            status_ = FrontendStatus::LOST;
        }

        insertKeyFrame();

        // T_c_l = T_c_w * T_l_w.inv
        relative_motion_ = current_frame_->pose() * last_frame_->pose().inverse();

        if(viewer_) viewer_->addCurrentFrame(current_frame_);
        return true;
    }





    bool Frontend::Reset() {
        LOG(INFO) << "Reset is not implemented. ";
        return true;
    }

}