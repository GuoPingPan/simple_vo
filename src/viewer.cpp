#include "../include/myslam/viewer.h"
#include "../include/myslam/feature.h"
#include "../include/myslam/frame.h"

#include <pangolin/pangolin.h>
#include <opencv2/opencv.hpp>


namespace myslam {


    Viewer::Viewer() {
        viewer_thread_ = std::thread(std::bind(&Viewer::threadLoop,this));
    }

    void Viewer::close() {
        viewer_running_ = false;
        viewer_thread_.join();
    }

    void Viewer::addCurrentFrame(Frame::Ptr current_frame) {
        std::unique_lock<std::mutex> lck(viewer_data_mutex_);
        current_frame_ = current_frame;
    }

    void Viewer::updateMap() {
        std::unique_lock<std::mutex> lck(viewer_data_mutex_);
        assert(map_ != nullptr);
        active_keyframes_ = map_->getActiveKeyFrames();
        active_landmarks_ = map_->getActiveMapPoints();

        map_updated_ = true;
    }

    void Viewer::drawFrame(Frame::Ptr frame, const float *color) {
        SE3d  Twc = frame->pose().inverse();
        const float sz = 1.0;
        const int line_width = 2.0;
        const float fx = 400;
        const float fy = 400;
        const float cx = 512;
        const float cy = 384;
        const float width = 1080;
        const float height = 768;

        glPushMatrix();

        Sophus::Matrix4f m = Twc.matrix().template cast<float>();
        glMultMatrixf((GLfloat*)m.data());

        if (color == nullptr) {
            glColor3f(1, 0, 0);
        } else
            glColor3f(color[0], color[1], color[2]);

        glLineWidth(line_width);
        glBegin(GL_LINES);
        /// 8条边，16个顶点，对应着相机平面和归一化平面
        glVertex3f(0, 0, 0);
        glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
        glVertex3f(0, 0, 0);
        glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
        glVertex3f(0, 0, 0);
        glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
        glVertex3f(0, 0, 0);
        glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

        glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
        glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

        glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
        glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

        glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
        glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);

        glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
        glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

        glEnd();
        glPopMatrix();

    }

    void Viewer::followCurrentFrame(pangolin::OpenGlRenderState& vis_camera) {
        SE3d Twc = current_frame_->pose().inverse();
        pangolin::OpenGlMatrix m(Twc.matrix());
        vis_camera.Follow(m, true);
    }

    cv::Mat Viewer::plotFrameImage() {
        cv::Mat img_out;
        /// 单通道变成三通道，但是每个像素值只是单纯的复制
        cv::cvtColor(current_frame_->left_img_, img_out, CV_GRAY2BGR);
        for (size_t i = 0; i < current_frame_->feature_left_.size(); ++i) {
            /// 只画出有landmarks的features
            if (current_frame_->feature_left_[i]->map_point_.lock()) {
                auto feat = current_frame_->feature_left_[i];
                cv::circle(img_out, feat->position_.pt, 2, cv::Scalar(0, 250, 0),
                           2);
            }
        }
        return img_out;
    }

    void Viewer::drawMapPoint() {
        const float red[3] = {1.0, 0, 0};
        for (auto& kf : active_keyframes_) {
            drawFrame(kf.second, red);
        }

        glPointSize(2);
        glBegin(GL_POINTS);
        for (auto& landmark : active_landmarks_) {
            auto pos = landmark.second->pos();
            glColor3f(red[0], red[1], red[2]);
            glVertex3d(pos[0], pos[1], pos[2]);
        }
        glEnd();
    }



    void Viewer::threadLoop() {
        pangolin::CreateWindowAndBind("myslam", 1024, 768);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        pangolin::OpenGlRenderState vis_camera(
                pangolin::ProjectionMatrix(1024, 768, 400, 400, 512, 384, 0.1, 1000),
                pangolin::ModelViewLookAt(0, -5, -10, 0, 0, 0, 0.0, -1.0, 0.0));

        // Add named OpenGL viewport to window and provide 3D Handler
        pangolin::View& vis_display =
                pangolin::CreateDisplay()
                        .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
                        .SetHandler(new pangolin::Handler3D(vis_camera));

        // rgb
        const float blue[3] = {0, 0, 1};
        const float green[3] = {0, 1, 0};

        while (!pangolin::ShouldQuit() && viewer_running_) {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
            vis_display.Activate(vis_camera);

            std::unique_lock<std::mutex> lock(viewer_data_mutex_);
            if (current_frame_) {
                // 画出当前帧
                drawFrame(current_frame_, green);
                // 显示视角跟随当前帧
                followCurrentFrame(vis_camera);

                cv::Mat img = plotFrameImage();
                cv::imshow("image", img);
                cv::waitKey(1);
            }

            if (map_) {
                drawMapPoint();
            }

            pangolin::FinishFrame();
            usleep(5000);
        }

        LOG(INFO) << "Stop viewer";
    }


} // namespace myslam