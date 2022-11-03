#include "visual_odometry.h"
#include <gflags/gflags.h>

DEFINE_string(config_file, "../config/default.yaml", "config file path");
DEFINE_string(gtest_filter, "", "");
DEFINE_string(gtest_color, "", "");

int main(int argc,char **argv){

    google::ParseCommandLineFlags(&argc, &argv, true);

    myslam::VisualOdometry::Ptr vo(new myslam::VisualOdometry(fLS::FLAGS_config_file));

    assert(vo->init() == true);

    vo->run();
    

    return 0;
}