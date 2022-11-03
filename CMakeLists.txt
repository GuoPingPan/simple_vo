CMAKE_MINIMUM_REQUIRED(VERSION 3.0)
PROJECT(VO)

SET(CMAKE_BUILD_TYPE "Release")
# 这里的CMAKE_CXX_FLAGS的选项还需要了解
SET(CMAKE_CXX_FLAGS "-std=c++14 -Wall")
SET(CMAKE_CXX_FLAGS_RELEASE "--std=c++14 -O3 -fopenmp -pthread")

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


# find libraries
FIND_PACKAGE(OpenCV 3 REQUIRED)
FIND_PACKAGE(Pangolin REQUIRED)
FIND_PACKAGE(Sophus REQUIRED)
FIND_PACKAGE(G2O REQUIRED)
FIND_PACKAGE(Glog REQUIRED)
FIND_PACKAGE(GTest REQUIRED)
FIND_PACKAGE(GFlags REQUIRED)
FIND_PACKAGE(CSparse REQUIRED)

INCLUDE_DIRECTORIES(
        /usr/include/eigen3
        ${OpenCV_INCLUDE_DIRS}
        ${Pangolin_INCLUDE_DIRS}
        ${Sophus_INCLUDE_DIRS}
        ${G2O_INCLUDE_DIR}
        ${GLOG_INCLUDE_DIRS}
        ${GFLAGS_INCLUDE_DIRS}
        ${CSPARSE_INCLUDE_DIR}
)

SET(THIRD_PARTY_LIBS
        ${OpenCV_LIBS}
        ${Sophus_LIBRARIES}
        ${Pangolin_LIBRARIES} GL GLU GLEW glut
        g2o_core g2o_stuff g2o_types_sba g2o_solver_csparse g2o_csparse_extension
        ${GTEST_BOTH_LIBRARIES}
        ${GLOG_LIBRARIES}
        ${GFLAGS_LIBRARIES}
        pthread
        ${CSPARSE_LIBRARY}
        Sophus::Sophus
        )



INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include/myslam)

add_subdirectory(src)
add_subdirectory(test)
add_subdirectory(app)

add_dependencies(run_kitti_stereo myslam)

FILE(GLOB Header ${PROJECT_SOURCE_DIR}/include/myslam/*.h)
#MESSAGE(STATUS "header file ${Header}")

INSTALL(TARGETS run_kitti_stereo myslam
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
)
INSTALL(FILES ${Header}
        DESTINATION include/myslam
)

FIND_PACKAGE(VO REQUIRED)