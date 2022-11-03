add_library(
        myslam SHARED
        config.cpp
        dataset.cpp
        camera.cpp
        frame.cpp
        feature.cpp
        mappoint.cpp
        map.cpp
        frontend.cpp
        backend.cpp
        viewer.cpp
        visual_odometry.cpp
)

target_link_libraries(myslam ${THIRD_PARTY_LIBS})

