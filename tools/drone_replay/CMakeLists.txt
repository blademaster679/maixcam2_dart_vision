cmake_minimum_required(VERSION 3.13)
project(drone_green_replay LANGUAGES CXX)
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc)
find_package(Threads REQUIRED)
add_library(drone_green_core SHARED
    drone_core.cpp
    ../video_replay/maix_image_opencv.cpp
    ../../main/src/nv21_pipeline.cpp
    ../../main/src/config.cpp
    ../../main/src/green_detector.cpp
    ../../main/src/green_detector_core.cpp
    ../../main/src/target_fusion.cpp
    ../../main/src/target_json.cpp
    ../../main/src/visual_motion.cpp)
target_compile_features(drone_green_core PRIVATE cxx_std_17)
target_include_directories(drone_green_core PRIVATE ../video_replay ../../main/include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(drone_green_core PRIVATE ${OpenCV_LIBS} Threads::Threads)
target_compile_options(drone_green_core PRIVATE -Wall -Wextra -Wpedantic)
if(UNIX)
  target_link_options(drone_green_core PRIVATE -Wl,--no-undefined)
endif()
