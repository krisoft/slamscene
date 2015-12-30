cmake_minimum_required(VERSION 2.8.7)
project(lsd_slam_core)

set(CMAKE_MODULE_PATH   ${PROJECT_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})

find_package(Eigen3 REQUIRED)
find_package(SuiteParse REQUIRED) # Apparently needed by g2o
find_package(g2o REQUIRED)
find_package(OpenCV REQUIRED)
find_package(gflags REQUIRED)
find_package(glog REQUIRED)
find_package(HDF5 COMPONENTS CXX)
FIND_PACKAGE(Boost 1.59 COMPONENTS thread system REQUIRED)

set(CMAKE_CXX_FLAGS
   "${CMAKE_CXX_FLAGS} -std=c++11"
)

set(lsd_SOURCE_FILES
  ${PROJECT_SOURCE_DIR}/src/DataStructures/Frame.cpp
  ${PROJECT_SOURCE_DIR}/src/DataStructures/FramePoseStruct.cpp
  ${PROJECT_SOURCE_DIR}/src/DataStructures/FrameMemory.cpp
  ${PROJECT_SOURCE_DIR}/src/SlamSystem.cpp
  ${PROJECT_SOURCE_DIR}/src/LiveSLAMWrapper.cpp
  ${PROJECT_SOURCE_DIR}/src/DepthEstimation/DepthMap.cpp
  ${PROJECT_SOURCE_DIR}/src/DepthEstimation/DepthMapPixelHypothesis.cpp
  ${PROJECT_SOURCE_DIR}/src/util/globalFuncs.cpp
  ${PROJECT_SOURCE_DIR}/src/util/SophusUtil.cpp
  ${PROJECT_SOURCE_DIR}/src/util/settings.cpp
  ${PROJECT_SOURCE_DIR}/src/util/Undistorter.cpp
  ${PROJECT_SOURCE_DIR}/src/util/FrameReader.cpp
  ${PROJECT_SOURCE_DIR}/src/Tracking/Sim3Tracker.cpp
  ${PROJECT_SOURCE_DIR}/src/Tracking/Relocalizer.cpp
  ${PROJECT_SOURCE_DIR}/src/Tracking/SE3Tracker.cpp
  ${PROJECT_SOURCE_DIR}/src/Tracking/TrackingReference.cpp
  ${PROJECT_SOURCE_DIR}/src/IOWrapper/Timestamp.cpp
  ${PROJECT_SOURCE_DIR}/src/GlobalMapping/FabMap.cpp
  ${PROJECT_SOURCE_DIR}/src/GlobalMapping/KeyFrameGraph.cpp
  ${PROJECT_SOURCE_DIR}/src/GlobalMapping/g2oTypeSim3Sophus.cpp
  ${PROJECT_SOURCE_DIR}/src/GlobalMapping/TrackableKeyFrameSearch.cpp
)
set(SOURCE_FILES
  ${lsd_SOURCE_FILES}
  ${PROJECT_SOURCE_DIR}/src/IOWrapper/OpenCV/ImageDisplay_OpenCV.cpp
)

include_directories(
  ${PROJECT_SOURCE_DIR}/src
  ${EIGEN3_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}/thirdparty/Sophus
  ${Boost_INCLUDE_DIR}
  ${GLOG_INCLUDE_DIRS}
  ${HDF5_INCLUDE_DIRS}
)

add_library(lsdslam SHARED ${SOURCE_FILES})
target_link_libraries(lsdslam
  g2o_core g2o_stuff g2o_solver_csparse g2o_csparse_extension g2o_types_sim3 g2o_types_sba
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
  ${GLOG_LIBRARIES} 
  ${HDF5_CXX_LIBRARIES}  ${HDF5_LIBRARIES}
)


add_executable(dataset_slam src/main_on_images.cpp)
target_link_libraries(dataset_slam lsdslam gflags ${GLOG_LIBRARIES} ${HDF5_CXX_LIBRARIES}  ${HDF5_LIBRARIES})