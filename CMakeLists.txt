#cmake version
cmake_minimum_required(VERSION 3.1)

#project name
project(Kitti)
set(KITTI_NAME Kitti)

# As moc files are generated in the binary dir, tell CMake
# to always look for includes there:
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# compatibility with c++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

#header files
set(HPP_FILES
  includes/kitti_point_cloud_reader.h
)

#source files
set(CPP_FILES
  src/kitti_point_cloud_reader.cpp
)

set(KITTI_DIRS
  libs/kitti
  libs/kitti/includes

  PARENT_SCOPE
)
set(KITTI_LIB ${KITTI_NAME} PARENT_SCOPE)

#add every file to the executable
add_library(${KITTI_NAME}
  ${HPP_FILES}
  ${CPP_FILES}
)
