#cmake version
cmake_minimum_required(VERSION 3.1)

if(NOT TARGET ${KITTI_LIB})
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

  # finding dependencies
  find_package(Boost COMPONENTS system filesystem thread REQUIRED)

  #header files
  set(HPP_FILES
    include/kitti/kitti_controller.h
    include/kitti/kitti_custom_objects.h
    include/kitti/kitti_objects_reader.h
    include/kitti/kitti_point_cloud_reader.h
    include/kitti/kitti_vehicle_reader.h
    include/kitti/types.h
  )

  #source files
  set(CPP_FILES
    src/kitti_controller.cpp
    src/kitti_custom_objects.cpp
    src/kitti_objects_reader.cpp
    src/kitti_point_cloud_reader.cpp
    src/kitti_vehicle_reader.cpp
  )

  set(KITTI_DIRS
    libs/kitti
    libs/kitti/include
    lib/kitti
    lib/kitti/include

    CACHE INTERNAL ""
  )
  set(KITTI_LIB ${KITTI_NAME} CACHE INTERNAL "")

  include_directories(include)

  # +++++++++++++++++++++++++++++++++++ ALGEBRAICA +++++++++++++++++++++++++++++++++++
  # adding the root directory of algebraica source tree to your project
  add_subdirectory(lib/algebraica)
  include_directories(${ALGEBRAICA_DIRS})

  # +++++++++++++++++++++++++++++++++++ XML READER +++++++++++++++++++++++++++++++++++
  # adding the root directory of algebraica source tree to your project
  add_subdirectory(lib/ticpp-master)
  include_directories(lib)

  #add every file to the executable
  add_library(${KITTI_NAME}
    ${XML_FILES}
    ${HPP_FILES}
    ${CPP_FILES}
  )

  #indicates which libraries to use in the executable
  target_link_libraries(${KITTI_NAME}
    ${Boost_LIBRARIES}
    ${ALGEBRAICA_LIB}
    ticpp
  )
endif(NOT TARGET ${KITTI_LIB})
