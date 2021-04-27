#cmake version
cmake_minimum_required(VERSION 3.1)

if(NOT TARGET ${ALGEBRAICA_LIB})
  #project name
  set(ALGEBRAICA_NAME Algebraica)
  project(${ALGEBRAICA_NAME})

  # As moc files are generated in the binary dir, tell CMake
  # to always look for includes there:
  set(CMAKE_INCLUDE_CURRENT_DIR ON)
  # compatibility with c++14
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)

  #header files
  set(HPP_FILES
    include/algebraica/algebraica.h
    include/algebraica/linear.h
    include/algebraica/mat3.h
    include/algebraica/mat4.h
    include/algebraica/quaternion.h
    include/algebraica/vec2.h
    include/algebraica/vec3.h
    include/algebraica/vec4.h
  )

  #source files
  set(CPP_FILES
    src/linear.cpp
    src/mat3.cpp
    src/mat4.cpp
    src/quaternion.cpp
    src/vec2.cpp
    src/vec3.cpp
    src/vec4.cpp
  )

  include_directories(include)

  set(ALGEBRAICA_DIRS
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include

    CACHE INTERNAL ""
  )
  set(ALGEBRAICA_LIB ${ALGEBRAICA_NAME} CACHE INTERNAL "")

  #add every file to the executable
  add_library(${ALGEBRAICA_NAME}
    ${HPP_FILES}
    ${CPP_FILES}
  )

  set_target_properties(${ALGEBRAICA_NAME} PROPERTIES LINKER_LANGUAGE CXX)
endif(NOT TARGET ${ALGEBRAICA_LIB})
