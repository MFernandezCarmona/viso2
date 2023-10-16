cmake_minimum_required(VERSION 3.5.0)
project(libviso2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

add_library(${PROJECT_NAME}
  libviso2/src/filter.cpp
  libviso2/src/matcher.cpp
  libviso2/src/matrix.cpp
  libviso2/src/reconstruction.cpp
  libviso2/src/triangle.cpp
  libviso2/src/viso.cpp
  libviso2/src/viso_mono.cpp
  libviso2/src/viso_mono_omnidirectional.cpp
  libviso2/src/viso_stereo.cpp)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY ${PROJECT_NAME}/src/ DESTINATION include
        FILES_MATCHING PATTERN "*.h")

install(DIRECTORY ${PROJECT_NAME}/src/ DESTINATION include
        FILES_MATCHING PATTERN "*.hpp")

ament_export_include_directories(include)

ament_export_libraries(${PROJECT_NAME})


ament_package()
