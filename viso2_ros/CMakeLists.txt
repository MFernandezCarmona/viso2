cmake_minimum_required(VERSION 3.5.0)
project(viso2_ros)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(libviso2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
   "msg/VisoInfo.msg"
   DEPENDENCIES std_msgs
)


include_directories(src 
    ${pcl_conversions_INCLUDE_DIRS}
    ${libviso2_INCLUDE_DIRS}
    ${cv_bridge_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

#add_executable(stereo_odometer
#  src/stereo_odometer.cpp)

#ament_target_dependencies(stereo_odometer
#    "libviso2"
#    "rclcpp"
#   "sensor_msgs"
#    "std_msgs"
#    "pcl_msgs"
#    "nav_msgs"
#    "cv_bridge"
#    "message_filters"
#    "image_transport"
#    "image_geometry"
#    "tf2"
#    "pcl_conversions"
#    #"pcl_ros"
#    "OpenCV")


#rosidl_target_interfaces(stereo_odometer
#      viso2_ros "rosidl_typesupport_cpp")

add_executable(mono_odometer
  src/mono_odometer.cpp)

ament_target_dependencies(mono_odometer
  "libviso2"
  "rclcpp"
  "sensor_msgs"
  "std_msgs"
  "nav_msgs"
  "tf2_msgs"
  "std_srvs"
  "geometry_msgs"
  "tf2_geometry_msgs"
  "cv_bridge"
  "message_filters"
  "image_transport"
  "image_geometry"
  "tf2"
  #"pcl_ros"
  "OpenCV")

rosidl_target_interfaces(mono_odometer
      ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(mono_odometer_omnidirectional
  src/mono_odometer_omnidirectional.cpp)

ament_target_dependencies(mono_odometer_omnidirectional
  "libviso2"
  "rclcpp"
  "sensor_msgs"
  "std_msgs"
  "std_srvs"
  "nav_msgs"
  "tf2_msgs"
  "geometry_msgs"
  "tf2_geometry_msgs"
  "cv_bridge"
  "message_filters"
  "image_transport"
  "image_geometry"
  "tf2"
  #"pcl_ros"
  "OpenCV")

rosidl_target_interfaces(mono_odometer_omnidirectional
      ${PROJECT_NAME} "rosidl_typesupport_cpp")

#target_link_libraries(stereo_odometer ${OpenCV_LIBRARIES})

target_link_libraries(mono_odometer ${OpenCV_LIBRARIES})

target_link_libraries(mono_odometer_omnidirectional ${OpenCV_LIBRARIES})

install(TARGETS
  #stereo_odometer
  mono_odometer
  mono_odometer_omnidirectional
DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

