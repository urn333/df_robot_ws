cmake_minimum_required(VERSION 3.14)
project(df_robot_head_kits)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_library(NVBUF_UTILS_LIB nvbuf_utils)
find_library(NVJPEG_LIB nvjpeg)
find_library(NVIDIA_ENCODE_LIB nvidia-encode)

if(NOT NVBUF_UTILS_LIB)
    set(NVBUF_UTILS_LIB "/usr/lib/aarch64-linux-gnu/tegra/libnvbuf_utils.so.1.0.0")
endif()
if(NOT NVJPEG_LIB)
    set(NVJPEG_LIB "/usr/lib/aarch64-linux-gnu/tegra/libnvjpeg.so")
endif()


# Include directories for BionicEyes SDK
include_directories(
	/usr/Evo_BionicEyes/include
	/home/be/Desktop/workspace/BionicEyes/dependencies/opencv-3.4.11/tegra/arm64/2004/include
)
# Add executable
add_executable(bionic_ctrl_node src/bionic_ctrl_node.cpp)

# Link libraries
ament_target_dependencies(bionic_ctrl_node
  rclcpp
  std_msgs
)

# Link BionicEyes SDK libraries
target_link_libraries(bionic_ctrl_node
 /usr/Evo_BionicEyes/lib_arm/libevo_be_Device_6A_shared_2004.so
 /usr/Evo_BionicEyes/lib_arm/libevo_be_Communication_shared_2004.so
 /usr/Evo_BionicEyes/dependencies/opencv-3.4.11/tegra/arm64/2004/lib/libopencv_world.so
 ${NVBUF_UTILS_LIB}
 ${NVJPEG_LIB}
 #${NVIDIA_ENCODE_LIB}
)
# Install targets
install(TARGETS
  bionic_ctrl_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
