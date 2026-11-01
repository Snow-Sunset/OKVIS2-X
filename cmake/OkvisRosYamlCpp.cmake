# System ROS binaries must use the YAML library from their own vendor/system
# installation. With Conda active, a missing yaml_cpp_vendor fallback directory
# otherwise makes find_package select Conda's yaml-cpp. Its RPATH then causes
# unrelated system libraries (GDAL/HDF5/RViz) to pick an incompatible libcurl.
find_package(yaml_cpp_vendor REQUIRED)

if(CMAKE_SYSTEM_NAME STREQUAL "Linux"
    AND NOT EXISTS "${yaml-cpp_DIR}/yaml-cpp-config.cmake"
    AND NOT EXISTS "${yaml-cpp_DIR}/yaml-cppConfig.cmake")
  find_file(OKVIS_ROS_YAML_CPP_CONFIG
    NAMES yaml-cpp-config.cmake yaml-cppConfig.cmake
    PATHS
      "/usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}/cmake/yaml-cpp"
      "/usr/lib64/cmake/yaml-cpp"
      "/usr/lib/cmake/yaml-cpp"
      "/usr/share/cmake/yaml-cpp"
    NO_DEFAULT_PATH)
  if(OKVIS_ROS_YAML_CPP_CONFIG)
    get_filename_component(_okvis_ros_yaml_cpp_dir "${OKVIS_ROS_YAML_CPP_CONFIG}" DIRECTORY)
    # Repair a cached Conda path too, without removing the user's build tree.
    set(yaml-cpp_DIR "${_okvis_ros_yaml_cpp_dir}" CACHE PATH "ROS-compatible yaml-cpp config" FORCE)
    # CMP0126 NEW preserves the invalid normal variable set by the vendor.
    # Set both scopes so it cannot shadow the repaired cache entry.
    set(yaml-cpp_DIR "${_okvis_ros_yaml_cpp_dir}")
    unset(_okvis_ros_yaml_cpp_dir)
  endif()
endif()

# Import before RViz/rosbag so both share this target and its library location.
find_package(yaml-cpp REQUIRED CONFIG)
message(STATUS "OKVIS ROS yaml-cpp: ${yaml-cpp_DIR}")
