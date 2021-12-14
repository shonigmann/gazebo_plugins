cmake_minimum_required(VERSION 3.5)
project(gazebo_plugins)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Wno-inconsistent-missing-override")
endif()

if(WIN32)
  add_compile_definitions(
    # For math constants
    _USE_MATH_DEFINES
    # Minimize Windows namespace collision
    NOMINMAX
    WIN32_LEAN_AND_MEAN
  )
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_msgs REQUIRED)

link_directories(${gazebo_dev_LIBRARY_DIRS})

option(ENABLE_PROFILER "Enable Ignition Profiler" FALSE)
if(ENABLE_PROFILER)
  find_package(ignition-common3 QUIET COMPONENTS profiler)
  if(ignition-common3_FOUND)
    add_definitions("-DIGN_PROFILER_ENABLE=1" "-DIGN_PROFILER_REMOTERY=1")
    message(STATUS "Profiler is active")
  else()
    message(WARNING "Can't find Ignition common3. Profiler will not be actived")
    add_definitions("-DIGN_PROFILER_ENABLE=0" "-DIGN_PROFILER_REMOTERY=0")
  endif()
endif()

# gazebo_gui_camera_clip
add_library(gazebo_gui_camera_clip SHARED
  src/gazebo_gui_camera_clip.cpp
)
target_include_directories(gazebo_gui_camera_clip PUBLIC include)
ament_target_dependencies(gazebo_gui_camera_clip
  "gazebo_dev"
)
ament_export_libraries(gazebo_gui_camera_clip)
if(ENABLE_PROFILER)
  target_include_directories(gazebo_gui_camera_clip PUBLIC ${ignition-common3_INCLUDE_DIRS})
  target_link_libraries(gazebo_gui_camera_clip ${ignition-common3_LIBRARIES})
endif()

# gazebo_log_state
add_library(gazebo_log_state SHARED
  src/gazebo_log_state.cpp
)
target_include_directories(gazebo_log_state PUBLIC include)
ament_target_dependencies(gazebo_log_state
  "gazebo_dev"
)
ament_export_libraries(gazebo_log_state)
if(ENABLE_PROFILER)
  target_include_directories(gazebo_log_state PUBLIC ${ignition-common3_INCLUDE_DIRS})
  target_link_libraries(gazebo_log_state ${ignition-common3_LIBRARIES})
endif()

# gazebo_set_initial_vel
add_library(gazebo_set_initial_vel SHARED
  src/gazebo_set_initial_vel.cpp
)
target_include_directories(gazebo_set_initial_vel PUBLIC include)
ament_target_dependencies(gazebo_set_initial_vel
  "gazebo_dev"
)
ament_export_libraries(gazebo_set_initial_vel)
if(ENABLE_PROFILER)
  target_include_directories(gazebo_set_initial_vel PUBLIC ${ignition-common3_INCLUDE_DIRS})
  target_link_libraries(gazebo_set_initial_vel ${ignition-common3_LIBRARIES})
endif()

ament_export_include_directories(include)
ament_export_dependencies(gazebo_dev)
ament_export_dependencies(gazebo_msgs)

#if(BUILD_TESTING)
#  add_subdirectory(test)
#  find_package(ament_lint_auto REQUIRED)
#  ament_lint_auto_find_test_dependencies()
#endif()

if(NOT WIN32)
  if(NOT APPLE)
    set(
      AMENT_CMAKE_ENVIRONMENT_HOOKS_DESC_gazebo_plugins
      "prepend-non-duplicate;LD_LIBRARY_PATH;${GAZEBO_PLUGIN_PATH}")
  else()
    set(
      AMENT_CMAKE_ENVIRONMENT_HOOKS_DESC_gazebo_plugins
      "prepend-non-duplicate;DYLD_LIBRARY_PATH;${GAZEBO_PLUGIN_PATH}")
  endif()
endif()
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/gazebo_plugins.sh.in")

ament_package()


install(DIRECTORY include/
        DESTINATION include)

install(TARGETS
    gazebo_gui_camera_clip
    gazebo_log_state
    gazebo_set_initial_vel
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

#install(DIRECTORY
#  worlds
#  DESTINATION share/${PROJECT_NAME}/
#)
