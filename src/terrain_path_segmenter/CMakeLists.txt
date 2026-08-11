cmake_minimum_required(VERSION 3.8)
project(terrain_path_segmenter)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  terrain_path_segmenter/terrain_path_visualizer.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME terrain_path_visualizer
)

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_segmentation test/test_segmentation.py)
  ament_add_pytest_test(test_progress test/test_progress.py)
endif()

ament_package()
