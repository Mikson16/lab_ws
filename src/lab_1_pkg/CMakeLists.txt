cmake_minimum_required(VERSION 3.8)
project(lab_1_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS nodes/nodo_graficador_perfect.py nodes/pose_loader.py  nodes/coordenadas.txt nodes/dead_reckoning_nav_perfect.py nodes/dead_reckoning_nav.py nodes/dead_reckoning_nav_obstacle.py 
                 nodes/obstacle_detector.py nodes/nodo_graficador.py nodes/publisher_vel.py nodes/key_manager.py DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
