cmake_minimum_required(VERSION 3.8)
project(robotpadi)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

# add_executable(test_node script/smc_controller.py)
# ament_target_dependencies(test_node rclpy)

# ament_python_install_package(${PROJECT_NAME})
# install(TARGETS
#     test_node
#     DESTINATION lib/${PROJECT_NAME}/
# )

# Install direktori yang berisi resource
install(DIRECTORY
  launch
  config
  worlds
  urdf
  rviz
  models
  meshes
  DESTINATION share/${PROJECT_NAME}/
)

# =======================================================
# # === TAMBAHKAN BLOK PENTING INI UNTUK PYTHON SCRIPT ===
# # =======================================================
# install(PROGRAMS
#   # Daftarkan semua skrip Python yang ingin Anda jadikan node di sini
#   # "script/imu_covariance_fixer.py"
#   script/smc_controller.py
#   DESTINATION lib/${PROJECT_NAME}
# )
# =======================================================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()




# cmake_minimum_required(VERSION 3.8)
# project(robotpadi)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # find dependencies
# find_package(ament_cmake REQUIRED)
# install(DIRECTORY
#   launch
#   config
#   # description
#   worlds
#   urdf
#   rviz
#   models
#   meshes
#   DESTINATION share/${PROJECT_NAME}/
# )

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# ament_package()


