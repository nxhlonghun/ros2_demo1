cmake_minimum_required(VERSION 3.8)
project(demo_cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)#查找未找头文件
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(include)#包含include目录


add_executable(learn_thread src/learn_thread.cpp)
add_executable(test_node src/test.cpp)
add_executable(person_node src/person_node.cpp)

#target_include_directories(test_node PUBLIC ${rclcpp_INCLUDE_DIRS})#对可执行文件添加头文件
#target_link_libraries(test_node ${rclcpp_LIBRARIES})#对可执行文件添加库文件
ament_target_dependencies(test_node rclcpp)#等同于上面两行
ament_target_dependencies(person_node rclcpp)

install(TARGETS test_node person_node learn_thread
  DESTINATION lib/${PROJECT_NAME}
)

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
