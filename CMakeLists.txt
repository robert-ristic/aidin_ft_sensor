cmake_minimum_required(VERSION 3.8)
project(aidin_ft_sensor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Use modern C++
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Find SOEM library
find_library(SOEM_LIBRARY soem /usr/lib)

# Define executable
add_executable(aidin_ft_sensor_node src/aidin_ft_node.cpp)

ament_target_dependencies(aidin_ft_sensor_node
        rclcpp
        std_msgs
        geometry_msgs
)

# Link libraries
target_link_libraries(aidin_ft_sensor_node ${SOEM_LIBRARY})

# Install executable
install(TARGETS
        aidin_ft_sensor_node
        DESTINATION lib/${PROJECT_NAME}
)

# Testing (optional)
if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()