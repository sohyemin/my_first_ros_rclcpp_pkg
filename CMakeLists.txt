# cmake_minimum_required > cmake의 최소 요구버젼
cmake_minimum_required(VERSION 3.5)
# 패키지의 이름 항목.
#package.xml의 <name>태그 패키지 이름과 다를 경우 빌드 시 오류가 발생한다.
project(my_first_ros_rclcpp_pkg)

# Default to C++14 | C++ 버전을 기재
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# 컴파일 옵션을 기재
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# 사용자가 만든 패키지가 의존하는 다른 패키지를 먼저 설치하도록 만드는 옵션
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Build

# 빌드 시 참조할 코드와 실행 파일 이름을 지정
# helloworld_publisher 실행 시, helloworld_publisher.cpp이 실행된다.
add_executable(helloworld_publisher src/helloworld_publisher.cpp)
# 해당 라이브러리 및 실행 파일 빌드에 앞서 생성해야 할 의존성이 있는 인터페이스가 있다면 우선적으로 이를 수행한다.
ament_target_dependencies(helloworld_publisher rclcpp std_msgs)

add_executable(helloworld_subscriber src/helloworld_subscriber.cpp)
ament_target_dependencies(helloworld_subscriber rclcpp std_msgs)

# Install
install(TARGETS
  helloworld_publisher
  helloworld_subscriber
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Macro for ament package
ament_package()
