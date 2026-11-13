cmake_minimum_required(VERSION 3.16)
project(amr_mpc_cpp LANGUAGES C CXX)

# colcon 은 빌드 타입을 비워 둔다 → 최적화 없이 빌드되어 100 Hz 예산 측정이 무의미해진다
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(amr_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# ── DAQP v0.10.3 (MIT, third_party/daqp/LICENSE) — 소스를 같이 빌드한다. 설치·네트워크 불필요
file(GLOB DAQP_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/third_party/daqp/src/*.c)
add_library(daqp_vendored STATIC ${DAQP_SOURCES})
target_include_directories(daqp_vendored PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/third_party/daqp/include)
# 업스트림 기본값(FASTER_MATH ON)과 같은 플래그. 2026-10-22 벤치마크도 이 설정이었다
target_compile_options(daqp_vendored PRIVATE -fassociative-math -fno-signed-zeros -fno-trapping-math -w)
target_link_libraries(daqp_vendored PUBLIC m)

# ── ROS 무관 핵심부: 모델·d 단 지연·condensing·관측기·DAQP 호출, 기준열 보간
add_library(amr_wheel_mpc STATIC src/wheel_mpc.cpp src/reference.cpp)
target_include_directories(amr_wheel_mpc PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(amr_wheel_mpc PUBLIC Eigen3::Eigen PRIVATE daqp_vendored)

# ── 토크 입력 MPC 핵심부 (구조도 v3, 1판 설계). ROS 무관
add_library(amr_torque_mpc STATIC src/torque_mpc.cpp)
target_include_directories(amr_torque_mpc PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(amr_torque_mpc PUBLIC Eigen3::Eigen PRIVATE daqp_vendored)

# ── 노드
add_executable(wheel_mpc_node src/wheel_mpc_node.cpp)
target_link_libraries(wheel_mpc_node amr_wheel_mpc yaml-cpp)
ament_target_dependencies(wheel_mpc_node rclcpp ament_index_cpp amr_interfaces geometry_msgs nav_msgs std_msgs)
# 토크 MPC 노드. 기준열 보간(reference.cpp)은 amr_wheel_mpc 것을 그대로 쓴다
add_executable(torque_mpc_node src/torque_mpc_node.cpp)
target_link_libraries(torque_mpc_node amr_torque_mpc amr_wheel_mpc yaml-cpp)
ament_target_dependencies(torque_mpc_node rclcpp ament_index_cpp amr_interfaces geometry_msgs nav_msgs std_msgs)

# ── 도구: Python 판과의 일치 검증(replay), 폐루프 시뮬·시간 측정(sim)
add_executable(wheel_mpc_replay tools/replay.cpp)
target_link_libraries(wheel_mpc_replay amr_wheel_mpc)
add_executable(wheel_mpc_sim tools/sim.cpp)
target_link_libraries(wheel_mpc_sim amr_wheel_mpc)

install(TARGETS wheel_mpc_node torque_mpc_node wheel_mpc_replay wheel_mpc_sim DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_wheel_mpc test/test_wheel_mpc.cpp)
  target_link_libraries(test_wheel_mpc amr_wheel_mpc)
  ament_add_gtest(test_torque_mpc test/test_torque_mpc.cpp)
  target_link_libraries(test_torque_mpc amr_torque_mpc)
endif()

ament_package()
