cmake_minimum_required(VERSION 3.8)
project(amr_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObservationProvenance.msg"
  "msg/TrackedObservation.msg"
  "msg/TrackedObservationArray.msg"
  "msg/DetectedObject.msg"
  "msg/DetectedObjectArray.msg"
  "msg/TrackedObject.msg"
  "msg/TrackedObjectArray.msg"
  "msg/ImmModeState.msg"
  "msg/ImmTrackedObject.msg"
  "msg/ImmTrackedObjectArray.msg"
  "msg/ObjectPrediction.msg"
  "msg/PlanarReference.msg"
  "msg/PlanarReferenceArray.msg"
  "msg/ObjectPredictionArray.msg"
  "msg/GaussianPrediction2D.msg"
  "msg/MotionModePrediction.msg"
  "msg/MultiModalObjectPrediction.msg"
  "msg/MultiModalObjectPredictionArray.msg"
  "msg/RiskObject.msg"
  "msg/RiskLayeredModel.msg"
  "msg/TraversableMap.msg"
  "msg/SemanticObservation.msg"
  "msg/SemanticObservationArray.msg"
  "msg/SemanticObject.msg"
  "msg/SemanticObjectArray.msg"
  "msg/SemanticRelation.msg"
  "msg/WheelTorqueCommand.msg"
  "msg/WheelStates.msg"
  "srv/SetObjectAlias.srv"
  "srv/PlanObjectApproach.srv"
  "action/NavigateToObject.action"
  DEPENDENCIES std_msgs geometry_msgs vision_msgs nav_msgs builtin_interfaces action_msgs
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
