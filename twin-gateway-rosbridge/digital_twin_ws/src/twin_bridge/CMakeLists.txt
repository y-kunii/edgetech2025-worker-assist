cmake_minimum_required(VERSION 3.8)
project(twin_bridge)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# msg/srv をここに列挙
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SimpleMsg.msg"
  "msg/DetectedObject.msg"
  "msg/DetectedObjects.msg"
  # "srv/YourSrv.srv"  # 追加があれば列挙
  DEPENDENCIES std_msgs geometry_msgs
)

# Pythonノードは「スクリプト」としてインストール
install(PROGRAMS
  scripts/service_server.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
