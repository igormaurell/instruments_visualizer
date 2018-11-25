cmake_minimum_required(VERSION 2.8.3)
project(instruments_visualizer)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  genmsg
  message_generation
  std_msgs
  sensor_msgs
  cv_bridge
)

find_package (OpenCV REQUIRED)

add_message_files(
  FILES
  PainelState.msg
  ManometerState.msg
  ValveState.msg
)

add_service_files(
  FILES
  VisualizePainel.srv
  VisualizeManometer.srv
  VisualizeValve.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS roscpp message_generation std_msgs sensor_msgs cv_bridge
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_library(CircleDetector
 src/CircleDetector/circle_detector.cpp
)
target_link_libraries(CircleDetector
 ${OpenCV_LIBS}
)

add_library(AnalogMeterDetector
 src/AnalogMeterDetector/analog_meter_detector.cpp
)
target_link_libraries(AnalogMeterDetector
 CircleDetector
 ${OpenCV_LIBS}
)

add_library(ValveDetector
 src/ValveDetector/valve_detector.cpp
)
target_link_libraries(ValveDetector
 ${OpenCV_LIBS}
)


add_executable(painel_visualizer_node src/painel_visualizer_node.cpp)
target_link_libraries(painel_visualizer_node
 ${catkin_LIBRARIES}
 ${OpenCV_LIBS}
)

add_executable(manometer_visualizer_node src/manometer_visualizer_node.cpp)
target_link_libraries(manometer_visualizer_node
 CircleDetector
 AnalogMeterDetector
 ${catkin_LIBRARIES}
 ${OpenCV_LIBS}
)

add_executable(valve_visualizer_node src/valve_visualizer_node.cpp)
target_link_libraries(valve_visualizer_node
 ValveDetector
 ${catkin_LIBRARIES}
 ${OpenCV_LIBS}
)
