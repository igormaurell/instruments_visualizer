cmake_minimum_required(VERSION 2.8.3)
project(instruments_visualizer)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  genmsg
  message_generation
  std_msgs
  sensor_msgs
)

find_package (OpenCV REQUIRED)

find_package (Eigen3 REQUIRED)

add_message_files(
  FILES
  CircuitState.msg
  ManometerState.msg
  ValveState.msg
)

add_service_files(
  FILES
  VisualizeCircuit.srv
  VisualizeManometer.srv
  VisualizeValve.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
 CATKIN_DEPENDS roscpp message_runtime std_msgs
)

include_directories(
 lib/CvWorks/Core/Vision
 lib/CvWorks/Core/FieldVision
 lib/CvWorks/Components/VisionSensors
 lib/CvWorks/Components/VisionImplementationCv
 lib/CvWorks/Libs/CsvUtil
 ${catkin_INCLUDE_DIRS}
)

link_directories(
 lib/CvWorks/Components/VisionSensors
 lib/CvWorks/Components/VisionImplementationCv
)

add_executable(circuit_visualizer_node src/circuit_visualizer_node.cpp)
target_link_libraries(circuit_visualizer_node
 ${catkin_LIBRARIES}
 VisionSensors
 VisionImplementationCv
 ${OpenCV_LIBS}
 ${Eigen_LIBS}
)

add_executable(manometer_visualizer_node src/manometer_visualizer_node.cpp)
target_link_libraries(manometer_visualizer_node
 ${catkin_LIBRARIES}
 VisionSensors
 VisionImplementationCv
 ${OpenCV_LIBS}
 ${Eigen_LIBS}
)

add_executable(valve_visualizer_node src/valve_visualizer_node.cpp)
target_link_libraries(valve_visualizer_node
 ${catkin_LIBRARIES}
 VisionSensors
 VisionImplementationCv
 ${OpenCV_LIBS}
 ${Eigen_LIBS}
)
