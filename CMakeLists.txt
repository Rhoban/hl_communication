cmake_minimum_required(VERSION 3.16.3)
project(hl_communication)

include(FindProtobuf)
find_package(Protobuf REQUIRED)
find_package(OpenCV 4.2.0 REQUIRED)
find_package(jsoncpp REQUIRED)

#Enable C++17 / for coverage add --coverage -fprofile-abs-path
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -std=c++17")

# Protobuf generate files with unused parameters
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-parameter")

protobuf_generate_cpp(PROTO_SOURCES PROTO_HEADERS ${PROTOBUF_MESSAGES}
  proto/camera.proto
  proto/capabilities.proto
  proto/captain.proto
  proto/game_controller.proto
  proto/intention.proto
  proto/labelling.proto
  proto/perception.proto
  proto/position.proto
  proto/robot_estimation.proto
  proto/team_play.proto
  proto/wrapper.proto
)
set_source_files_properties(${PROTO_SOURCES} ${PROTO_HEADERS} PROPERTIES GENERATED TRUE)

set(SOURCES_DIRECTORIES
  src/hl_communication
  )

set(ALL_SOURCES 
  src/hl_communication/game_controller_utils.cpp
  src/hl_communication/labelling_utils.cpp
  src/hl_communication/message_manager.cpp
  src/hl_communication/robot_msg_utils.cpp
  src/hl_communication/udp_broadcast.cpp
  src/hl_communication/udp_message_manager.cpp
  src/hl_communication/utils.cpp
)

add_library (${PROJECT_NAME} SHARED 
  ${PROTO_SOURCES} 
  ${PROTO_HEADERS}
  ${ALL_SOURCES}
)
target_link_libraries(${PROJECT_NAME} PUBLIC jsoncpp_lib Eigen3::Eigen protobuf::libprotobuf ${OpenCV_LIBS})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  ${OpenCV_INCLUDE_DIRS}
  ${PROTOBUF_INCLUDE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}/..
)

option(BUILD_HL_COMMUNICATION_EXAMPLES "Building hl_communication examples" OFF)

if (BUILD_HL_COMMUNICATION_EXAMPLES)
  add_executable(server_example examples/server_example.cpp)
  target_link_libraries(server_example ${PROJECT_NAME} )

  add_executable(client_example examples/client_example.cpp)
  target_link_libraries(client_example ${PROJECT_NAME} )
endif()
