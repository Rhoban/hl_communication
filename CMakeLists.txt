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
  proto/captain.proto
  proto/intention.proto
  proto/perception.proto
  proto/position.proto
  proto/team_play.proto
  proto/wrapper.proto
)
set_source_files_properties(${PROTO_SOURCES} ${PROTO_HEADERS} PROPERTIES GENERATED TRUE)


set(ALL_SOURCES 
  src/hl_communication/robot_msg_utils.cpp
  src/hl_communication/udp_broadcast.cpp
  src/hl_communication/udp_message_manager.cpp
#  src/hl_communication/invert.cpp
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

option(BUILD_TEAMPLAY_BIN "Building teamplay binaries" ON)

# Generate Python protobuf bindings into python/hl_communication_proto/
set(HL_PYTHON_OUT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/python/hl_communication_proto)

add_custom_target(hl_communication_python_proto ALL
    COMMAND ${CMAKE_COMMAND} -E make_directory ${HL_PYTHON_OUT_DIR}
    COMMAND ${PROTOBUF_PROTOC_EXECUTABLE}
            --proto_path=${CMAKE_CURRENT_SOURCE_DIR}/proto
            --python_out=${HL_PYTHON_OUT_DIR}
            ${CMAKE_CURRENT_SOURCE_DIR}/proto/captain.proto
            ${CMAKE_CURRENT_SOURCE_DIR}/proto/intention.proto
            ${CMAKE_CURRENT_SOURCE_DIR}/proto/perception.proto
            ${CMAKE_CURRENT_SOURCE_DIR}/proto/position.proto
            ${CMAKE_CURRENT_SOURCE_DIR}/proto/team_play.proto
            ${CMAKE_CURRENT_SOURCE_DIR}/proto/wrapper.proto
    COMMAND bash -c "sed -i 's/^import \\([a-z_]*_pb2\\) as /from hl_communication_proto import \\1 as /' ${HL_PYTHON_OUT_DIR}/*_pb2.py"
    COMMAND ${CMAKE_COMMAND} -E touch ${HL_PYTHON_OUT_DIR}/__init__.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Generating Python protobuf bindings for hl_communication"
    VERBATIM
)

if (BUILD_TEAMPLAY_BIN)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/tools)
 # add_executable(server_example examples/server_example.cpp)
 # target_link_libraries(server_example ${PROJECT_NAME} )

 # add_executable(client_example examples/client_example.cpp)
 # target_link_libraries(client_example ${PROJECT_NAME} )

  add_executable(tp_dump_msg examples/tp_dump_msg.cpp)
  target_link_libraries(tp_dump_msg ${PROJECT_NAME} rhoban_utils)
endif()
