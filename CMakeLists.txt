cmake_minimum_required(VERSION 2.8)
project(hl_communication)

include(FindProtobuf)
find_package(Protobuf REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED
  eigen)

#Enable C++17
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -std=c++17")

# Protobuf generate files with unused parameters
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-parameter")

set (PROTOBUF_MESSAGES
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

# Handling generated headers is based on Dirk Thomas answer on topic:
# - https://answers.ros.org/answers/123416/revisions/
catkin_destinations()
file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION})

protobuf_generate_cpp(PROTO_SOURCES PROTO_HEADERS ${PROTOBUF_MESSAGES})

set_source_files_properties(${PROTO_SOURCES} ${PROTO_HEADERS} PROPERTIES GENERATED TRUE)

set(PROTO_DUMMY_FILE proto_generation)

add_custom_command(
  OUTPUT ${PROTO_DUMMY_FILE}
  COMMAND cp ${PROTO_HEADERS} ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  COMMAND touch ${PROTO_DUMMY_FILE}
  DEPENDS ${PROTO_HEADERS}
  )

set(DELEGATE_INCLUDE_DIRS
  include
  ${OpenCV_DIR}
  ${JSONCPP_INCLUDE_DIRS}
  ${PROTOBUF_INCLUDE_DIR}
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION})


set(DELEGATE_LIBRARIES
  ${PROTOBUF_LIBRARY}
  ${OpenCV_LIBS}
  ${JSONCPP_LIBRARIES})

catkin_package(
  INCLUDE_DIRS ${DELEGATE_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME} ${DELEGATE_LIBRARIES}
  CATKIN_DEPENDS eigen
  )

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${DELEGATE_INCLUDE_DIRS})

set(SOURCES_DIRECTORIES
  src/hl_communication
  )

#Include Sources sub sources
foreach (DIRECTORY ${SOURCES_DIRECTORIES})
    include (${DIRECTORY}/Sources.cmake)
    set (PREFIXED_SOURCES)
    foreach (SOURCE ${SOURCES})
        set (PREFIXED_SOURCES ${PREFIXED_SOURCES} ${DIRECTORY}/${SOURCE})
    endforeach (SOURCE)
    set (ALL_SOURCES ${ALL_SOURCES} ${PREFIXED_SOURCES})
endforeach (DIRECTORY)

add_library (${PROJECT_NAME} SHARED ${PROTO_SOURCES} ${ALL_SOURCES} ${PROTO_DUMMY_FILE})
target_link_libraries(${PROJECT_NAME} ${PROTOBUF_LIBRARIES})

option(BUILD_HL_COMMUNICATION_EXAMPLES "Building hl_communication examples" OFF)

if (BUILD_HL_COMMUNICATION_EXAMPLES)
  add_executable(server_example examples/server_example.cpp)
  target_link_libraries(server_example ${PROJECT_NAME} ${PROTOBUF_LIBRARIES})

  add_executable(client_example examples/client_example.cpp)
  target_link_libraries(client_example ${PROJECT_NAME} ${PROTOBUF_LIBRARIES})
endif()
