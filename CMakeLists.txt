cmake_minimum_required(VERSION 2.8)
project(hl_communication)

include(FindProtobuf)
find_package(Protobuf REQUIRED)

find_package(catkin REQUIRED)

#Enable C++11
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -std=c++11")

# Protobuf generate files with unused parameters
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-parameter")

# Handling generated headers is based on Dirk Thomas answer on topic:
# - https://answers.ros.org/answers/123416/revisions/
catkin_destinations()
file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION})

set (PROTOBUF_MESSAGES
  proto/capabilities.proto
  proto/game_controller.proto
  proto/intention.proto
  proto/labelling.proto
  proto/perception.proto
  proto/position.proto
  proto/robot_estimation.proto
  proto/team_play.proto
  proto/wrapper.proto
  )

protobuf_generate_cpp(PROTO_SOURCES PROTO_HEADERS ${PROTOBUF_MESSAGES})

set_source_files_properties(${PROTO_SOURCES} ${PROTO_HEADERS} PROPERTIES GENERATED TRUE)

add_custom_target(${PROJECT_NAME}_generate_headers ALL
  COMMAND cp ${PROTO_HEADERS} ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  DEPENDS ${PROTO_HEADERS}
  )

set(${PROJECT_NAME}_EXPORTED_TARGETS ${PROJECT_NAME}_generate_headers)

set(DELEGATE_INCLUDE_DIRS
  include
  ${PROTOBUF_INCLUDE_DIR}
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION})

catkin_package(
  INCLUDE_DIRS ${DELEGATE_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME} ${PROTOBUF_LIBRARY}
  )

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION})

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

#Build the kid_size main library
add_library (${PROJECT_NAME} SHARED ${PROTO_SOURCES} ${ALL_SOURCES})
target_link_libraries(${PROJECT_NAME} ${PROTOBUF_LIBRARIES})

option(BUILD_HL_COMMUNICATION_EXAMPLES "Building hl_communication examples" OFF)

if (BUILD_HL_COMMUNICATION_EXAMPLES)
  add_executable(server_example examples/server_example.cpp)
  target_link_libraries(server_example ${PROJECT_NAME} ${PROTOBUF_LIBRARIES})

  add_executable(client_example examples/client_example.cpp)
  target_link_libraries(client_example ${PROJECT_NAME} ${PROTOBUF_LIBRARIES})
endif()
