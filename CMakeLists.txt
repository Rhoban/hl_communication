cmake_minimum_required(VERSION 2.8)
project(hl_communication)

include(FindProtobuf)
find_package(Protobuf REQUIRED)

find_package(catkin REQUIRED)

# Handling generated headers is based on Dirk Thomas answer on topic:
# - https://answers.ros.org/answers/123416/revisions/
catkin_destinations()
file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION})

set (PROTOBUF_MESSAGES
  proto/capabilities.proto
  proto/intention.proto
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

catkin_package(
  INCLUDE_DIRS ${PROTOBUF_INCLUDE_DIR} ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  LIBRARIES ${PROJECT_NAME} ${PROTOBUF_LIBRARY}
)

#Build the kid_size main library
add_library (${PROJECT_NAME} SHARED ${PROTO_SOURCES})
target_link_libraries(${PROJECT_NAME} ${PROTOBUF_LIBRARIES})
