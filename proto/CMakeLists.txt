include("~/Downloads/vcpkg/scripts/buildsystems/vcpkg.cmake")
find_package(Protobuf REQUIRED)

file(GLOB PROTOFILES
  "*.proto"
)

protobuf_generate(OUT_VAR ${PROJECT_NAME}_PROTO_CPP_SOURCES LANGUAGE cpp PROTOS ${PROTOFILES})

add_library(${PROJECT_NAME}_proto ${${PROJECT_NAME}_PROTO_CPP_SOURCES})
target_include_directories(${PROJECT_NAME}_proto
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/proto>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_proto PUBLIC protobuf::libprotobuf)

include_directories(
  ${Protobuf_INCLUDE_DIRS}
)

install(TARGETS ${PROJECT_NAME}_proto
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
install(FILES ${PROTOFILES} DESTINATION share/${PROJECT_NAME}/proto)

set(pb_headers ${${PROJECT_NAME}_PROTO_CPP_SOURCES})
list(FILTER pb_headers INCLUDE REGEX "[.]*.pb.h")
list(TRANSFORM pb_headers REPLACE "//" "/")
message(STATUS "Installing protobuf headers: ${pb_headers}")
install(FILES ${pb_headers} DESTINATION include/${PROJECT_NAME}/proto)
unset(install_targets)
