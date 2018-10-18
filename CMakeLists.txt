cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR}/msgs)
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

# add_subdirectory(msgs)
# Add protobuf and message files directly instead of adding another CMake file in a submodule
# to avoid having to create two separate shared
# libraries, as there is some issue with indirectly loading .so objects in Gazebo.
find_package(Protobuf REQUIRED)

set(PROTOBUF_IMPORT_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
    if(ITR MATCHES ".*gazebo-[0-9.]+$")
        set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
    endif()
endforeach()

set (msgs
        msgs/custom_messages.proto
        ${PROTOBUF_IMPORT_DIRS}/vector2d.proto
        ${PROTOBUF_IMPORT_DIRS}/header.proto
        ${PROTOBUF_IMPORT_DIRS}/time.proto
        )
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${msgs})

# Build the stand-alone test program
add_executable(client_controller client.cpp ${PROTO_SRCS})

target_link_libraries(client_controller ${GAZEBO_LIBRARIES} ${PROTOBUF_LIBRARY})
