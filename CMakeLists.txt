cmake_minimum_required(VERSION 3.5.0)
project(franka-teleop-cpp)

# Set default build type if not specified
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Use modern C++
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(CapnProto REQUIRED)
find_package(Franka REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# # generate capnp files
# capnp_generate_cpp(CAPNP_SRCS CAPNP_HDRS messages/robot-state.capnp)
# # Add the directory containing .capnp.h to the include path
# include_directories(${CMAKE_CURRENT_BINARY_DIR})

# copy config file to build
configure_file(
    ${CMAKE_SOURCE_DIR}/teleop_config.yml
    ${CMAKE_CURRENT_BINARY_DIR}/teleop_config.yml
    COPYONLY
)


add_executable(follower
            follower.cpp
            ${CAPNP_SRCS}
            ${CAPNP_HDRS})

# Link libraries
target_link_libraries(follower
    Franka::Franka
    CapnProto::capnp
    yaml-cpp
)


add_executable(leader 
            leader.cpp
            ${CAPNP_SRCS}
            ${CAPNP_HDRS})

# Link libraries
target_link_libraries(leader
    Franka::Franka
    CapnProto::capnp
    yaml-cpp
)