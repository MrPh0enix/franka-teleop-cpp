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
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)

# # for libfranka auto
# find_package(Franka REQUIRED)

# use libfarnka 0.9.2
# set(Franka_DIR "/home/johnkiran/libfranka0.9.2/libfranka/build")
# find_package(Franka REQUIRED)

# use libfarnka 0.18.0
set(FRANKA_ROOT "/home/johnkiran/libfranka/build")
find_package(Franka REQUIRED PATHS ${FRANKA_ROOT} NO_DEFAULT_PATH)


# generate capnp files
capnp_generate_cpp(CAPNP_SRCS CAPNP_HDRS messages/robot-state.capnp)
# Add the directory containing .capnp.h to the include path
include_directories(${CMAKE_CURRENT_BINARY_DIR})
# include franka examples directory
include_directories(${CMAKE_SOURCE_DIR}/utils)
# include Eigen3
include_directories(${EIGEN3_INCLUDE_DIR})

# copy config file to build
configure_file(
    ${CMAKE_SOURCE_DIR}/teleop_config.yml
    ${CMAKE_CURRENT_BINARY_DIR}/teleop_config.yml
    COPYONLY
)


# compile the utils library
add_library(utils
    utils/examples_common.cpp
    utils/VelocityObserver.cpp
)
target_include_directories(utils PUBLIC
    ${CMAKE_SOURCE_DIR}/utils
)
target_link_libraries(utils
    Franka::Franka
)


set(LEADER_SOURCE leader_optimized_dob.cpp)
set(FOLLOWER_SOURCE follower_optimized_dob.cpp)


#unilateral executables
add_executable(leader_unilateral
            ${LEADER_SOURCE}
            ${CAPNP_SRCS}
            ${CAPNP_HDRS})
target_compile_definitions(
    leader_unilateral
    PRIVATE TELEOP_UNILATERAL
)
target_link_libraries(leader_unilateral
    utils
    Franka::Franka
    CapnProto::capnp
    CapnProto::kj
    yaml-cpp
)

add_executable(follower_unilateral
            ${FOLLOWER_SOURCE}
            ${CAPNP_SRCS}
            ${CAPNP_HDRS})
target_compile_definitions(
    follower_unilateral
    PRIVATE TELEOP_UNILATERAL
)
target_link_libraries(follower_unilateral
    utils
    Franka::Franka
    CapnProto::capnp
    CapnProto::kj
    yaml-cpp
)


#bilateral executables
add_executable(leader_bilateral
            ${LEADER_SOURCE}
            ${CAPNP_SRCS}
            ${CAPNP_HDRS})
target_compile_definitions(
    leader_bilateral
    PRIVATE TELEOP_BILATERAL
)
target_link_libraries(leader_bilateral
    utils
    Franka::Franka
    CapnProto::capnp
    CapnProto::kj
    yaml-cpp
)

add_executable(follower_bilateral
            ${FOLLOWER_SOURCE}
            ${CAPNP_SRCS}
            ${CAPNP_HDRS})
target_compile_definitions(
    follower_bilateral
    PRIVATE TELEOP_BILATERAL
)
target_link_libraries(follower_bilateral
    utils
    Franka::Franka
    CapnProto::capnp
    CapnProto::kj
    yaml-cpp
)
