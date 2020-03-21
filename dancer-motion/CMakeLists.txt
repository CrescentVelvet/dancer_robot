cmake_minimum_required(VERSION 2.8.3)
project(dmotion)

add_definitions(-std=c++11 -Wall -O2)
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        # dmsgs
        # dcommon
        # dtransmit
#       dconfig
        # dprocess
        )
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

catkin_package(
        INCLUDE_DIRS
        include
        #  LIBRARIES dancer_motion
        #  CATKIN_DEPENDS roscpp std_msgs
        #  DEPENDS system_lib
)

include_directories(include
        ${EIGEN3_INCLUDE_DIR}
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        )


add_library( Parameters         ./src/Parameters.cpp)   

add_library( InverseKinematics  ./src/InverseKinematics.cpp)
target_link_libraries(InverseKinematics Parameters)

add_library( ForwardKinematics  ./src/ForwardKinematics.cpp)

add_library( ThreeInterpolation ./src/ThreeInterpolation.cpp)

add_library( OneFootLanding     ./src/OneFootLanding.cpp)
target_link_libraries(OneFootLanding InverseKinematics Parameters)

# add_library( Kick     ./src/Kick.cpp)
# target_link_libraries(Kick OneFootLanding ThreeInterpolation ForwardKinematics)

add_library( PendulumWalk       ./src/PendulumWalk.cpp)
target_link_libraries(PendulumWalk OneFootLanding ThreeInterpolation Parameters)

add_library( Climb              ./src/climb.cpp)
target_link_libraries(Climb OneFootLanding ThreeInterpolation Parameters ForwardKinematics)

add_executable(main node/main.cpp)
target_link_libraries(main PendulumWalk ForwardKinematics Climb Parameters ${catkin_LIBRARIES} )

# add_executable(kick_test example/kick_test.cpp)
# target_link_libraries(kick_test PendulumWalk ForwardKinematics Kick Climb Parameters ${catkin_LIBRARIES})

# add_executable(onefootlanding_test example/onefootlanding_test.cpp)
# target_link_libraries(onefootlanding_test PendulumWalk ForwardKinematics Kick Climb Parameters ${catkin_LIBRARIES})

# add_executable(guard_test example/guard_test.cpp)
# target_link_libraries(guard_test PendulumWalk ForwardKinematics Kick Climb Parameters ${catkin_LIBRARIES})