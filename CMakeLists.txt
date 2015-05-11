cmake_minimum_required(VERSION 2.8.3)
project(blackboard)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    roscpp
)

# find_package(Boost REQUIRED COMPONENTS system program_options)
# find_package(PCL REQUIRED)
# find_package(OpenCV REQUIRED)

# ------------------------------------------------------------------------------------------------
#                                     ROS MESSAGES AND SERVICES
# ------------------------------------------------------------------------------------------------

# Generate messages
add_message_files(
   FILES
   ValueUpdate.msg
   ValueUpdates.msg
   KeyUpdate.msg
)

# Generate services
# add_service_files(
#    FILES
#    service1.srv
#    ...
# )

# Generate added messages and services with any dependencies listed here
generate_messages(
#    DEPENDENCIES
#    geometry_msgs
#    ...
)

# ------------------------------------------------------------------------------------------------
#                                          CATKIN EXPORT
# ------------------------------------------------------------------------------------------------

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES blackboard
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

# ------------------------------------------------------------------------------------------------
#                                              BUILD
# ------------------------------------------------------------------------------------------------

include_directories(
    include
#    ${catkin_INCLUDE_DIRS}
)

add_library(blackboard
    include/blackboard/blackboard.h
    include/blackboard/key.h
    include/blackboard/variant.h
    include/blackboard/serializer.h
    include/blackboard/buffer.h
    src/blackboard.cpp
)
target_link_libraries(blackboard ${catkin_LIBRARIES})

# ------------------------------------------------------------------------------------------------
#                                              TEST
# ------------------------------------------------------------------------------------------------

add_executable(test_blackboard test/test_blackboard.cpp)
target_link_libraries(test_blackboard blackboard)


