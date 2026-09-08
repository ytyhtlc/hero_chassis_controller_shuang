cmake_minimum_required(VERSION 3.10)
project(hero_chassis_controller)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        roslint
        controller_interface
        hardware_interface
        geometry_msgs
        pluginlib

)

##hand make code shuang!

###################################
## catkin specific configuration ##
###################################
catkin_package(
        INCLUDE_DIRS include

        LIBRARIES

        CATKIN_DEPENDS
        roscpp
        roslint
        controller_interface
        hardware_interface
        geometry_msgs
        pluginlib

        DEPENDS
)

###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS}

)

add_library(
        ${PROJECT_NAME}
        src/${PROJECT_NAME}.cpp
)

add_dependencies(${PROJECT_NAME}
        ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
)

roslint_cpp()

##shuang!