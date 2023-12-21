cmake_minimum_required(VERSION 3.10)##所需的Cmake版本
project(hero_chassis_controller)##包名称

## Use C++14
set(CMAKE_CXX_STANDARD 14)##可以使用的c++版本
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)##启动的自行检查报错功能

## Find catkin macros and libraries
##查找构建所需的其他Cmake/Catkin包
find_package(catkin REQUIRED
    COMPONENTS
        COMPONENTS
        roscpp
        roslint
        controller_interface
        hardware_interface
        forward_command_controller
        control_toolbox
        geometry_msgs
        control_msgs
        realtime_tools
        tf
        nav_msgs
        pluginlib
    )

## Find system libraries
find_package(Eigen3 REQUIRED)##用于查找并配置使用线性代数运算的C++模板库
find_package(Boost REQUIRED)##查找并使用一个广泛使用的C++扩展库

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
##指定包构建信息导出
##参数：导出的包的包含路径
catkin_package(
    INCLUDE_DIRS
    include
    ## This is only necessary because Eigen3 sets a non-standard EIGEN3_INCLUDE_DIR variable
    ${EIGEN3_INCLUDE_DIR}
    LIBRARIES##参数：从项目中导出的库
    ${PROJECT_NAME}_core
    CATKIN_DEPENDS##参数：该项目依赖的其他catkn项目
        roscpp
        roslint
        controller_interface
        hardware_interface
        forward_command_controller
        control_toolbox
        geometry_msgs
        control_msgs
        realtime_tools
        tf
        nav_msgs
        pluginlib
        DEPENDS##参数：该项目依赖的非catkin Cmake项目
    ## find_package(Eigen3) provides a non standard EIGEN3_INCLUDE_DIR instead of Eigen3_INCLUDE_DIRS.
    ## Therefore, the DEPEND does not work as expected and we need to add the directory to the INCLUDE_DIRS
    # Eigen3

    ## Boost is not part of the DEPENDS since it is only used in source files,
    ## Dependees do not depend on Boost when they depend on this package.
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    # Set manually because Eigen sets a non standard INCLUDE DIR
    ${EIGEN3_INCLUDE_DIR}
    # Set because Boost is an internal dependency, not transitive.
    ${Boost_INCLUDE_DIRS}
        ${hardware_interface_INCLUDE_DIRS}

)

## Declare a cpp library
add_library(${PROJECT_NAME}_core##要构建的库/可执行文件
        src/hero_chassis_controller.cpp
    )
target_compile_features(${PROJECT_NAME}_core INTERFACE cxx_std_11)

## Declare cpp executables
add_executable(${PROJECT_NAME}##要构建的库/可执行文件
#    src/${PROJECT_NAME}_node.cpp
    src/hero_chassis_controller.cpp
    )
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_11)

## Add dependencies to exported targets, like ROS msgs or srvs
add_dependencies(${PROJECT_NAME}_core
    ${catkin_EXPORTED_TARGETS}
    )

add_dependencies(${PROJECT_NAME}
    ${catkin_EXPORTED_TARGETS}
    )

## Specify libraries to link executable targets against
target_link_libraries(${PROJECT_NAME}_core
    ${catkin_LIBRARIES}
    )

target_link_libraries(${PROJECT_NAME}
    ${PROJECT_NAME}_core
    ${catkin_LIBRARIES}
    )

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
    TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_core
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.hpp"
)

# Mark other files for installation
install(
    DIRECTORY doc
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

if (${CATKIN_ENABLE_TESTING})
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
  ## Add gtest based cpp test target and link libraries
  catkin_add_gtest(${PROJECT_NAME}-test
      test/test_ros_package_template.cpp
      test/AlgorithmTest.cpp)
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME}_core)
endif ()

##########################
## Static code analysis ##
##########################

roslint_cpp()
