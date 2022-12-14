cmake_minimum_required(VERSION 2.8.3)
project(asr_ftc_local_planner)

find_package(catkin REQUIRED
        COMPONENTS
            cmake_modules
            costmap_2d
            dynamic_reconfigure
            nav_core
            nav_msgs
            pluginlib
            pcl_conversions
            roscpp
            tf
        )

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
set (asr_INCLUDE_DIRS "/home/cczhengy/catkin_ws/src")
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    ${asr_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    )
add_definitions(${Eigen3_DEFINITIONS})

link_directories(${catkin_LIBRARY_DIRS})

# dynamic reconfigure
generate_dynamic_reconfigure_options(
    cfg/FTCPlanner.cfg
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES asr_ftc_local_planner
    CATKIN_DEPENDS
        dynamic_reconfigure
        pluginlib
        roscpp
)

add_library(asr_ftc_local_planner 
	    src/ftc_planner.cpp
	    src/transform_global_plan.cpp
	    src/join_costmap.cpp)
target_link_libraries(asr_ftc_local_planner ${catkin_LIBRARIES})
add_dependencies(asr_ftc_local_planner asr_ftc_local_planner_gencfg)
add_dependencies(asr_ftc_local_planner nav_msgs_gencpp)

install(TARGETS asr_ftc_local_planner
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       )
       
install(FILES blp_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
 
