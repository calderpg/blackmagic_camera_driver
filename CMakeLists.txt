cmake_minimum_required(VERSION 2.8.3)
project(blackmagic_camera_driver)

find_package(catkin REQUIRED COMPONENTS
             roscpp image_transport sensor_msgs geometry_msgs tf2_msgs)
find_package(Eigen3 REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
find_package(OpenMP)

catkin_package(CATKIN_DEPENDS
               roscpp
               image_transport
               sensor_msgs
               geometry_msgs
               tf2_msgs
               DEPENDS
               Eigen3)

add_library(decklink
            decklink_sdk/DeckLinkAPIConfiguration.h
            decklink_sdk/DeckLinkAPIConfiguration_v10_11.h
            #decklink_sdk/DeckLinkAPIConfiguration_v10_2.h
            #decklink_sdk/DeckLinkAPIConfiguration_v10_4.h
            #decklink_sdk/DeckLinkAPIConfiguration_v10_5.h
            #decklink_sdk/DeckLinkAPIConfiguration_v10_9.h
            decklink_sdk/DeckLinkAPIDeckControl.h
            decklink_sdk/DeckLinkAPIDiscovery.h
            decklink_sdk/DeckLinkAPIDispatch.cpp
            decklink_sdk/DeckLinkAPIDispatch_v10_11.cpp
            #decklink_sdk/DeckLinkAPIDispatch_v10_8.cpp
            #decklink_sdk/DeckLinkAPIDispatch_v7_6.cpp
            #decklink_sdk/DeckLinkAPIDispatch_v8_0.cpp
            decklink_sdk/DeckLinkAPI.h
            decklink_sdk/DeckLinkAPIModes.h
            decklink_sdk/DeckLinkAPITypes.h
            #decklink_sdk/DeckLinkAPI_v10_11.h
            #decklink_sdk/DeckLinkAPI_v10_2.h
            #decklink_sdk/DeckLinkAPI_v10_4.h
            #decklink_sdk/DeckLinkAPI_v10_5.h
            #decklink_sdk/DeckLinkAPI_v10_6.h
            #decklink_sdk/DeckLinkAPI_v10_9.h
            decklink_sdk/DeckLinkAPI_v11_5_1.h
            #decklink_sdk/DeckLinkAPI_v11_5.h
            #decklink_sdk/DeckLinkAPI_v7_1.h
            #decklink_sdk/DeckLinkAPI_v7_3.h
            #decklink_sdk/DeckLinkAPI_v7_6.h
            #decklink_sdk/DeckLinkAPI_v7_9.h
            #decklink_sdk/DeckLinkAPI_v8_0.h
            #decklink_sdk/DeckLinkAPI_v8_1.h
            #decklink_sdk/DeckLinkAPI_v9_2.h
            #decklink_sdk/DeckLinkAPI_v9_9.h
            decklink_sdk/DeckLinkAPIVersion.h
            decklink_sdk/DeckLinkAPIVideoEncoderInput_v10_11.h
            #decklink_sdk/DeckLinkAPIVideoInput_v10_11.h
            #decklink_sdk/DeckLinkAPIVideoInput_v11_4.h
            decklink_sdk/DeckLinkAPIVideoInput_v11_5_1.h
            #decklink_sdk/DeckLinkAPIVideoOutput_v10_11.h
            decklink_sdk/DeckLinkAPIVideoOutput_v11_4.h
            decklink_sdk/LinuxCOM.h)
set_target_properties(decklink PROPERTIES COMPILE_FLAGS "-Wno-multichar")

include_directories(include SYSTEM decklink_sdk
                            ${catkin_INCLUDE_DIRS}
                            ${Eigen3_INCLUDE_DIRS})

## Build options
add_compile_options(-std=c++11)
add_compile_options(-Wall)
add_compile_options(-Wextra)
add_compile_options(-Werror)
add_compile_options(-Wconversion)
add_compile_options(-Wshadow)
add_compile_options(-O3)
add_compile_options(-g)
add_compile_options(-flto)
add_compile_options(-Werror=non-virtual-dtor)

## It's not clear if add_compile_options does the right things for flags that
## may differ between languages and target type.
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS
    "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
set(CMAKE_SHARED_LINKER_FLAGS
    "${CMAKE_SHARED_LINKER_FLAGS} ${OpenMP_SHARED_LINKER_FLAGS}")

add_library(${PROJECT_NAME}
            include/${PROJECT_NAME}/bmd_handle.hpp
            include/${PROJECT_NAME}/decklink_interface.hpp
            src/${PROJECT_NAME}/decklink_interface.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} decklink)

add_executable(blackmagic_camera_driver_node
               src/blackmagic_camera_driver_node.cpp)
add_dependencies(blackmagic_camera_driver_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(blackmagic_camera_driver_node
                      ${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS decklink ${PROJECT_NAME} blackmagic_camera_driver_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.hpp"
    PATTERN ".svn" EXCLUDE
)
