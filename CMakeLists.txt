cmake_minimum_required(VERSION 4.0)

string(REGEX REPLACE "(.*)/" "" THIS_FOLDER_NAME "${CMAKE_CURRENT_SOURCE_DIR}")
project(${THIS_FOLDER_NAME})

# Add external
set(C74_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/externals")
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/source/ahrs)

# Add Fusion
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/source/Fusion/Fusion)

target_link_libraries(ahrs PRIVATE Fusion)