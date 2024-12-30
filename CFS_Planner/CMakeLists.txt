cmake_minimum_required(VERSION 3.12.0)
project(cfslib VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# /* -------------------------------------------------------------------------- */
# /*                              Build Properties                              */
# /* -------------------------------------------------------------------------- */

# set(DEFAULT_BUILD_TYPE "debug")

# if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
#     message(STATUS "Setting build type to '${DEFAULT_BUILD_TYPE}' as none was specified.")
#     set(CMAKE_BUILD_TYPE "${DEFAULT_BUILD_TYPE}" CACHE STRING "Choose the type of build." FORCE)
#     # Set the possible values of build type for cmake-gui
#     set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "debug" "release")
# endif()

# add_compile_options(-std=c++17)
# add_definitions(-D_OS_UNIX)

add_compile_definitions(BUILD_TYPE_DEBUG=0)
add_compile_definitions(BUILD_TYPE_RELEASE=1)
if (CMAKE_BUILD_TYPE MATCHES "debug")
    set(BUILD_TYPE_VAL BUILD_TYPE_DEBUG)
else()
    set(BUILD_TYPE_VAL BUILD_TYPE_RELEASE)
endif()
add_compile_definitions(BUILD_TYPE=${BUILD_TYPE_VAL})
add_compile_definitions("SOURCE_ROOT=${CMAKE_SOURCE_DIR}")

# /* -------------------------------------------------------------------------- */
# /*                                  Vrep API                                  */
# /* -------------------------------------------------------------------------- */
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/3rd/include/
    ${CMAKE_CURRENT_SOURCE_DIR}/3rd/remoteApi/
)

add_definitions(-DNON_MATLAB_PARSING)
add_definitions(-DDO_NOT_USE_SHARED_MEMORY)
add_definitions(-DMAX_EXT_API_CONNECTIONS=255)

set(EXTERN_API 
    ${CMAKE_CURRENT_SOURCE_DIR}/3rd/remoteApi/extApi.c 
    ${CMAKE_CURRENT_SOURCE_DIR}/3rd/remoteApi/extApiPlatform.c
)

# /* -------------------------------------------------------------------------- */
# /*                                Find Package                                */
# /* -------------------------------------------------------------------------- */

find_package(Eigen3 3.3.7 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(aruco REQUIRED)
# find_package(serial)
find_package(realsense2 REQUIRED)
set(realsense2_INCLUDE_DIRS /usr/local/include/librealsense2 /usr/include/librealsense2)

include_directories(include)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${realsense2_INCLUDE_DIRS})

# /* -------------------------------------------------------------------------- */
# /*                                  ikfast                                    */
# /* -------------------------------------------------------------------------- */
include_directories(ikfast/ikfast)
add_subdirectory(ikfast)
include_directories(include/robot_model)
# /* -------------------------------------------------------------------------- */
# /*                                   QP lib                                   */
# /* -------------------------------------------------------------------------- */

include_directories(lib/qplib)
add_subdirectory(lib)

# /* -------------------------------------------------------------------------- */
# /*                               Create Library                               */
# /* -------------------------------------------------------------------------- */

file(GLOB src_files src/Core/*.cpp
                    src/robot_model/*.cpp
                    src/Query/*.cpp
                    src/Robot/*.cpp
                    src/Trajectory/*.cpp
                    src/Highlevel/*.cpp
                    src/cfslib.cpp
                    src/Utils/*.cpp
                    src/Perception/detector/*.cpp
                    src/Perception/realsense/*.cpp
                    src/*.cpp
                    )

add_library(${PROJECT_NAME} ${src_files})

target_link_libraries(${PROJECT_NAME}
    realsense2
    qplib
    Eigen3::Eigen
)

# IF (Serial_FOUND)
#     file(GLOB src_files_gripper src/Gripper/*.cpp)
#     add_library(${PROJECT_NAME}_gripper ${src_files_gripper})
#     target_link_libraries(${PROJECT_NAME}_gripper
#         serial
#     )
# ENDIF()

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

set_target_properties(${PROJECT_NAME} PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION 1)

# /* -------------------------------------------------------------------------- */
# /*                                Installation                                */
# /* -------------------------------------------------------------------------- */

# install(TARGETS ${PROJECT_NAME} EXPORT cfslibConfig
#     ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
#     LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
#     RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR})

# install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/cfslib DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
# message("CMAKE_INSTALL_INCLUDEDIR " ${CMAKE_INSTALL_INCLUDEDIR})

# install(EXPORT cfslibConfig DESTINATION share/cfslib/cmake)

# export(TARGETS ${PROJECT_NAME} FILE cfslibConfig.cmake)

# /* -------------------------------------------------------------------------- */
# /*                                  pybind11                                  */
# /* -------------------------------------------------------------------------- */
# add_subdirectory(extern/pybind11)
# include_directories(extern/pybind11/include)
# pybind11_add_module(cfspy src/Highlevel/CFSPickPlace.cpp)
# install(TARGETS cfspy LIBRARY DESTINATION .)
find_package(Python 3.8 COMPONENTS Interpreter Development REQUIRED)
find_package(pybind11 CONFIG REQUIRED)
pybind11_add_module(cfspy src/Highlevel/CFSPickPlace.cpp)
target_link_libraries(cfspy PRIVATE
    ikfast_gp7
    ikfast_gp12
    aruco
    ${OpenCV_LIBS}
    ${PROJECT_NAME}
    pthread
    rt
    m
)
# install(TARGETS cfspy DESTINATION .)

include_directories(${PYTHON_INCLUDE_DIRS})
include_directories(/usr/include/python3.10/)
include_directories(/usr/include/python3.8/)

# /* -------------------------------------------------------------------------- */
# /*                             Testing Executables                            */
# /* -------------------------------------------------------------------------- */

add_executable(${PROJECT_NAME}_hand_eye_calibration
  src/Test/1_hand_eye_calibration.cpp
  ${EXTERN_API}
)
target_link_libraries(${PROJECT_NAME}_hand_eye_calibration
    aruco
    realsense2
    ${OpenCV_LIBS}
    ${PROJECT_NAME}
    pthread
    rt
    m
)

add_executable(${PROJECT_NAME}_hand_eye_calibration_gp7
  src/Test/1_hand_eye_calibration.cpp
  ${EXTERN_API}
)
target_link_libraries(${PROJECT_NAME}_hand_eye_calibration_gp7
    aruco
    realsense2
    ${OpenCV_LIBS}
    ${PROJECT_NAME}
    pthread
    rt
    m
)

add_executable(${PROJECT_NAME}_hand_eye_calibration_record
  src/Test/1_hand_eye_calibration_record.cpp
  ${EXTERN_API}
)
target_link_libraries(${PROJECT_NAME}_hand_eye_calibration_record
    aruco
    realsense2
    ${OpenCV_LIBS}
    ${PROJECT_NAME}
    pthread
    rt
    m
)

add_executable(${PROJECT_NAME}_hand_eye_calibration_evaluation
  src/Test/1_hand_eye_calibration_evaluation.cpp
  ${EXTERN_API}
)
target_link_libraries(${PROJECT_NAME}_hand_eye_calibration_evaluation
    aruco
    realsense2
    ${OpenCV_LIBS}
    ${PROJECT_NAME}
    pthread
    rt
    m
)

add_executable(${PROJECT_NAME}_hand_eye_calibration_pnpUT
  src/Test/1_hand_eye_calibration_pnpUT.cpp
  ${EXTERN_API}
)
target_link_libraries(${PROJECT_NAME}_hand_eye_calibration_pnpUT
    aruco
    realsense2
    ${OpenCV_LIBS}
    ${PROJECT_NAME}
    pthread
    rt
    m
)

add_executable(${PROJECT_NAME}_hand_eye_calibration_UT
  src/Test/1_hand_eye_calibration_UT.cpp
  ${EXTERN_API}
)
target_link_libraries(${PROJECT_NAME}_hand_eye_calibration_UT
    aruco
    realsense2
    ${OpenCV_LIBS}
    ${PROJECT_NAME}
    pthread
    rt
    m
)

add_executable(${PROJECT_NAME}_hand_eye_calibration_touch_test
  src/Test/1_hand_eye_calibration_touch_test.cpp
  ${EXTERN_API}
)
target_link_libraries(${PROJECT_NAME}_hand_eye_calibration_touch_test
    aruco
    realsense2
    ${OpenCV_LIBS}
    ${PROJECT_NAME}
    pthread
    rt
    m
)

add_executable(${PROJECT_NAME}_tool_calibration
  src/Test/2_tool_calibration.cpp
  ${EXTERN_API}
)
target_link_libraries(${PROJECT_NAME}_tool_calibration
    aruco
    ${OpenCV_LIBS}
    ${PROJECT_NAME}
    pthread
    rt
    m
)

add_executable(${PROJECT_NAME}_tool_calibration_UT
  src/Test/2_tool_calibration_UT.cpp
  ${EXTERN_API}
)
target_link_libraries(${PROJECT_NAME}_tool_calibration_UT
    aruco
    ${OpenCV_LIBS}
    ${PROJECT_NAME}
    pthread
    rt
    m
)

add_executable(${PROJECT_NAME}_optimized_pick_place
  src/Test/5_optimized_pick_place.cpp
  ${EXTERN_API}
)
target_link_libraries(${PROJECT_NAME}_optimized_pick_place
    aruco
    ${OpenCV_LIBS}
    ${PROJECT_NAME}
    pthread
    rt
    m
    pybind11::embed
)

add_executable(${PROJECT_NAME}_test_CFS_subroutine
  src/Test/test_CFS_subroutine.cpp
  ${EXTERN_API}
)
target_link_libraries(${PROJECT_NAME}_test_CFS_subroutine
    aruco
    ${OpenCV_LIBS}
    ${PROJECT_NAME}
    pthread
    rt
    m
    pybind11::embed
)

# IF (Serial_FOUND)

#     add_executable(${PROJECT_NAME}_pick_and_place
#     src/Test/4_pickplace.cpp
#     ${EXTERN_API}
#     )
#     target_link_libraries(${PROJECT_NAME}_pick_and_place
#         aruco
#         ${OpenCV_LIBS}
#         ${PROJECT_NAME}
#         ${PROJECT_NAME}_grippermat_base2cam
#         pthread
#         rt
#         m
#     )

#     add_executable(${PROJECT_NAME}_gripper_activate
#     src/Test/gripper_activate.cpp
#     ${EXTERN_API}
#     )
#     target_link_libraries(${PROJECT_NAME}_gripper_activate
#         aruco
#         ${OpenCV_LIBS}
#         ${PROJECT_NAME}
#         pthread
#         rt
#         m
#     )

#     add_executable(${PROJECT_NAME}_gripper_grip
#     src/Test/gripper_grip.cpp
#     ${EXTERN_API}
#     )
#     target_link_libraries(${PROJECT_NAME}_gripper_grip
#         aruco
#         ${OpenCV_LIBS}
#         ${PROJECT_NAME}
#         pthread
#         rt
#         m
#     )

#     add_executable(${PROJECT_NAME}_gripper_release
#     src/Test/gripper_release.cpp
#     ${EXTERN_API}
#     )
#     target_link_libraries(${PROJECT_NAME}_gripper_release
#         aruco
#         ${OpenCV_LIBS}
#         ${PROJECT_NAME}
#         pthread
#         rt
#         m
#     )
# ENDIF()
