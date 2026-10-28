# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Humanplus Intelligent Robotics Technology Co.,Ltd. All rights reserved.

set(MOVESENSE_X95_CORE_TARGET ${PROJECT_NAME}_core)

file(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/VERSION" MOVESENSE_X95_RELEASE_VERSION LIMIT_COUNT 1)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/VERSION")
message(STATUS "MoveSense X95 ROS release: ${MOVESENSE_X95_RELEASE_VERSION}")

add_library(${MOVESENSE_X95_CORE_TARGET} STATIC
  src/calibration_data.cpp
  src/calibration_export.cpp
  src/camera_config.cpp
  src/camera_session.cpp
  src/data_receiver.cpp
  src/image_geometry.cpp
  src/seg_result.cpp
  src/session_supervisor.cpp
)

target_include_directories(${MOVESENSE_X95_CORE_TARGET} PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(${MOVESENSE_X95_CORE_TARGET} PUBLIC
  MoveSense::X95_SDK
  ${OpenCV_LIBS}
)

target_compile_definitions(${MOVESENSE_X95_CORE_TARGET} PUBLIC
  "MOVESENSE_X95_RELEASE_VERSION=\"${MOVESENSE_X95_RELEASE_VERSION}\""
)

option(MOVESENSE_VERIFY_PEER_CORE "Verify identical common files in the sibling ROS repository" ON)

if(MOVESENSE_VERIFY_PEER_CORE)
  if(PROJECT_NAME STREQUAL "movesense_x95_ros")
    set(MOVESENSE_X95_PEER_NAME movesense_x95_ros2)
  else()
    set(MOVESENSE_X95_PEER_NAME movesense_x95_ros)
  endif()

  get_filename_component(MOVESENSE_X95_PEER_ROOT
    "${CMAKE_CURRENT_SOURCE_DIR}/../${MOVESENSE_X95_PEER_NAME}" ABSOLUTE)

  if(EXISTS "${MOVESENSE_X95_PEER_ROOT}/CMakeLists.txt")
    file(GLOB MOVESENSE_X95_SHARED_HEADERS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
      "${CMAKE_CURRENT_SOURCE_DIR}/include/movesense_x95/*.h")
    file(GLOB MOVESENSE_X95_PEER_HEADERS RELATIVE "${MOVESENSE_X95_PEER_ROOT}"
      "${MOVESENSE_X95_PEER_ROOT}/include/movesense_x95/*.h")
    if(NOT "${MOVESENSE_X95_SHARED_HEADERS}" STREQUAL "${MOVESENSE_X95_PEER_HEADERS}")
      message(FATAL_ERROR "Common header lists differ between the ROS repositories")
    endif()

    set(MOVESENSE_X95_SHARED_FILES
      ${MOVESENSE_X95_SHARED_HEADERS}
      src/calibration_data.cpp
      src/calibration_export.cpp
      src/camera_config.cpp
      src/camera_session.cpp
      src/data_receiver.cpp
      src/image_geometry.cpp
      src/seg_result.cpp
      src/session_supervisor.cpp
      cmake/MoveSenseX95Core.cmake
      VERSION
    )
    foreach(MOVESENSE_X95_SHARED_FILE IN LISTS MOVESENSE_X95_SHARED_FILES)
      set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/${MOVESENSE_X95_SHARED_FILE}"
        "${MOVESENSE_X95_PEER_ROOT}/${MOVESENSE_X95_SHARED_FILE}"
      )
      if(NOT EXISTS "${MOVESENSE_X95_PEER_ROOT}/${MOVESENSE_X95_SHARED_FILE}")
        message(FATAL_ERROR "Missing common peer file: ${MOVESENSE_X95_SHARED_FILE}")
      endif()
      file(SHA256 "${CMAKE_CURRENT_SOURCE_DIR}/${MOVESENSE_X95_SHARED_FILE}" MOVESENSE_X95_LOCAL_HASH)
      file(SHA256 "${MOVESENSE_X95_PEER_ROOT}/${MOVESENSE_X95_SHARED_FILE}" MOVESENSE_X95_PEER_HASH)
      if(NOT MOVESENSE_X95_LOCAL_HASH STREQUAL MOVESENSE_X95_PEER_HASH)
        message(FATAL_ERROR "Common file differs between the ROS repositories: ${MOVESENSE_X95_SHARED_FILE}")
      endif()
    endforeach()
    message(STATUS "Common ROS1/ROS2 core files are byte-identical")
  endif()
endif()
