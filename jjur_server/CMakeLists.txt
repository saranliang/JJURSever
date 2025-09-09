cmake_minimum_required(VERSION 3.5.0)

project(JJURServer)

set(QT_USE_QTNETWORK TRUE)
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(Qt5Gui REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Network REQUIRED)

set(QT_USE_QTMAIN TRUE) #使用qtmain库，去除黑窗口

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/third/
    ${QT_INCLUDES}
    ${PROJECT_BINARY_DIR}
)

link_directories(
    ${QT_LIBRARY_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/lib
)

set(JJURServer_SRCS
    URStruct.h
    URTcp.cpp
    URTcp.h
    #URStatus.cpp
    #URStatus.h
    URServer.cpp
    URServer.h
    URBasicMotion.cpp
    URBasicMotion.h
    #UR3DMouse.cpp
    #UR3DMouse.h
    URToolSetting.cpp
    URToolSetting.h
    URForceMode.cpp
    URForceMode.h
    CommandStruct.h
    CommandStruct.cpp
    ConnectionObserver.cpp
    ConnectionObserver.h
    PID.h
)

set(JJURServer_MOC_SRCS
    URStruct.h
    #URStatus.h
    URTcp.h
    URServer.h
    URBasicMotion.h
    #UR3DMouse.h
    URToolSetting.h
    URForceMode.h
    CommandStruct.h
    ConnectionObserver.h
    PID.h
)

qt5_wrap_cpp(JJURServer_MOC_CXX ${JJURServer_MOC_SRCS})
# qt5_wrap_ui(JJURServer_UI_CXX ${JJURServer_UI_SRCS})

source_group("Resources" FILES
    ${JJURServer_UI_SRCS}
)

source_group("Generated" FILES
    ${JJURServer_MOC_CXX}
    ${JJURServer_UI_CXX}
)

add_library(${PROJECT_NAME}
    ${JJURServer_SRCS}
    ${JJURServer_MOC_CXX}
)

target_link_libraries(${PROJECT_NAME}
    Qt5::Core
    Qt5::Widgets
    Qt5::Network
    #spnav
)
add_custom_command(
    OUTPUT
       ${CMAKE_CURRENT_BINARY_DIR}/generated/version.hpp
    ALL
    COMMAND
       ${CMAKE_COMMAND} -D TARGET_DIR=${CMAKE_CURRENT_BINARY_DIR} -P ${CMAKE_CURRENT_SOURCE_DIR}/git-info.cmake
    WORKING_DIRECTORY
       ${CMAKE_CURRENT_SOURCE_DIR}
)
add_custom_target(
  get_git_info
  ALL
  DEPENDS
    ${CMAKE_CURRENT_BINARY_DIR}/generated/version.hpp
  )
add_dependencies(${PROJECT_NAME} get_git_info)