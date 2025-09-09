cmake_minimum_required(VERSION 3.5.0)

project(JJURServerTest)

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
    ${QT_INCLUDES}
    ${JJURServer_SOURCE_DIR}
    ${JJURServer_SOURCE_DIR}/include
    ${JJURServer_SOURCE_DIR}/third/
)

link_directories(
    ${QT_LIBRARY_DIR}
    ${JJURServer_BINARY_DIR}
    ${JJURServerTest_SOURCE_DIR}/lib
)

set(JJURServerTest_SRCS
    main.cpp
    mainwindow.cpp
    mainwindow.h
)

set(JJURServerTest_MOC_SRCS
    mainwindow.h
)

set(JJURServerTest_UI_SRCS
    mainwindow.ui
)

qt5_wrap_cpp(JJURServerTest_MOC_CXX ${JJURServerTest_MOC_SRCS})
qt5_wrap_ui(JJURServerTest_UI_CXX ${JJURServerTest_UI_SRCS})

add_executable(${PROJECT_NAME}
    ${JJURServerTest_SRCS}
    ${JJURServerTest_MOC_CXX}
    ${JJURServerTest_UI_SRCS}
)

add_dependencies(${PROJECT_NAME}
    JJURServer
)

target_link_libraries(${PROJECT_NAME}
    Qt5::Core
    Qt5::Widgets
    Qt5::Network
    JJURServer
)


##########################################################################
########################### install package ##############################
##########################################################################

#include (InstallRequiredSystemLibraries) #导入 InstallRequiredSystemLibraries 模块
#set(CPACK_SET_DESTDIR ON)  #lib will be install /usr/local if set TRUE
set(CPACK_PACKAGING_INSTALL_PREFIX "/home/jjst/")  #it's called cpack_install_prefix in older version
set(CPACK_INSTALL_PREFIX "/home/jjst/")
set(CPACK_PACKAGE_INSTALL_DIRECTORY "/home/jjst/")

# exe and libraries files package configuration
set(CPACK_PACKAGE_NAME "JJURServer")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "This package provides the JJURServer.")
set(CPACK_PACKAGE_VENDOR "Jointech LLC.")
set(CPACK_PACKAGE_CONTACT "qiaotian@outlook.com")
if (WIN32)
    message(STATUS "Now is windows")
elseif (APPLE)
    message(STATUS "Now is Apple systens.")
    set(CPACK_GENERATOR "DragNDrop")
    set(CPACK_SYSTEM_NAME "macosx")
elseif (UNIX)
  message(STATUS "Now is UNIX-like OS's.")
  set(CPACK_GENERATOR "DEB")
  set(CPACK_SYSTEM_NAME "ubuntu")
endif ()

set(CPACK_PACKAGE_VERSION_MAJOR "1")
set(CPACK_PACKAGE_VERSION_MINOR "0")
set(CPACK_PACKAGE_VERSION_PATCH "0")
set(CPACK_PACKAGE_VERSION_DATE "20221230")
set(CPACK_PACKAGE_VERSION ${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH}.${CPACK_PACKAGE_VERSION_DATE})
set(CPACK_SOURCE_PACKAGE_FILE_NAME ${PROJECT_NAME}-${CPACK_PACKAGE_VERSION})
#SET(CPACK_PACKAGE_DEFAULT_LOCATION "/opt/${CPACK_PACKAGE_NAME}")

# sources files package configuration
set(CPACK_SOURCE_IGNORE_FILES
        ${JJURServerLib_SOURCE_DIR}/build
        ${JJURServerLib_SOURCE_DIR}/cmake-build-debug
        ${JJURServerLib_SOURCE_DIR}/pack
        ${JJURServerLib_SOURCE_DIR}/.idea
        ${JJURServerLib_SOURCE_DIR}/.DS_Store
        ${JJURServerLib_SOURCE_DIR}/.git
        ${JJURServerLib_SOURCE_DIR}/.gitignore
        ${JJURServerLib_SOURCE_DIR}/.vscode
        ${JJURServerLib_SOURCE_DIR}/.PIC)


set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA ${JJURServerLib_SOURCE_DIR}/Scripts/postinst)

file(GLOB Library_Config LIST_DIRECTORIES false
    ${JJURServerLib_SOURCE_DIR}/Scripts/JJURServer.conf
)
install(FILES ${Library_Config} DESTINATION /etc/ld.so.conf.d/)


set(QT_PLUGIN
    ${Qt5Gui_DIR}/../../../plugins/platforms
    ${Qt5Gui_DIR}/../../../plugins/platformthemes
    ${Qt5Gui_DIR}/../../../plugins/platforminputcontexts
    ${Qt5Gui_DIR}/../../../plugins/iconengines
    ${Qt5Gui_DIR}/../../../plugins/imageformats
    ${Qt5Gui_DIR}/../../../plugins/xcbglintegrations
) 
install(DIRECTORY ${QT_PLUGIN}
        DESTINATION /home/jjst/JJURServer/bin
)

file(GLOB QT_LIB  LIST_DIRECTORIES false
    ${Qt5Gui_DIR}/../../*
)
install(FILES ${QT_LIB}
        DESTINATION /home/jjst/Qt_UR/Qt_UR_Lib
)

install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION /home/jjst/JJURServer/bin)


file(GLOB URServer_LIBRARY LIST_DIRECTORIES false
    ${JJURServerLib_SOURCE_DIR}/build/jjur_server/*.so
)
install(FILES ${URServer_LIBRARY} DESTINATION /home/jjst/JJURServer/Library)


file(GLOB URServer LIST_DIRECTORIES false
    ${JJURServerLib_SOURCE_DIR}/Scripts/URServer.desktop
)
install(FILES ${URServer}
        DESTINATION /home/jjst/Desktop
)

include (CPack)