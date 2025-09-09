cmake_minimum_required(VERSION 3.5.1)
add_definitions(-std=c++11)
add_definitions(-D _GLIBCXX_USE_CXX11_ABI=0)

project(JJURServerLib)

#设置库文件是否动态编译
if(NOT DEFINED BUILD_SHARED_LIBS)
  option(BUILD_SHARED_LIBS "Build with shared libraries." ON)
endif()

#设置是否编译Test
if(NOT DEFINED BUILD_TESTING)
  option(BUILD_TESTING "Build Test" ON)
endif()

set(QT_USE_QTNETWORK TRUE)
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(Qt5Gui REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Threads REQUIRED)
find_package(Qt5Network REQUIRED)

include_directories(${QT_USE_FILE})
include_directories(${Qt5Core_INCLUDE_DIRS})
include_directories(${Qt5Widgets_INCLUDE_DIRS})
include_directories(${Qt5Gui_INCLUDE_DIRS})

set(QT_USE_QTMAIN TRUE) #使用qtmain库，去除黑窗口

ADD_SUBDIRECTORY(jjur_server)

if(BUILD_TESTING)
  ADD_SUBDIRECTORY(jjur_server_test)
endif()



