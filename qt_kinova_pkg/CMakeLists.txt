##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.0)
project(qt_kinova_pkg)

##############################################################################
# Catkin
##############################################################################

# qt_build provides the qt cmake glue, roscpp the comms for a default talker
find_package(catkin REQUIRED COMPONENTS qt_build roscpp image_transport cv_bridge)
include_directories(${catkin_INCLUDE_DIRS})
# Use this to define what the package will export (e.g. libs, headers).
# Since the default here is to produce only a binary, we don't worry about
# exporting anything.
catkin_package()

##############################################################################
# OpenCV
##############################################################################
# Specify OpenCV folder, and take care of dependenciesand includes

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

##############################################################################
# Qt Environment
##############################################################################

# this comes from qt_build's qt-ros.cmake which is automatically
# included via the dependency call in package.xml
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
#SET(CMAKE_AUTOUIC ON)

find_package(Qt5 COMPONENTS Core Gui Widgets PrintSupport) # Add the appropriate components to the component list here

##############################################################################
# Sections
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)


QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})

include_directories(
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Gui_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
    ${Qt5PrintSupport_INCLUDE_DIRS}
    )

add_definitions( -std=c++11 -fPIC)

##############################################################################
# Sources
##############################################################################

set(QT_SOURCES
    src/main.cpp
    src/main_window.cpp
    src/qnode.cpp

    include/qt_kinova_pkg/main_window.hpp
    include/qt_kinova_pkg/qnode.hpp
    )

##############################################################################
# Binaries
##############################################################################

add_executable(qt_kinova_pkg ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(qt_kinova_pkg
${catkin_LIBRARIES}
    Qt5::Core
    Qt5::Widgets
    Qt5::PrintSupport)
install(TARGETS qt_kinova_pkg RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

