project(FishSwarm)
cmake_minimum_required(VERSION 2.8.0)
enable_testing()

#set(CMAKE_POSITION_INDEPENDENT_CODE ON)
SET(CMAKE_BUILD_TYPE Debug)
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/modules")
set(ENV{FISHSWARM_DIR} ${CMAKE_SOURCE_DIR}) 

find_package(Qt5 REQUIRED Core Widgets DBus)
FIND_PACKAGE(Boost 1.40 COMPONENTS program_options REQUIRED )
INCLUDE_DIRECTORIES(SYSTEM ${Boost_INCLUDE_DIR} )


# SYSTEM is used to suppress GCC warnings in libraries headers
include_directories(SYSTEM ${QT_INCLUDE_DIR})

#add_definitions(${QTGSTREAMER_DEFINITIONS}) # defines QT_NO_KEYWORDS that demands replacing all "signals" by "Q_SIGNALS"
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${QTGSTREAMER_FLAGS}")

set(FISHSWARM_CXX_FLAGS "-std=c++11 -Wall -Wextra -Wcast-align -Wcast-qual -Wdisabled-optimization -Wformat=2 \
-Wlogical-op -Wmissing-declarations -Wmissing-include-dirs -Wnoexcept -Woverloaded-virtual \
-Wredundant-decls -Wstrict-null-sentinel -Wstrict-overflow=5 -Wswitch-default \
-Wno-unused-parameter -Wold-style-cast  -Wsuggest-override \
-Wsuggest-final-methods -Wfloat-equal -Wundef -Wshadow")

if (CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${FISHSWARM_CXX_FLAGS}")
endif ()

if ("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${FISHSWARM_CXX_FLAGS} -Wnull-dereference")
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${FISHSWARM_CXX_FLAGS}")
endif()

set(srcs
    main.cpp
    target.cpp
    swarminterface.cpp
    fishrobot.cpp
    djikstraboost.cpp
    potentialfield.cpp
    dynamicwindow.cpp
    priorityplanning.cpp
    rectangle.cpp
    constants.h
)

qt5_wrap_ui(uis_hdrs swarminterface.ui)

qt5_add_resources(resources icons.qrc)

add_executable(fishswarm  ${srcs} ${uis_hdrs} ${resources})
target_link_libraries(fishswarm Qt5::Core Qt5::Widgets Qt5::Gui Qt5::DBus ${Boost_LIBRARIES} )

