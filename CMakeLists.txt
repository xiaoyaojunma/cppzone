#==============================================================================
# Copyright (c) 2011-2014 Evgeny Proydakov <lord.tiran@gmail.com>
#==============================================================================

CMAKE_MINIMUM_REQUIRED(VERSION 2.8.0)

PROJECT(I++)
MESSAGE(STATUS "CREATE  PROJECT  I++")

STRING(REGEX MATCH "Linux" LINUX ${CMAKE_SYSTEM_NAME})

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Debug)
    #SET(CMAKE_BUILD_TYPE Release)
ENDIF()

IF(NOT BUILD_SHARED_LIBS)
    SET(BUILD_SHARED_LIBS ON)
ENDIF()

IF(LINUX)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")
ENDIF()

IF(APPLE)
    SET(CMAKE_MACOSX_RPATH 1)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -stdlib=libc++")
    SET(CMAKE_XCODE_ATTRIBUTE_GCC_VERSION "com.apple.compilers.llvm.clang.1_0")
    SET(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LANGUAGE_STANDARD "c++11")
    SET(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LIBRARY           "libc++")
ENDIF()

IF(APPLE)
    SET(CMAKE_MACOSX_RPATH 1)
ENDIF()

IF(MSVC)
    ADD_DEFINITIONS(-D_USE_MATH_DEFINES)
    ADD_DEFINITIONS(-D_WIN32_WINNT=0x0501)
ENDIF()

SET(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_BINARY_DIR})
SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_BINARY_DIR})

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)
IF(WIN32)
    SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake/windows)
ENDIF()

# call system info script
INCLUDE(FindOS)

# include CMake macro
INCLUDE(imacro)
INCLUDE(icompiler)

COMPILER_SET_MAX_WARNING_LEVEL()

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR})

IF(MSVC)
    SET(Boost_USE_STATIC_LIBS    ON)
    SET(Boost_USE_MULTITHREADED  ON)
ENDIF()

SET_PROPERTY(GLOBAL PROPERTY USE_FOLDERS ON)

SET(PROJECT_FOLDER "null")

SET(I_PLUS_PLUS_DIRECTORY ${PROJECT_SOURCE_DIR})

SET(FILES
    ${I_PLUS_PLUS_DIRECTORY}/README
    ${I_PLUS_PLUS_DIRECTORY}/COPYING
    )

ADD_SUBDIRECTORY(asm)

ADD_SUBDIRECTORY(algorithms)

FIND_PACKAGE(Boost COMPONENTS thread system date_time regex chrono coroutine context)
IF(Boost_FOUND)
    ADD_SUBDIRECTORY(boost)
ENDIF()

ADD_SUBDIRECTORY(multithreading)
ADD_SUBDIRECTORY(stl)
ADD_SUBDIRECTORY(syntax)
ADD_SUBDIRECTORY(system)
ADD_SUBDIRECTORY(tasks)

IF(UNIX)
    ADD_SUBDIRECTORY(unix)
ENDIF()

IF(WIN32)
    ADD_SUBDIRECTORY(nt)
ENDIF()

IF(WIN32)
    FIND_PACKAGE(DIRECTX9)
    IF(DIRECTX9_FOUND AND MSVC)
        ADD_SUBDIRECTORY(D3DX9)
    ENDIF()
ENDIF()

FIND_PACKAGE(ALUT)
FIND_PACKAGE(OpenAL)
IF(ALUT_FOUND AND OPENAL_FOUND)
    ADD_SUBDIRECTORY(openal)
ENDIF()

IF(WIN32)
    SET(GLUT_ROOT_PATH $ENV{GLUT_ROOT_PATH})
ENDIF()

FIND_PACKAGE(GLUT)
FIND_PACKAGE(OpenGL)
IF(GLUT_FOUND AND OPENGL_FOUND)
    ADD_SUBDIRECTORY(glut)
    ADD_SUBDIRECTORY(opengl)
ENDIF()

FIND_PACKAGE(Lua51)
IF(LUA51_FOUND)
    ADD_SUBDIRECTORY(lua)
ENDIF()

SET(BUILD_OpenCV FALSE)

IF(BUILD_OpenCV)
    FIND_PACKAGE(OpenCV)
    IF(OpenCV_FOUND)
        ADD_SUBDIRECTORY(opencv)
    ENDIF()
ENDIF()

#==============================================================================
#---------------------------------- info --------------------------------------
#==============================================================================

# build info
SET(BUILD_INFO_BAR "====================================================================================")
SET(NOOP_STRING "")

MESSAGE(STATUS ${BUILD_INFO_BAR})
MESSAGE(STATUS "Summary of the build:")
MESSAGE(STATUS ${BUILD_INFO_BAR})
MESSAGE(STATUS "Build type : ${CMAKE_BUILD_TYPE}")
MESSAGE(STATUS ${BUILD_INFO_BAR})
MESSAGE(STATUS ${NOOP_STRING})
