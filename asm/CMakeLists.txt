#==============================================================================
# Copyright (c) 2016 Evgeny Proydakov <lord.tiran@gmail.com>
#==============================================================================
# CMake for  EXAMPLE  PROJECT
#==============================================================================

MESSAGE(STATUS "BUILD ASM EXAMPLE\n")

SET(PROJECT_FOLDER "asm")

IF(MSVC)
    ADD_SUBDIRECTORY(amd64masm)
ENDIF()

MESSAGE("")
