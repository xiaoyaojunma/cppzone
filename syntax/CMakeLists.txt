#==============================================================================
# Copyright (c) 2012 Evgeny Proydakov <lord.tiran@gmail.com>
#==============================================================================
# CMake for  SYNTAX  PROJECT
#==============================================================================

MESSAGE(STATUS "BUILD  SYNTAX\n")

SET(PROJECT_FOLDER "syntax")

SET(DIAMOND_INHERITANCE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/diamond_inheritance)

ADD_SUBDIRECTORY(adress_base)
ADD_SUBDIRECTORY(bind)
ADD_SUBDIRECTORY(caller)
ADD_SUBDIRECTORY(cycle)
ADD_SUBDIRECTORY(typeinfo)
ADD_SUBDIRECTORY(diamond_inheritance_basic)
ADD_SUBDIRECTORY(diamond_inheritance_casting)
ADD_SUBDIRECTORY(macro)
ADD_SUBDIRECTORY(exception_basic)
ADD_SUBDIRECTORY(exception_in_constructor)
ADD_SUBDIRECTORY(inheritance_basic)
ADD_SUBDIRECTORY(inheritance_casting)
ADD_SUBDIRECTORY(operator_brackets)
ADD_SUBDIRECTORY(operator_cast_type)
ADD_SUBDIRECTORY(initialization)
ADD_SUBDIRECTORY(sizeof)
ADD_SUBDIRECTORY(numeric_limits)
ADD_SUBDIRECTORY(return_const_ref)
ADD_SUBDIRECTORY(template_fun)
ADD_SUBDIRECTORY(template_explicit_specialization_function)
ADD_SUBDIRECTORY(variadic_templates)
ADD_SUBDIRECTORY(unique_ptr)
ADD_SUBDIRECTORY(union)

MESSAGE("")
