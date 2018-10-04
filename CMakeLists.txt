cmake_minimum_required(VERSION 3.2)

set(CMAKE_USER_MAKE_RULES_OVERRIDE ${CMAKE_CURRENT_SOURCE_DIR}/c_flag_overrides.cmake)
set(CMAKE_USER_MAKE_RULES_OVERRIDE_CXX ${CMAKE_CURRENT_SOURCE_DIR}/cxx_flag_overrides.cmake)

project(asrc C CXX)

unset(ASRC_STANDALONE)
if (${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})
    set (ASRC_STANDALONE TRUE)
    option(ASRC_BUILD_STATIC "" OFF)
else()
    option(ASRC_BUILD_STATIC "" ON)
endif()

if (ASRC_BUILD_STATIC)
    set (ASRC_LINK_MODE STATIC)
else ()
    set (ASRC_LINK_MODE SHARED)
    if (WIN32)
        set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
    endif ()
endif ()

if (NOT MSVC)
    if (${CMAKE_GENERATOR} STREQUAL "Ninja")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fdiagnostics-color=always")
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fdiagnostics-color=always")
    endif()
    if (APPLE)
        set(CMAKE_XCODE_ATTRIBUTE_MACOSX_DEPLOYMENT_TARGET "10.12")
    endif()
endif()

if (ASRC_STANDALONE)
    option(ASRC_ENABLE_TEST "" ON)

    if (NOT CMAKE_BUILD_TYPE)
        set(CMAKE_BUILD_TYPE "RELEASE" CACHE STRING
            "Choose the type of build, options are: RELEASE (default) and DEBUG")
    endif()

    add_subdirectory(externals)
endif ()

add_subdirectory(asrc)

if (ASRC_ENABLE_TEST)
    enable_testing()
    #add_subdirectory(test)
endif ()

unset(ASRC_STANDALONE)

