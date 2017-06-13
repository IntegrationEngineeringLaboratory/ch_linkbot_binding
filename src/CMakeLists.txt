cmake_minimum_required(VERSION 3.2.1)
project(chlinkbot C CXX)

# Specify -DCH_ROOT=C:/path/to/ch on the command line, or set CH_ROOT as an
# environment variable, or we default to C:/Ch.
if(NOT DEFINED CH_ROOT)
    if (DEFINED ENV{CH_ROOT})
        set(CH_ROOT $ENV{CH_ROOT})
    else()
        set(CH_ROOT "C:/Ch")
    endif()
endif()
message(STATUS "Using Ch at ${CH_ROOT}")

find_package(Threads)

# In order to test the C++ wrapper easily, make two targets: a static
# linkbot_wrapper which links to baromesh and friends, and a shared chlinkbot
# target which only contains the Ch entry functions and links to the wrapper.

set(WRAPPER_SOURCES
    rgbhashtable.c.cpp
    )

find_library(LINKBOT linkbot PATHS /lib)

set(SOURCES
    chlinkbot_chdl.cpp
    chlinkboti_chdl.cpp
    chlinkbotl_chdl.cpp
    rgbhashtable.c.cpp
    linkbot_wrapper.cpp
    )

add_library(chlinkbot MODULE ${SOURCES})
set_property(TARGET chlinkbot PROPERTY CXX_STANDARD 11)
set_property(TARGET chlinkbot PROPERTY CXX_STANDARD_REQUIRED ON)

# We want the library to be named liblinkbot.dl, not chlinkbot.dll.
set_target_properties(chlinkbot PROPERTIES PREFIX lib SUFFIX .dl OUTPUT_NAME linkbot)

if(APPLE)
    set_target_properties(chlinkbot PROPERTIES LINK_FLAGS "-undefined dynamic_lookup")
endif()

target_include_directories(chlinkbot
    PRIVATE
            ${CH_ROOT}/extern/include
            /include
    )
target_include_directories(chlinkbot PRIVATE ${CMAKE_SOURCE_DIR}/c-stem/robotics)

if(WIN32)
    find_library(CHLIB ch PATHS "${CH_ROOT}/extern/lib" NO_DEFAULT_PATH)
endif()
find_library(CHSDKLIB chsdk PATHS "${CH_ROOT}/extern/lib" NO_DEFAULT_PATH)
set(chLibraries ${CHLIB} ${CHSDKLIB})
message(STATUS "Found Ch libraries: ${chLibraries}")

target_link_libraries(chlinkbot
            ${chLibraries}
            linkbot
            c_stem_robotics
    )

install(TARGETS chlinkbot LIBRARY DESTINATION chbarobo/dl)
