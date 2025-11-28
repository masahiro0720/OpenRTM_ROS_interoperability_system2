# RTMtoROS CMake config file
#
# This file sets the following variables:
# RTMtoROS_FOUND - Always TRUE.
# RTMtoROS_INCLUDE_DIRS - Directories containing the RTMtoROS include files.
# RTMtoROS_IDL_DIRS - Directories containing the RTMtoROS IDL files.
# RTMtoROS_LIBRARIES - Libraries needed to use RTMtoROS.
# RTMtoROS_DEFINITIONS - Compiler flags for RTMtoROS.
# RTMtoROS_VERSION - The version of RTMtoROS found.
# RTMtoROS_VERSION_MAJOR - The major version of RTMtoROS found.
# RTMtoROS_VERSION_MINOR - The minor version of RTMtoROS found.
# RTMtoROS_VERSION_REVISION - The revision version of RTMtoROS found.
# RTMtoROS_VERSION_CANDIDATE - The candidate version of RTMtoROS found.

message(STATUS "Found RTMtoROS-1.0.0")
set(RTMtoROS_FOUND TRUE)

find_package(<dependency> REQUIRED)

#set(RTMtoROS_INCLUDE_DIRS
#    "/usr/local/include/rtmtoros-1"
#    ${<dependency>_INCLUDE_DIRS}
#    )
#
#set(RTMtoROS_IDL_DIRS
#    "/usr/local/include/rtmtoros-1/idl")
set(RTMtoROS_INCLUDE_DIRS
    "/usr/local/include/"
    ${<dependency>_INCLUDE_DIRS}
    )
set(RTMtoROS_IDL_DIRS
    "/usr/local/include//idl")


if(WIN32)
    set(RTMtoROS_LIBRARIES
        "/usr/local//librtmtoros.a"
        ${<dependency>_LIBRARIES}
        )
else(WIN32)
    set(RTMtoROS_LIBRARIES
        "/usr/local//librtmtoros.so"
        ${<dependency>_LIBRARIES}
        )
endif(WIN32)

set(RTMtoROS_DEFINITIONS ${<dependency>_DEFINITIONS})

set(RTMtoROS_VERSION 1.0.0)
set(RTMtoROS_VERSION_MAJOR 1)
set(RTMtoROS_VERSION_MINOR 0)
set(RTMtoROS_VERSION_REVISION 0)
set(RTMtoROS_VERSION_CANDIDATE )

