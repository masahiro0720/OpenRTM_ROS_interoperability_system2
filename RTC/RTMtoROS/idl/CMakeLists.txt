cmake_minimum_required(VERSION 2.8)

get_filename_component(PROJECT_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
set(CMAKE_MODULE_PATH ${PROJECT_ROOT_DIR}/RTC/ManipulatorControlSample/cmake/Modules ${CMAKE_MODULE_PATH})

find_package(OpenRTM REQUIRED)

# 【修正点】ここから Stub.cpp を削除しました。
# これにより、Stub.cpp はコンパイル対象リスト(ALL_IDL_SRCS)に含まれなくなり、多重定義エラーが解消されます。
macro(_IDL_OUTPUTS _idl _dir _result)
    set(${_result} ${_dir}/${_idl}Skel.cpp ${_dir}/${_idl}Skel.h ${_dir}/${_idl}Stub.h)
endmacro(_IDL_OUTPUTS)

macro(_COMPILE_IDL _idl_file)
    execute_process(COMMAND rtm-config --idlc OUTPUT_VARIABLE OPENRTM_IDLC OUTPUT_STRIP_TRAILING_WHITESPACE)
    execute_process(COMMAND rtm-config --idlflags OUTPUT_VARIABLE OPENRTM_IDLFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
    separate_arguments(OPENRTM_IDLFLAGS)
    
    get_filename_component(_idl ${_idl_file} NAME_WE)
    set(_idl_srcs_var ${_idl}_SRCS)
    _IDL_OUTPUTS(${_idl} ${CMAKE_CURRENT_BINARY_DIR} ${_idl_srcs_var})

    add_custom_command(OUTPUT ${${_idl_srcs_var}}
        COMMAND python -u /usr/bin/rtm-skelwrapper --include-dir="" --skel-suffix=Skel --stub-suffix=Stub --idl-file=${CMAKE_CURRENT_SOURCE_DIR}/${_idl}.idl
        COMMAND ${OPENRTM_IDLC} ${OPENRTM_IDLFLAGS} -I${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/${_idl}.idl
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        DEPENDS ${_idl_file}
        COMMENT "Compiling ${_idl_file}")

    add_custom_target(${_idl}_TGT DEPENDS ${${_idl_srcs_var}})
    set(ALL_IDL_SRCS ${ALL_IDL_SRCS} ${${_idl_srcs_var}})
    
    if(NOT TARGET ALL_IDL_TGT)
        add_custom_target(ALL_IDL_TGT)
    endif(NOT TARGET ALL_IDL_TGT)
    add_dependencies(ALL_IDL_TGT ${_idl}_TGT)
endmacro(_COMPILE_IDL)

macro(OPENRTM_COMPILE_IDL_FILES)
    foreach(idl ${ARGN})
        _COMPILE_IDL(${idl})
    endforeach(idl)
endmacro(OPENRTM_COMPILE_IDL_FILES)

set(idls 
    ManipulatorCommonInterface_Common.idl 
    ManipulatorCommonInterface_MiddleLevel.idl 
    ManipulatorCommonInterface_DataTypes.idl 
    BasicDataType.idl
)

OPENRTM_COMPILE_IDL_FILES(${idls})
set(ALL_IDL_SRCS ${ALL_IDL_SRCS} PARENT_SCOPE)

install(FILES ${idls} DESTINATION include/idl COMPONENT idl)
