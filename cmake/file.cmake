
#! generate_cpp_sources
#
# cmake function for generating cpp sources
# a file is only generated if it does not exist
#
#
# \arg FILES: file list of relative paths
function(generate_cpp_sources)
    set(options "")
    set(oneValueArgs "")
    set(multiValueArgs FILES)
    cmake_parse_arguments(ARGS "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    foreach(file ${ARGS_FILES})
        set(file_path ${CMAKE_CURRENT_SOURCE_DIR}/${file})
        if(NOT EXISTS ${file_path})
            get_filename_component(extension ${file_path} EXT)
            get_filename_component(file_name ${file_path} NAME_WE)
            if(${extension} STREQUAL ".h")
                string(REPLACE "." "_" cpp_guard ${file})
                string(REPLACE "/" "_" cpp_guard ${cpp_guard})
                string(TOUPPER ${cpp_guard} cpp_guard)
                set(cpp_guard "__${cpp_guard}_")
                configure_file(${CMAKE_SRC}/cpp_header.in ${file_path})
            elseif(extension STREQUAL ".cpp")
                configure_file(${CMAKE_SRC}/cpp.in ${file_path})
            endif()
        endif()
    endforeach()
endfunction()
