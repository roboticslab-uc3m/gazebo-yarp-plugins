# InstallGazeboPlugin.cmake
include_guard(GLOBAL)

macro(install_gazebo_plugin target)
    execute_process(COMMAND "${GAZEBO_ROOT_DIR}/bin/gazebo-config" --prefix --plugins-dir
                    OUTPUT_VARIABLE _gazebo_config_cmd_output
                    OUTPUT_STRIP_TRAILING_WHITESPACE
                    RESULT_VARIABLE _gazebo_config_res)

    if(_gazebo_config_res)
        message(FATAL_ERROR "Unable to install Gazebo plugins. \"${GAZEBO_ROOT_DIR}/bin/gazebo-config\": ${_gazebo_config_res}")
    endif()

    string(REPLACE "\n" ";" _gazebo_config_results ${_gazebo_config_cmd_output})
    list(LENGTH _gazebo_config_results _len)

    if(_len EQUAL 2)
        list(GET _gazebo_config_results 0 _gazebo_install_prefix)
        list(GET _gazebo_config_results 1 _gazebo_plugin_path)
        file(RELATIVE_PATH _relative_path "${_gazebo_install_prefix}" "${_gazebo_plugin_path}")
        install(TARGETS ${target} DESTINATION ${_relative_path})
    else()
        message(AUTHOR_WARNING "Unsuccessful call to 'gazebo-config', the output was: ${_gazebo_config_cmd_output}")
    endif()
endmacro()
