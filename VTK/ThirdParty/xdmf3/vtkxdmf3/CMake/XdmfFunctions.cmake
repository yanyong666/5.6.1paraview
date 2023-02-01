function(xdmf_create_config_file name)
  get_cmake_property(ALL_VARS VARIABLES)
  set(XDMF_VARS "")
  foreach(var ${ALL_VARS})
    if (NOT "x${${var}}" STREQUAL "x")
      string(REGEX REPLACE "\\\\" "\\\\\\\\" ${var} "${${var}}")
      if (var MATCHES "^(XDMF).*$")
        set(XDMF_VARS "${XDMF_VARS}\nset(${var}\t\t\"${${var}}\")")
      else()
        set(XDMF_VARS "${XDMF_VARS}\nset(XDMF_${var}\t\t\"${${var}}\")")
      endif()
    endif()
  endforeach()
  configure_file( ${CMAKE_CURRENT_SOURCE_DIR}/${name}Config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/${name}Config.cmake @ONLY)
endfunction()