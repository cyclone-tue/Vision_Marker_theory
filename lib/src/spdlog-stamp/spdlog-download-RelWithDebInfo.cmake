

set(command "/usr/bin/cmake;-P;/home/stijn/Documents/tue/cyclone/Vision_Marker_theory/lib/tmp/spdlog-gitclone.cmake")
execute_process(
  COMMAND ${command}
  RESULT_VARIABLE result
  OUTPUT_FILE "/home/stijn/Documents/tue/cyclone/Vision_Marker_theory/lib/src/spdlog-stamp/spdlog-download-out.log"
  ERROR_FILE "/home/stijn/Documents/tue/cyclone/Vision_Marker_theory/lib/src/spdlog-stamp/spdlog-download-err.log"
  )
if(result)
  set(msg "Command failed: ${result}\n")
  foreach(arg IN LISTS command)
    set(msg "${msg} '${arg}'")
  endforeach()
  set(msg "${msg}\nSee also\n  /home/stijn/Documents/tue/cyclone/Vision_Marker_theory/lib/src/spdlog-stamp/spdlog-download-*.log")
  message(FATAL_ERROR "${msg}")
else()
  set(msg "spdlog download command succeeded.  See also /home/stijn/Documents/tue/cyclone/Vision_Marker_theory/lib/src/spdlog-stamp/spdlog-download-*.log")
  message(STATUS "${msg}")
endif()
