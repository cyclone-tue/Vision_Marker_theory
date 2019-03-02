

set(command "/snap/clion/61/bin/cmake/linux/bin/cmake;-P;/home/arnoud/Github/Vision_Marker_theory/lib/tmp/spdlog-gitclone.cmake")
execute_process(
  COMMAND ${command}
  RESULT_VARIABLE result
  OUTPUT_FILE "/home/arnoud/Github/Vision_Marker_theory/lib/src/spdlog-stamp/spdlog-download-out.log"
  ERROR_FILE "/home/arnoud/Github/Vision_Marker_theory/lib/src/spdlog-stamp/spdlog-download-err.log"
  )
if(result)
  set(msg "Command failed: ${result}\n")
  foreach(arg IN LISTS command)
    set(msg "${msg} '${arg}'")
  endforeach()
  set(msg "${msg}\nSee also\n  /home/arnoud/Github/Vision_Marker_theory/lib/src/spdlog-stamp/spdlog-download-*.log")
  message(FATAL_ERROR "${msg}")
else()
  set(msg "spdlog download command succeeded.  See also /home/arnoud/Github/Vision_Marker_theory/lib/src/spdlog-stamp/spdlog-download-*.log")
  message(STATUS "${msg}")
endif()
