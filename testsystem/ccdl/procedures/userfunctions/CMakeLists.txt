# packages all files as a source code sample
#
###############################################################################

cmake_minimum_required(VERSION 2.6)

#----------------------
# project build settings
#----------------------

###############################################################################

# install(CODE 
#   "EXECUTE_PROCESS(COMMAND ${CMAKE_COMMAND} -E tar czvf ${CMAKE_BINARY_DIR}/samplecode.tar.gz 
#   ${CMAKE_SOURCE_DIR}/samplecode)"
# )
# add_custom_command( OUTPUT samplecode.tar.gz
#   COMMAND ${CMAKE_COMMAND} -E tar czvf ${CMAKE_BINARY_DIR}/samplecode.tar.gz samplecode
#   WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
#   COMMENT "packing..."
#   )
add_custom_target( pack ALL
  COMMAND ${CMAKE_COMMAND} -E tar czvf ${CMAKE_CURRENT_BINARY_DIR}/samplecode.tar.gz samplecode
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "Pack sample code..."
  )
set_property(DIRECTORY APPEND PROPERTY
    ADDITIONAL_MAKE_CLEAN_FILES
    ${CMAKE_CURRENT_BINARY_DIR}/samplecode.tar.gz)

# do not install, let rpmbuild pick it up from ${CMAKE_CURRENT_BINARY_DIR}
#install(FILES ${CMAKE_CURRENT_BINARY_DIR}/samplecode.tar.gz DESTINATION ${CMAKE_INSTALL_DOCDIR})
