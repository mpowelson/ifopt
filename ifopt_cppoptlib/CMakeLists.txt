set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

find_package(cpp_numerical_solvers REQUIRED)
message(WARNING ${cpp_numerical_solvers_INCLUDE_DIRS})
###########
## Build ##
###########
set(LIB_cppoptlib ifopt_cpp_numerical_solvers)
# The cppoptlib solver interface
add_library(${LIB_cppoptlib} SHARED
    src/cppoptlib_adapter.cc
    src/cppoptlib_solver.cc
)
target_include_directories(${LIB_cppoptlib}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${LIB_cppoptlib}
  PUBLIC
    ${LIB_CORE}
    trajopt::cpp_numerical_solvers
)
target_compile_definitions(${LIB_cppoptlib}
  PRIVATE
    ${cppoptlib_DEFINITIONS}
)


#############
## Testing ##
#############
add_executable(${LIB_cppoptlib}-example test/ex_test_cppoptlib.cc)
target_link_libraries(${LIB_cppoptlib}-example
  PRIVATE
    ${LIB_cppoptlib}
)
add_test(${LIB_cppoptlib}-example ${LIB_cppoptlib}-example)


#############
## Install ##
#############
 # Copy library files to usr/local/lib/libifopt_cppoptlib.so
install(
  TARGETS ${LIB_cppoptlib} ${LIB_cppoptlib}-example
  EXPORT  ${LIB_cppoptlib}-targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${IFOPT_INSTALL_BINDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
# Copy header files to usr/local/include/ifopt/*
install(DIRECTORY include/ifopt/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ifopt
)
# install a *.cmake file in destination
install(EXPORT ${LIB_cppoptlib}-targets
  FILE ${LIB_cppoptlib}-targets.cmake
  NAMESPACE ifopt::
  DESTINATION ${config_package_location}
)



