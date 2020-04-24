set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

find_package(OsqpEigen REQUIRED)

###########
## Build ##
###########
set(LIB_osqp_eigen ifopt_osqp_eigen)
# The osqp_eigen solver interface
add_library(${LIB_osqp_eigen} SHARED
    src/osqp_eigen_solver.cc
    src/sqp_solver.cc
)
target_include_directories(${LIB_osqp_eigen}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${LIB_osqp_eigen}
  PUBLIC
    ${LIB_CORE}
    OsqpEigen::OsqpEigen
)
target_compile_definitions(${LIB_osqp_eigen}
  PRIVATE
    ${osqp_eigen_DEFINITIONS}
)


#############
## Testing ##
#############
add_executable(${LIB_osqp_eigen}-example test/ex_test_osqp_eigen.cc)
target_link_libraries(${LIB_osqp_eigen}-example
  PRIVATE
    ${LIB_osqp_eigen}
)
add_test(${LIB_osqp_eigen}-example ${LIB_osqp_eigen}-example)


#############
## Install ##
#############
 # Copy library files to usr/local/lib/libifopt_osqp_eigen.so
install(
  TARGETS ${LIB_osqp_eigen} ${LIB_osqp_eigen}-example
  EXPORT  ${LIB_osqp_eigen}-targets
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
install(EXPORT ${LIB_osqp_eigen}-targets
  FILE ${LIB_osqp_eigen}-targets.cmake
  NAMESPACE ifopt::
  DESTINATION ${config_package_location}
)



