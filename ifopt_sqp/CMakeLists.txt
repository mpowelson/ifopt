set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

find_package(OsqpEigen REQUIRED)

###########
## Build ##
###########
set(LIB_SQP_SOLVER ifopt_sqp)
# The osqp_eigen solver interface
add_library(${LIB_SQP_SOLVER} SHARED
    src/osqp_eigen_solver.cc
    src/simple_sqp_solver.cc
)
target_include_directories(${LIB_SQP_SOLVER}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${LIB_SQP_SOLVER}
  PUBLIC
    ${LIB_CORE}
    OsqpEigen::OsqpEigen
)


#############
## Testing ##
#############
add_executable(${LIB_SQP_SOLVER}-example test/ex_test_osqp_eigen.cc)
target_link_libraries(${LIB_SQP_SOLVER}-example
  PRIVATE
    ${LIB_SQP_SOLVER}
)
add_test(${LIB_SQP_SOLVER}-example ${LIB_SQP_SOLVER}-example)


#############
## Install ##
#############
 # Copy library files to usr/local/lib/libifopt_osqp_eigen.so
install(
  TARGETS ${LIB_SQP_SOLVER} ${LIB_SQP_SOLVER}-example
  EXPORT  ${LIB_SQP_SOLVER}-targets
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
install(EXPORT ${LIB_SQP_SOLVER}-targets
  FILE ${LIB_SQP_SOLVER}-targets.cmake
  NAMESPACE ifopt::
  DESTINATION ${config_package_location}
)



