cmake_minimum_required(VERSION 3.16)
project(g2o_ba_demo LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)
find_package(g2o REQUIRED)

add_executable(test_g2o main.cpp)

# Modern, namespaced targets (preferred)
if(TARGET g2o::core)
  target_link_libraries(test_g2o PRIVATE
    g2o::core
    g2o::stuff
    g2o::types_sba
    g2o::solver_cholmod
    Eigen3::Eigen
  )
else()
  # Fallback for older g2o installs
  find_package(SuiteSparse REQUIRED)
  target_link_libraries(test_g2o PRIVATE
    g2o_core
    g2o_stuff
    g2o_types_sba
    g2o_solver_cholmod
    Eigen3::Eigen
    ${SuiteSparse_LIBRARIES}
  )
endif()
