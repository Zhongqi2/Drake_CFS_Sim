# cmake -DCMAKE_BUILD_TYPE=Release -DJulia_PREFIX=/opt/julia-1.10.4/ -DCMAKE_PREFIX_PATH=/root/.julia/artifacts/0c7b615ac941d356502c54c3f343cc97f247f3c1 ../
# cmake --build . --config Release


project(ikfast)

cmake_minimum_required(VERSION 3.5)
set(CMAKE_MACOSX_RPATH 1)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
set(CMAKE_CXX_FLAGS "-O3")

INCLUDE_DIRECTORIES(ikfast)

add_library(ikfast_gp7 SHARED ikfast/ikfast61gp7.cpp)
target_link_libraries(ikfast_gp7 -llapack)

add_library(ikfast_gp12 SHARED ikfast/ikfast61gp12.cpp)
target_link_libraries(ikfast_gp12 -llapack)

install(TARGETS
  ikfast_gp7
  ikfast_gp12
LIBRARY DESTINATION lib
ARCHIVE DESTINATION lib
RUNTIME DESTINATION lib)