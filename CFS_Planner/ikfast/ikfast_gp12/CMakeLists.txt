# cmake -DCMAKE_BUILD_TYPE=Release -DJulia_PREFIX=/opt/julia-1.10.4/ -DCMAKE_PREFIX_PATH=/root/.julia/artifacts/0c7b615ac941d356502c54c3f343cc97f247f3c1 ../
# cmake --build . --config Release

project(ikfast_gp12)

cmake_minimum_required(VERSION 3.5)
set(CMAKE_MACOSX_RPATH 1)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
set(CMAKE_CXX_FLAGS "-O3")

add_library(ikfast_gp12 SHARED ikfast_wrapper.cpp)
target_link_libraries(ikfast_gp12 -llapack)

add_executable(compute ikfast_wrapper.cpp)
target_link_libraries(compute -llapack)

install(TARGETS
  ikfast_gp12
LIBRARY DESTINATION lib
ARCHIVE DESTINATION lib
RUNTIME DESTINATION lib)