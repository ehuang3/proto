# Example executables
# project(example)

add_executable(mastermind mastermind.cpp)
target_link_libraries(mastermind proto)
set_target_properties(mastermind PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")

add_executable(gnuplot_example gnuplot_example.cpp)
target_link_libraries(gnuplot_example proto)
set_target_properties(gnuplot_example PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")


