# project(prototype)

# Setup directory structure
set(SUBDIRS utils)
foreach(SUBDIR ${SUBDIRS})
	file(GLOB_RECURSE dir_files ${SUBDIR}/*.h ${SUBDIR}/*.hpp ${SUBDIR}/*.cpp)
	source_group(${SUBDIR} FILES ${dir_files})
endforeach()

# Search all header and source files
file(GLOB_RECURSE proto_srcs *.cpp)
file(GLOB_RECURSE proto_hdrs *.h *.hpp)

# Library
add_library(proto ${proto_srcs} ${proto_hdrs})
target_link_libraries(proto ${PROTO_LIBRARIES})
set_target_properties(proto PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib" ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib")
