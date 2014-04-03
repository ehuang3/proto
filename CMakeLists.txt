CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
PROJECT(prototype)
MESSAGE(STATUS "Creating Makefile for PROTOTYPE")
MESSAGE(STATUS "CMAKE_SOURCE_DIR = ${CMAKE_SOURCE_DIR}")
MESSAGE(STATUS "CMAKE_BINARY_DIR = ${CMAKE_BINARY_DIR}")

###############################################################################
#### Build type                                                             ###
###############################################################################
IF(NOT CMAKE_BUILD_TYPE)
   SET(CMAKE_BUILD_TYPE Debug CACHE STRING
       "Choose the type of build, options are: None Debug Release RelWithDebInfo Profile."
       FORCE)
ENDIF(NOT CMAKE_BUILD_TYPE)

###############################################################################
### Color gcc                                                               ###
###############################################################################
IF(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_IS_GNUCPP)
   OPTION(COLOR_GCC "Use GCCFilter to color compiler output messages" OFF)
   SET(COLOR_GCC_OPTIONS "-c -r -w -n")
   IF(COLOR_GCC)
      SET_PROPERTY(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CMAKE_SOURCE_DIR}/tools/gccfilter ${COLOR_GCC_OPTIONS}")
   ENDIF()
ENDIF()

###############################################################################
### Compile flags                                                           ###
###############################################################################
IF("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  SET(CMAKE_INSTALL_PREFIX /usr )
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse2 -std=c++0x")
  SET(CMAKE_CXX_FLAGS_RELEASE "-O3")
  SET(CMAKE_CXX_FLAGS_DEBUG "-g -fno-omit-frame-pointer -fno-inline-functions -fno-inline-functions-called-once -fno-optimize-sibling-calls")
  SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELEASE} ${CMAKE_CXX_FLAGS_DEBUG}")
  SET(CMAKE_CXX_FLAGS_PROFILE "${CMAKE_CXX_FLAGS_DEBUG} -pg")
ELSE()
  SET(CMAKE_INSTALL_PREFIX /usr )
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse2 -std=c++0x")
  SET(CMAKE_CXX_FLAGS_RELEASE "-O3")
  SET(CMAKE_CXX_FLAGS_DEBUG "-g -fno-omit-frame-pointer -fno-inline-functions -fno-inline-functions-called-once -fno-optimize-sibling-calls")
  SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELEASE} ${CMAKE_CXX_FLAGS_DEBUG}")
  SET(CMAKE_CXX_FLAGS_PROFILE "${CMAKE_CXX_FLAGS_DEBUG} -pg")
ENDIF()

SET(CMAKE_DEBUG_POSTFIX "d")

###############################################################################
### Generate path header file                                               ###
###############################################################################
SET(PATH_HEADER_DIR ${CMAKE_SOURCE_DIR}/src/utils/ProtoPaths.h)
MESSAGE(STATUS "Generate the path header file to ${PATH_HEADER_DIR}")
CONFIGURE_FILE(${PATH_HEADER_DIR}.in ${PATH_HEADER_DIR} @ONLY)

###############################################################################
### Link with libraries                                                     ###
###############################################################################
FIND_PACKAGE(Eigen REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIR})

SET(CURSES_NEED_NCURSES TRUE)
FIND_PACKAGE(Curses REQUIRED)
INCLUDE_DIRECTORIES(${CURSES_INCLUDE_DIR})
SET(Curses_LIBRARIES curses)

FIND_PACKAGE(PkgConfig REQUIRED)
PKG_CHECK_MODULES(LOG4CXX REQUIRED liblog4cxx)
INCLUDE_DIRECTORIES(${LOG4CXX_INCLUDE_DIRS})
SET(Log4cxx_LIBRARIES log4cxx)

SET(PROTO_LIBRARIES ${Curses_LIBRARIES} ${Log4cxx_LIBRARIES})
MESSAGE(STATUS "PROTO_LIBRARIES: ${PROTO_LIBRARIES}")

###############################################################################
### Build source libraries                                                  ###
###############################################################################
MESSAGE(STATUS "Prepare PROTOTYPE source")
INCLUDE_DIRECTORIES(BEFORE src)
LINK_DIRECTORIES(${CMAKE_SOURCE_DIR}/lib)
ADD_SUBDIRECTORY(src)

###############################################################################
### Build Unit tests                                                        ###
###############################################################################
ENABLE_TESTING()
ADD_SUBDIRECTORY(test)

###############################################################################
### Build target applications                                               ###
###############################################################################
ADD_SUBDIRECTORY(apps)
