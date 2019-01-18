# Project: CubicSpline
#
# All rights reserved.

cmake_minimum_required( VERSION 2.6 )
cmake_policy( SET CMP0004 OLD )

### initialize project ###########################################################################################

SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")


project(CubicSpline)


set(CMAKE_INSTALL_PREFIX /usr)


set(BUILD_SHARED_LIBS on)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -O0")


add_definitions( "-DPREFIX=\"${CMAKE_INSTALL_PREFIX}\""        )

### global default options #######################################################################################

set(SOURCES 
    main.cpp
    CubicSpline.cpp
    point.cpp 
)



add_executable(CubicSpline ${SOURCES})

# referenced targets

#add_dependencies(CubicSpline  )

#target_link_libraries(CubicSpline  )

### install ######################################################################################################



install(TARGETS CubicSpline DESTINATION lib)

