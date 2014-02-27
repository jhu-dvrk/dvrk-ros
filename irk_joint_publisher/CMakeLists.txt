cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()





# set cisst dependencies
set (REQUIRED_CISST_LIBRARIES
  cisstCommon
  cisstVector
  cisstOSAbstraction
  cisstMultiTask
  cisstParameterTypes
  )


# find cisst
find_package (cisst REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND)
  # Modify CMake configuration to use cisst
  include (${CISST_USE_FILE})

  # source
  set (SOURCE_FILES
    src/main.cpp)
  
  # rosbuild_add_executable(dv_jsp src/main.cpp)
  # cisst_target_link_libraries(dv_jsp ${REQUIRED_CISST_LIBRARIES} )

endif (cisst_FOUND)