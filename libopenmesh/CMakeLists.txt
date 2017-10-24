cmake_minimum_required(VERSION 2.8.3)
project(libopenmesh)

set(VERSION 3.2)
set(LOCAL_CORE_LIB ${PROJECT_SOURCE_DIR}/lib/OpenMesh/libOpenMeshCore.so.${VERSION})
set(LOCAL_TOOLS_LIB ${PROJECT_SOURCE_DIR}/lib/OpenMesh/libOpenMeshTools.so.${VERSION})
set(GLOBAL_CORE_LIB ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/libOpenMeshCore.so)
set(GLOBAL_TOOLS_LIB ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/libOpenMeshTools.so)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)
add_custom_target(
  build_openmesh ALL
  COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR} make -f Makefile.tarball.${VERSION}
)



add_custom_target(
  copy_openmesh ALL
  COMMAND cmake -E copy ${LOCAL_CORE_LIB} ${GLOBAL_CORE_LIB}.${VERSION}
  COMMAND cmake -E copy ${LOCAL_TOOLS_LIB} ${GLOBAL_TOOLS_LIB}.${VERSION}

  COMMAND ln -sf ${GLOBAL_CORE_LIB}.${VERSION} ${GLOBAL_CORE_LIB}
  COMMAND ln -sf ${GLOBAL_TOOLS_LIB}.${VERSION} ${GLOBAL_TOOLS_LIB}
)

message(STATUS "Copying libs to ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}")


catkin_package(
  DEPENDS
  CATKIN_DEPENDS
  INCLUDE_DIRS include
  LIBRARIES
  OpenMeshCore
  OpenMeshTools
)

include_directories(${catkin_INCLUDE_DIRS})
add_library(OpenMeshCore SHARED)
add_library(OpenMeshTools SHARED)
set_target_properties(OpenMeshCore PROPERTIES LINKER_LANGUAGE CXX)
set_target_properties(OpenMeshTools PROPERTIES LINKER_LANGUAGE CXX)

add_dependencies(OpenMeshCore copy_openmesh)
add_dependencies(OpenMeshTools copy_openmesh)
add_dependencies(copy_openmesh build_openmesh)

#install(TARGETS zxing
#	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#)

#install(DIRECTORY common/include/
#	DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
#)