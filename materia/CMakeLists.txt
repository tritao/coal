# Materia's build of coal: the collision and distance core as a static
# library, with no Boost, no assimp and none of coal's own CMake machinery
# (jrl-cmakemodules). Left out: mesh-file loading (assimp, Boost.Filesystem),
# serialization (Boost.Serialization), octrees (octomap), logging
# (Boost.Log) and the Python bindings. Meshes are built from triangle arrays
# by the caller. This branch's only source change drops Boost from the core
# headers (pi from Eigen, an unused boost/function include removed).
#
# Use: add_subdirectory(<coal>/materia) and link coal::core (needs Eigen3).
cmake_minimum_required(VERSION 3.21)

set(COAL_ROOT "${CMAKE_CURRENT_LIST_DIR}/..")
file(GLOB_RECURSE COAL_CORE_SOURCES CONFIGURE_DEPENDS "${COAL_ROOT}/src/*.cpp")
list(FILTER COAL_CORE_SOURCES EXCLUDE REGEX "/src/(mesh_loader|serialization)/|/src/octree\\.cpp$")

if(NOT TARGET Eigen3::Eigen)
    find_package(Eigen3 3.3 REQUIRED NO_MODULE)
endif()

add_library(coal_core STATIC ${COAL_CORE_SOURCES})
target_compile_features(coal_core PUBLIC cxx_std_17)
# The generated headers (config, deprecated, warning) come first.
target_include_directories(coal_core SYSTEM PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}/include"
    "${COAL_ROOT}/include")
target_link_libraries(coal_core PUBLIC Eigen3::Eigen)
set_target_properties(coal_core PROPERTIES
    POSITION_INDEPENDENT_CODE ON
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN ON)
if(NOT MSVC)
    target_compile_options(coal_core PRIVATE -w)
endif()
add_library(coal::core ALIAS coal_core)
