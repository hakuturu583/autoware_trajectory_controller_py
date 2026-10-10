# Third party C++ dependencies. Everything is fetched at configure time so that a plain
# `uv sync` / `pip install` works without system packages (only a C++17 compiler is needed).
include(FetchContent)
set(FETCHCONTENT_QUIET ON)

# ---- Eigen (header only) ---------------------------------------------------
find_package(Eigen3 3.3 QUIET NO_MODULE)
if(NOT TARGET Eigen3::Eigen)
  FetchContent_Declare(
    eigen
    URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
    URL_HASH SHA256=8586084f71f9bde545ee7fa6d00288b264a2b7ac3607b974e54d13e7162c1c72
    # Only the headers are needed: point SOURCE_SUBDIR to a directory without
    # CMakeLists.txt so that Eigen's own (heavy) CMake project is not configured.
    SOURCE_SUBDIR do-not-configure
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  )
  FetchContent_MakeAvailable(eigen)
  add_library(Eigen3::Eigen INTERFACE IMPORTED GLOBAL)
  set_target_properties(Eigen3::Eigen PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${eigen_SOURCE_DIR}")
endif()

# ---- OSQP 0.6.x (same major version as Autoware's osqp_vendor) ----------------
set(OSQP_BUILD_DEMO_EXE OFF CACHE BOOL "" FORCE)
set(UNITTESTS OFF CACHE BOOL "" FORCE)
set(PRINTING ON CACHE BOOL "" FORCE)
set(PROFILING ON CACHE BOOL "" FORCE)
set(CTRLC OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
  osqp
  GIT_REPOSITORY https://github.com/osqp/osqp.git
  GIT_TAG v0.6.3
  GIT_SHALLOW TRUE
  GIT_SUBMODULES lin_sys/direct/qdldl/qdldl_sources
)
FetchContent_MakeAvailable(osqp)
# Autoware includes "osqp/osqp.h": expose the headers under an `osqp/` prefix.
set(OSQP_PREFIXED_INCLUDE_DIR "${CMAKE_BINARY_DIR}/osqp_include")
file(GLOB _osqp_headers "${osqp_SOURCE_DIR}/include/*.h")
file(COPY ${_osqp_headers} DESTINATION "${OSQP_PREFIXED_INCLUDE_DIR}/osqp")
set_target_properties(osqp PROPERTIES EXCLUDE_FROM_ALL TRUE)
