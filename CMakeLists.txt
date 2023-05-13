cmake_minimum_required(VERSION 3.15)

# Determine if gncpy is built as a subproject (using add_subdirectory)
# or if it is the master project.
if (NOT DEFINED GNCPY_MASTER_PROJECT)
  set(GNCPY_MASTER_PROJECT OFF)
  if (CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
    set(GNCPY_MASTER_PROJECT ON)
    set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
    message(STATUS "CMake version: ${CMAKE_VERSION}")
  endif ()
endif ()

# Joins arguments and places the results in ${result_var}.
function(join result_var)
  set(result "")
  foreach (arg ${ARGN})
    set(result "${result}${arg}")
  endforeach ()
  set(${result_var} "${result}" PARENT_SCOPE)
endfunction()

include(CMakeParseArguments)

# Sets a cache variable with a docstring joined from multiple arguments:
#   set(<variable> <value>... CACHE <type> <docstring>...)
# This allows splitting a long docstring for readability.
function(set_verbose)
  # cmake_parse_arguments is broken in CMake 3.4 (cannot parse CACHE) so use
  # list instead.
  list(GET ARGN 0 var)
  list(REMOVE_AT ARGN 0)
  list(GET ARGN 0 val)
  list(REMOVE_AT ARGN 0)
  list(REMOVE_AT ARGN 0)
  list(GET ARGN 0 type)
  list(REMOVE_AT ARGN 0)
  join(doc ${ARGN})
  set(${var} ${val} CACHE ${type} ${doc})
endfunction()

# Set the default CMAKE_BUILD_TYPE to Release.
# This should be done before the project command since the latter can set
# CMAKE_BUILD_TYPE itself (it does so for nmake).
if (GNCPY_MASTER_PROJECT AND NOT CMAKE_BUILD_TYPE)
    set_verbose(CMAKE_BUILD_TYPE Release CACHE STRING
                "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or "
                "CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel.")
endif ()

project(gncpy CXX)
include(GNUInstallDirs)

#---------------------------------------------------------------------------
# compile settings
#---------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 20)
set(CXX_STANDARD_REQUIRED YES)

set_verbose(GNCPY_INC_DIR ${CMAKE_INSTALL_INCLUDEDIR} CACHE STRING
            "Installation directory for include files, a relative path that "
            "will be joined with ${CMAKE_INSTALL_PREFIX} or an absolute path."
)

# Options that control generation of various targets.
option(GNCPY_DOC "Generate the doc target." ${GNCPY_MASTER_PROJECT})
option(GNCPY_INSTALL "Generate the install target." ON)
option(GNCPY_TEST "Generate the test target." ${GNCPY_MASTER_PROJECT})

if(CMAKE_BUILD_TYPE MATCHES "^[Cc]overage")
    set(GNCPY_TEST ON)
endif()

# Get version from core.h
file(READ include/gncpy/core.h core_h)
if (NOT core_h MATCHES "GNCPY_VERSION ([0-9]+)([0-9][0-9])([0-9][0-9])")
    message(FATAL_ERROR "Cannot get GNCPY_VERSION from core.h.")
endif ()
# Use math to skip leading zeros if any.
math(EXPR CPACK_PACKAGE_VERSION_MAJOR ${CMAKE_MATCH_1})
math(EXPR CPACK_PACKAGE_VERSION_MINOR ${CMAKE_MATCH_2})
math(EXPR CPACK_PACKAGE_VERSION_PATCH ${CMAKE_MATCH_3})
join(GNCPY_VERSION ${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.
                 ${CPACK_PACKAGE_VERSION_PATCH})
message(STATUS "GNCPY_CPP Version: ${GNCPY_VERSION}")

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

if (NOT CMAKE_RUNTIME_OUTPUT_DIRECTORY)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin)
endif ()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}
    "${CMAKE_CURRENT_SOURCE_DIR}/support/cmake"
)

include(CheckCXXCompilerFlag)
include(JoinPaths)

if (GNCPY_MASTER_PROJECT AND NOT DEFINED CMAKE_CXX_VISIBILITY_PRESET)
    set_verbose(CMAKE_CXX_VISIBILITY_PRESET hidden CACHE STRING
                "Preset for the export of private symbols"
    )
    set_property(CACHE CMAKE_CXX_VISIBILITY_PRESET PROPERTY STRINGS
                hidden default
    )
endif ()

if (GNCPY_MASTER_PROJECT AND NOT DEFINED CMAKE_VISIBILITY_INLINES_HIDDEN)
    set_verbose(CMAKE_VISIBILITY_INLINES_HIDDEN ON CACHE BOOL
                "Whether to add a compile flag to hide symbols of inline functions"
    )
endif ()

# ------------------------------------------------------------------
# ---------------------- Find dependencies -------------------------
# ------------------------------------------------------------------
message(STATUS "Fetching cereal dependency...")
include(FetchContent)
FetchContent_Declare(
    cereal
    GIT_REPOSITORY https://github.com/USCiLab/cereal
    GIT_TAG v1.3.2
)
set(JUST_INSTALL_CEREAL ON CACHE INTERNAL "only install cereal")
set(CEREAL_INSTALL ON CACHE INTERNAL "install cereal")
set(THREAD_SAFE ON CACHE INTERNAL "allow multiple cereal archives to be accessed in parallel")
FetchContent_MakeAvailable(cereal)

message(STATUS "Fetching Eigen3 dependency...")
FetchContent_Declare(
    eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG 3.4.0
)
set(BUILD_TESTING OFF CACHE INTERNAL "Disable Eigen tests")
FetchContent_MakeAvailable(eigen)


# ------------------------------------------------------------------
# --------------------- Check status of options --------------------
# ------------------------------------------------------------------
if (GNCPY_DOC)
    message(STATUS "Adding documentation...")
    add_subdirectory(docs)
endif ()

if (GNCPY_TEST)
    message(STATUS "Enabling testing...")
    enable_testing()
    add_subdirectory(test)
endif ()

set(gitignore ${PROJECT_SOURCE_DIR}/.gitignore)
if (GNCPY_MASTER_PROJECT AND EXISTS ${gitignore})
    # Get the list of ignored files from .gitignore.
    file (STRINGS ${gitignore} lines)
    list(REMOVE_ITEM lines /doc/html)
    foreach (line ${lines})
        string(REPLACE "." "[.]" line "${line}")
        string(REPLACE "*" ".*" line "${line}")
        set(ignored_files ${ignored_files} "${line}$" "${line}/")
    endforeach ()
    set(ignored_files ${ignored_files}
        /.git /breathe /format-benchmark sphinx/ .buildinfo .doctrees
    )

    set(CPACK_SOURCE_GENERATOR ZIP)
    set(CPACK_SOURCE_IGNORE_FILES ${ignored_files})
    set(CPACK_SOURCE_PACKAGE_FILE_NAME gncpy-${GNCPY_VERSION})
    set(CPACK_PACKAGE_NAME gncpy)
    set(CPACK_RESOURCE_FILE_README ${PROJECT_SOURCE_DIR}/README.md)
    include(CPack)
endif ()


# ------------------------------------------------------------------
# ----------------------- Add GNCPy Library ------------------------
# ------------------------------------------------------------------
add_library(gncpy ${GNCPY_SOURCES})
add_library(lager::gncpy ALIAS gncpy)
add_subdirectory(src)


target_link_libraries(gncpy cereal::cereal Eigen3::Eigen)
target_include_directories(gncpy PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${GNCPY_INC_DIR}>
)

if(CMAKE_BUILD_TYPE MATCHES "^[Cc]overage")
    set(CODE_COVERAGE_VERBOSE ON)
    include(CodeCoverage)
    append_coverage_compiler_flags_to_target(gncpy)
    setup_target_for_coverage_gcovr_xml(
        NAME ctest_coverage_xml
        EXECUTABLE ctest -j3
        DEPENDENCIES
        # dynamics dependencies
            double_int_dyn_test
            curvilinear_dyn_test
        # filter dependencies
            kalman_test
        # math dependencies
            matrix_test
            matrix_test
            vector_test
            math_test
        # measurement dependencies
            measurement_test
        EXCLUDE build/* test/*
    )
    setup_target_for_coverage_gcovr_html(
        NAME ctest_coverage_html
        EXECUTABLE ctest -j3
        DEPENDENCIES 
        # dynamics dependencies
            double_int_dyn_test
            curvilinear_dyn_test
        # filter dependencies
            kalman_test
        # math dependencies
            matrix_test
            matrix_test
            vector_test
            math_test
        # measurement dependencies
            measurement_test
        EXCLUDE build/* test/*
    )
elseif(CMAKE_BUILD_TYPE MATCHES "^[Dd]ebug")
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
        message(STATUS "Using GNU compiler, adding full set of compiler flags")
        target_compile_options(gncpy PRIVATE
            "-fPIC"
            "-Wall"
            "-Wextra"
            "-Wpedantic"
            "-gdwarf-4"
            "-gstatement-frontiers"
            "-gvariable-location-views"
            "-ginline-points"
            "-fno-eliminate-unused-debug-symbols"
            "-fvar-tracking"
            "-fvar-tracking-assignments"
        )
    else()
    message(STATUS "Not using GNU compiler, adding reduced set of compiler flags")
        target_compile_options(gncpy PRIVATE
        "-fPIC"
        "-Wall"
        "-Wextra"
        "-Wpedantic"
        "-gdwarf-4"
        "-fno-eliminate-unused-debug-symbols"
        "-Og"
    )
    endif()
else()
    target_compile_options(gncpy PRIVATE
        "-fPIC"
        "-Wall"
        "-Wextra"
        "-Wpedantic"
        "-O3"
    )
endif()

set_target_properties(gncpy 
    PROPERTIES
        VERSION ${GNCPY_VERSION} SOVERSION ${CPACK_PACKAGE_VERSION_MAJOR}
)

# Install targets.
if (GNCPY_INSTALL)
    include(CMakePackageConfigHelpers)
    set_verbose(GNCPY_CMAKE_DIR ${CMAKE_INSTALL_LIBDIR}/cmake/gncpy CACHE STRING
                "Installation directory for cmake files, a relative path that "
                "will be joined with ${CMAKE_INSTALL_PREFIX} or an absolute "
                "path.")
    set(version_config ${PROJECT_BINARY_DIR}/gncpy-config-version.cmake)
    set(project_config ${PROJECT_BINARY_DIR}/gncpy-config.cmake)
    set(pkgconfig ${PROJECT_BINARY_DIR}/gncpy.pc)
    set(targets_export_name gncpy-targets)

    set_verbose(GNCPY_LIB_DIR ${CMAKE_INSTALL_LIBDIR} CACHE STRING
                "Installation directory for libraries, a relative path that "
                "will be joined to ${CMAKE_INSTALL_PREFIX} or an absolute path.")

    set_verbose(GNCPY_PKGCONFIG_DIR ${CMAKE_INSTALL_LIBDIR}/pkgconfig CACHE PATH
                "Installation directory for pkgconfig (.pc) files, a relative "
                "path that will be joined with ${CMAKE_INSTALL_PREFIX} or an "
                "absolute path.")

    # Generate the version, config and target files into the build directory.
    write_basic_package_version_file(
        ${version_config}
        VERSION ${GNCPY_VERSION}
        COMPATIBILITY SameMajorVersion
    )

    join_paths(libdir_for_pc_file "\${exec_prefix}" "${GNCPY_LIB_DIR}")
    join_paths(includedir_for_pc_file "\${prefix}" "${GNCPY_INC_DIR}")

    configure_file(
        "${PROJECT_SOURCE_DIR}/support/cmake/gncpy.pc.in"
        "${pkgconfig}"
        @ONLY
    )
    configure_package_config_file(
        ${PROJECT_SOURCE_DIR}/support/cmake/gncpy-config.cmake.in
        ${project_config}
        INSTALL_DESTINATION ${GNCPY_CMAKE_DIR}
    )

    # see https://stackoverflow.com/questions/5378528/install-export-problem-for-library-with-dependencies
    # for why cereal is included here
    set(INSTALL_TARGETS gncpy cereal eigen)

    # Install the library and headers.
    install(TARGETS ${INSTALL_TARGETS} EXPORT ${targets_export_name}
            LIBRARY DESTINATION ${GNCPY_LIB_DIR}
            ARCHIVE DESTINATION ${GNCPY_LIB_DIR}
            FRAMEWORK DESTINATION "."
            RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )
    # PUBLIC_HEADER does not preserve directory structure so instead use this. see https://gitlab.kitware.com/cmake/cmake/-/issues/23274
    install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include/"
        DESTINATION ${GNCPY_INC_DIR}
        FILES_MATCHING
        PATTERN "*.h*"
    )


    # Use a namespace because CMake provides better diagnostics for namespaced
    # imported targets.
    export(TARGETS ${INSTALL_TARGETS} NAMESPACE lager::
            FILE ${PROJECT_BINARY_DIR}/${targets_export_name}.cmake)

    # Install version, config and target files.
    install(
        FILES ${project_config} ${version_config}
        DESTINATION ${GNCPY_CMAKE_DIR}
    )
    install(
        EXPORT ${targets_export_name}
        DESTINATION ${GNCPY_CMAKE_DIR}
        NAMESPACE lager::
    )

    if (MSVC)
        install(
            FILES $<TARGET_PDB_FILE:${INSTALL_TARGETS}>
            DESTINATION ${GNCPY_LIB_DIR} OPTIONAL
        )
    endif()
    install(FILES "${pkgconfig}" DESTINATION "${GNCPY_PKGCONFIG_DIR}")
endif ()
