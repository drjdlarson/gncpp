find_package(Doxygen)
if(DOXYGEN_FOUND)
    set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in)
    set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

    # request to configure the file
    configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
    message(STATUS "Configuring Doxygen")

    # note the option ALL which allows to build the docs together with the application
    add_custom_target(docs_doxygen ALL
                        COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
                        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
                        COMMENT "Generating API documentation with Doxygen"
                        VERBATIM
    )

    find_package (Python COMPONENTS Interpreter)
    if(Python_Interpreter_FOUND)
        # TODO: use python interpreter to check if sphinx is installed

        message(STATUS "Configuring Sphinx")
        file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/source DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
        file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/../LICENSE DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/source)

        add_custom_target(docs ALL
                            DEPENDS docs_doxygen
                            COMMAND sphinx-build -b html -Dversion=${GNCPY_VERSION} -Drelease=${GNCPY_VERSION} -Aversion=${GNCPY_VERSION} -Arelease=${GNCPY_VERSION} ${CMAKE_CURRENT_BINARY_DIR}/source ${CMAKE_CURRENT_BINARY_DIR}/html
                            COMMENT "Generating documentation with Sphinx"
                            VERBATIM
        )

    else()
        message("Failed to find python, this must be available to generate the documentation")
    endif()

else()
    message("Doxygen needs to be installed to generate the doxygen documentation")
endif()