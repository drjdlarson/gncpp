target_sources(gncpy 
    PRIVATE 
        Exceptions.cpp
)

add_subdirectory(dynamics)
add_subdirectory(filters)
add_subdirectory(math)
add_subdirectory(measurements)
add_subdirectory(control)