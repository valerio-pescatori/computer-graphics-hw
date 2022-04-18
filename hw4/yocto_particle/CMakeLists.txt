add_library(yocto_particle yocto_particle.h yocto_particle.cpp)

set_target_properties(yocto_particle PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES)

target_include_directories(yocto_particle PUBLIC ${CMAKE_SOURCE_DIR}/libs)
target_link_libraries(yocto_particle yocto)
