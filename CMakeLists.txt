cmake_minimum_required(VERSION 3.12)
project(hfsm VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Opciones
option(HFSM_BUILD_TESTS "Build tests" ON)
option(HFSM_BUILD_EXAMPLES "Build examples" ON)

# Librería
add_library(hfsm STATIC
    src/hfsm_core.cpp
    src/hfsm_config_manager.cpp
    src/hfsm_logging.cpp
    src/hfsm_event_types.cpp
)

target_include_directories(hfsm PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_options(hfsm PRIVATE
    -Wall
    -Wextra
    -Wno-sign-compare
    -Wno-unused-parameter
    -pthread
)

target_link_libraries(hfsm PUBLIC pthread)

# Tests
if(HFSM_BUILD_TESTS)
    add_executable(test_minimal tests/test_minimal.cpp)
    target_link_libraries(test_minimal hfsm)
    
    add_executable(test_comprehensive tests/test_comprehensive.cpp)
    target_link_libraries(test_comprehensive hfsm)
endif()

# Examples
if(HFSM_BUILD_EXAMPLES)
    add_executable(example_basic examples/basic_usage.cpp)
    target_link_libraries(example_basic hfsm)
endif()

# Instalación
install(TARGETS hfsm
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/hfsm
    DESTINATION include
)
