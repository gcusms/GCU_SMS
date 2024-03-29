# Create project
cmake_minimum_required(VERSION 3.0.0)
project(WolfVision VERSION 2021.7.9 LANGUAGES CXX)

# Check system platform
if(NOT CMAKE_SYSTEM_NAME STREQUAL Linux)
    message(FATAL_ERROR "Not supported platform: ${CMAKE_SYSTEM_NAME}")
endif()

# Specialize CMake build flags for different build types
set(CMAKE_CXX_FLAGS "-Wall -Wextra -Wno-deprecated")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# Use standard c++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Add DEBUG/RELEASE marco to define if debug/release configured
add_compile_options("$<$<CONFIG:DEBUG>:-DDEBUG>")
add_compile_options("$<$<CONFIG:RELEASE>:-DRELEASE>")

# Don't build test by default
set(BUILD_TEST False)

# Prefer posix threads
set(THREADS_PREFER_PTHREAD_FLAG True)
set(CMAKE_THREAD_LIBS_INIT "-lpthread")
find_package(Threads REQUIRED)

# Find and add opencv as required
find_package(OpenCV REQUIRED)


include_directories(${OpenCV_INCLUDE_DIRS})
link_libraries(${OpenCV_LIBS})

# Add fmtlib directory
add_subdirectory(3rdparty/fmt)
# Add fmtlib include to EXTRA_INCLUDES
list(APPEND EXTRA_INCLUDES 3rdparty/fmt/include)

# Add mindvision-sdk directory and include path for target
add_library(MVSDK SHARED IMPORTED)
get_filename_component(MVSDK_PATH 3rdparty/mindvision/linux REALPATH)
# Check system architecture for mindvision-sdk library
execute_process(COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE)
message(STATUS "Checking platform architecture for MVSDK: ${ARCHITECTURE}")
if(${ARCHITECTURE} STREQUAL x86_64)
    set(MIMDVISION_SDK_LIBRARY ${MVSDK_PATH}/lib/x64/libMVSDK.so)
elseif(${ARCHITECTURE} STREQUAL aarch64)
    set(MIMDVISION_SDK_LIBRARY ${MVSDK_PATH}/lib/arm64/libMVSDK.so)
else()
    message(FATAL_ERROR "Not supported architecture for MVSDK: ${ARCHITECTURE}")
endif()
set_property(TARGET MVSDK PROPERTY IMPORTED_LOCATION ${MIMDVISION_SDK_LIBRARY})
# Add mindvision lib to build target
list(APPEND EXTRA_LIBS MVSDK)
# Add mindvision include path
list(APPEND EXTRA_INCLUDES 3rdparty/mindvision/linux/include)

# Add project source dir to include
list(APPEND EXTRA_INCLUDES ${PROJECT_SOURCE_DIR})

# Add base src and header
list(APPEND EXTRA_INCLUDES base)

# Add device as shared library to build target
list(APPEND EXTRA_INCLUDES devices)

list(APPEND EXTRA_INCLUDES ${PROJECT_SOURCE_DIR}/devices/camera)
add_library(wv-mv-video-capture SHARED devices/camera/mv_video_capture.cpp)
target_link_libraries(wv-mv-video-capture
    PRIVATE
    MVSDK
)
list(APPEND WV_EXTRA_LIBS wv-mv-video-capture)

list(APPEND EXTRA_INCLUDES ${PROJECT_SOURCE_DIR}/devices/serial)
add_library(wv-uart-serial SHARED devices/serial/uart_serial.cpp)
list(APPEND WV_EXTRA_LIBS wv-uart-serial)

# Add module as shared library to build target
list(APPEND EXTRA_INCLUDES module)

list(APPEND EXTRA_INCLUDES ${PROJECT_SOURCE_DIR}/module/angle_solve)
add_library(wv-basic-pnp SHARED module/angle_solve/basic_pnp.cpp)
list(APPEND WV_EXTRA_LIBS wv-basic-pnp)

list(APPEND EXTRA_INCLUDES ${PROJECT_SOURCE_DIR}/module/armor)
add_library(wv-basic-armor SHARED module/armor/basic_armor.cpp)
list(APPEND WV_EXTRA_LIBS wv-basic-armor)

# Add utils header
list(APPEND EXTRA_INCLUDES utils)

# Add EXTRA_INCLUDES to project includes
include_directories(${EXTRA_INCLUDES})

# Add base/wolfvision to executable
cmake_policy(SET CMP0069 NEW)
add_executable(WolfVision base/wolfvision.cpp)

# Add shared library link to WolfVision
target_link_libraries(WolfVision
    PUBLIC
    Threads::Threads
    fmt::fmt
    ${OpenCV_LIBS}
    ${EXTRA_LIBS}

    PRIVATE
    ${WV_EXTRA_LIBS}
)

# Set WolfVision compile cxx_std features
target_compile_features(WolfVision PUBLIC cxx_std_${CMAKE_CXX_STANDARD})

# Set WolfVision linking time optimization to true
set_target_properties(WolfVision PROPERTIES INTERPROCEDURAL_OPTIMIZATION True)

# Set built binary to ~/bin
set(EXECUTABLE_OUTPUT_PATH "${PROJECT_BINARY_DIR}/bin")

# Set built library to ~/lib
set(LIBRARY_OUTPUT_PATH "${PROJECT_BINARY_DIR}/lib")

# Set configs folder absolute path
set(CONFIG_FILE_PATH ${PROJECT_SOURCE_DIR}/configs)
set(SOURCE_PATH ${PROJECT_SOURCE_DIR})
target_compile_definitions(WolfVision PRIVATE "CONFIG_FILE_PATH=\"${CONFIG_FILE_PATH}\"")
target_compile_definitions(WolfVision PRIVATE "SOURCE_PATH=\"${SOURCE_PATH}\"")

# Add test_camera directory
if(BUILD_TEST)
    add_subdirectory(test/test_camera)
endif()
