cmake_minimum_required(VERSION 3.11)

project(xrobot LANGUAGES C CXX ASM)

# ---------------------------------------------------------------------------------------
# Default Options
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/build)

# ---------------------------------------------------------------------------------------
# Debug
set(CMAKE_C_FLAGS_DEBUG "-g")
set(CMAKE_ASM_FLAGS_DEBUG "-g")
add_compile_options(-Werror -gdwarf-4)

# ---------------------------------------------------------------------------------------
# Flags
add_compile_options(-Wall -Wextra -fno-builtin -ffunction-sections -fdata-sections)
link_libraries(pthread)

# ---------------------------------------------------------------------------------------
# PATH
set(WEBOTS_HOME /usr/local/webots)

# ---------------------------------------------------------------------------------------
# Sources
add_executable(${PROJECT_NAME} main.cpp)

# ---------------------------------------------------------------------------------------
# Webots
target_link_directories(
  ${PROJECT_NAME}
  PUBLIC ${WEBOTS_HOME}/lib/controller
)

target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC ${LIBRARIES}
  PUBLIC ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX}
  PUBLIC ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX}
)

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
  PUBLIC ${WEBOTS_HOME}/include/controller/c
  PUBLIC ${WEBOTS_HOME}/include/controller/cpp
  PUBLIC ${WEBOTS_HOME}/lib/controller
)

# ---------------------------------------------------------------------------------------
# Magic Enum
add_subdirectory(third_part/magic_enum)

target_include_directories(${PROJECT_NAME} 
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/third_part/magic_enum/include/magic_enum
)

# ---------------------------------------------------------------------------------------
# libxr
set(LIBXR_SYSTEM Webots)

add_compile_definitions(LIBXR_DEFAULT_SCALAR=double)

add_subdirectory(third_part/libxr)

target_link_libraries(${PROJECT_NAME} PUBLIC xr)

# ---------------------------------------------------------------------------------------
# SDL GUI
find_package(SDL2 REQUIRED)
target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC SDL2::SDL2
  PUBLIC SDL2::SDL2main
  PUBLIC SDL2_image
  PUBLIC SDL2_ttf)
