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
set(CMAKE_C_FLAGS_DEBUG "-Og -g")
set(CMAKE_ASM_FLAGS_DEBUG "-Og -g")
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
add_executable(${PROJECT_NAME} main.cpp head_control.cpp)

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
# OpenCV
find_package(OpenCV REQUIRED)

message(STATUS "OpenCV library status:")
message(STATUS "    config: ${OpenCV_DIR}")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC ${OpenCV_LIBS}
)

# ---------------------------------------------------------------------------------------
# Eigen
add_subdirectory(third_part/eigen)

target_include_directories(${PROJECT_NAME} 
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/third_part/eigen
)

# add_compile_definitions(EIGEN_NO_MALLOC)

# ---------------------------------------------------------------------------------------
# SPDLOG
find_package(spdlog REQUIRED)

target_link_libraries(${PROJECT_NAME} PUBLIC spdlog::spdlog)
