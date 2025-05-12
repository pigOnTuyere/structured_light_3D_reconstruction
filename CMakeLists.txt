cmake_minimum_required (VERSION 3.20)

# 如果支持，请为 MSVC 编译器启用热重载。
if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

project ("struct-light")

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# project setting
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

# 可执行文件输出位置
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/bin)
set(CMAKE_CXX_STANDARD 17)

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# dependencies
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

# PCL
#  set(PCL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/deps/PCL/PCL 1.12.1/cmake")
#  set(PCL_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/deps/PCL/PCL 1.12.1/")

#set(PCL_ROOT "D:/PCL/PCL 1.12.1")
# set(PCL_DIR "${PCL_ROOT}/cmake")
# set(PCL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/deps/PCL/PCL 1.12.1/cmake")

find_package(PCL  REQUIRED)
if( PCL_FOUND )
  include_directories( ${PCL_INCLUDE_DIRS} )
  add_definitions( ${PCL_DEFINITIONS} )
  link_directories( ${PCL_LIBRARY_DIRS} )
  link_libraries( ${PCL_LIBRARIES} )
endif()

# OpenCV
 set( OpenCV_DIR "D:/cpp_library/opencv-4.5.5/opencv/build")

# set( OpenCV_DIR ${CMAKE_CURRENT_SOURCE_DIR}/OpenCV/opencv481/build )
find_package( OpenCV REQUIRED )
if( OpenCV_FOUND )
  include_directories( ${OpenCV_INCLUDE_DIRS} )
  link_directories( ${OpenCV_LIB_DIR} )
  link_libraries(  ${OpenCV_LIBS} )
endif()


# src files
file(GLOB_RECURSE runtime_head_files "src/*.h")
file(GLOB_RECURSE runtime_source_files "src/*.cpp")

add_executable(${PROJECT_NAME} ${runtime_head_files} ${runtime_source_files} "src/Phaser.cpp" "src/Phaser.h" "src/Config.h")

target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  )