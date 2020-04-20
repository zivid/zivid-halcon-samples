cmake_minimum_required(VERSION 3.3.2 FATAL_ERROR)

project(
    C++Halcon
    CXX
)

SET(CMAKE_BUILD_TYPE Release)

set(
    CMAKE_CXX_STANDARD 
    17
)

set(SAMPLES
	HalconConnectUsingGenICam/HalconConnectUsingGenICam
	HalconConnectUsingZivid/HalconConnectUsingZivid)

# Look for Find*.cmake files in root directory
set(
    CMAKE_MODULE_PATH 
    ${CMAKE_MODULE_PATH} 
    ${CMAKE_CURRENT_SOURCE_DIR}
)

find_package(
    Zivid
    1.8.1
    REQUIRED
    COMPONENTS Core
)

if(DEFINED ENV{HALCONROOT} AND NOT $ENV{HALCONROOT} STREQUAL "")
  message("Using HALCONROOT " $ENV{HALCONROOT})
  STRING(REGEX REPLACE "\\\\" "/" HALCONROOT $ENV{HALCONROOT}) 
endif()
find_package(Halcon)
find_package(HDevEngine)


foreach(SAMPLE ${SAMPLES})
	get_filename_component(SAMPLE_NAME ${SAMPLE} NAME)
	
	link_directories(${CMAKE_CURRENT_SOURCE_DIR})
		
	add_executable(
		${SAMPLE_NAME} 
		${SAMPLE_NAME}/${SAMPLE_NAME}.cpp)

	target_include_directories(
		${SAMPLE_NAME}
		PUBLIC 
		${HALCON_INC_DIRS}
		${HDEVENGINE_INC_DIRS}
		include
	)

	target_link_libraries(
		${SAMPLE_NAME}
		Zivid::Core
		Zivid::Vis3D
		${HALCON_LIBS}
		${HDEVENGINE_LIBS}
	)
endforeach()