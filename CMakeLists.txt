project(bfm_manager) # Project name: Base Face Model Manager
cmake_minimum_required(VERSION 2.8.12)

add_compile_options(-std=c++11)
add_compile_options(-fPIC)


###############################################################################################
######################################## Dedendency ###########################################
###############################################################################################

# Eigen3
if(UNIX)
   include_directories("/usr/include/eigen3")
endif()

# Hdf5
find_package(HDF5 REQUIRED C CXX)
include_directories(${HDF5_INCLUDE_DIRS})

# OpenCV
set(OpenCV_DIR "/usr/local/opencv3/share/OpenCV")   
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Bfm Manager
include_directories(./include)
add_library(bfm_manager SHARED src/bfm_manager.cpp)
target_link_libraries(bfm_manager m ${OpenCV_LIBS} ${HDF5_LIBRARIES} ${HDF5_CXX_LIBRARIES})


###############################################################################################
######################################## Options ##############################################
###############################################################################################


OPTION(GEN_BFM_EXAMPLES "Generate usage examples of Basel Face Moldel manager." ON)
if(GEN_BFM_EXAMPLES)
   add_executable(gen_faces example/ex_gen_faces.cpp)
   target_link_libraries(gen_faces bfm_manager)
endif()