project(bfm_manager) # Project name: Base Face Model Manager
cmake_minimum_required(VERSION 2.8.12)


add_compile_options(-fPIC)


###############################################################################################
######################################## Dedendency ###########################################
###############################################################################################


# Hdf5
find_package(HDF5 REQUIRED C CXX)


# OpenCV
find_package(OpenCV QUIET)
if(OpenCV_FOUND)
   message("OpenCV found.")
   include_directories(${OpenCV_INCLUDE_DIRS})
else()
   message("OpenCV not found, so we won't build the landmark-fitting-cam example.")
endif()


# Bfm Manager
include_directories(./include)
add_library(bfm_manager SHARED src/bfm_manager.cpp)
target_link_libraries(bfm_manager m ${OpenCV_LIBS} ${HDF5_LIBRARIES} ${HDF5_CXX_LIBRARIES})


###############################################################################################
######################################## Options ##############################################
###############################################################################################


OPTION(GEN_BFM_EXAMPLES "Generate usage examples of Basel Face Moldel manager." OFF)
if(GEN_BFM_EXAMPLES)
   add_executable(write_avg_face example/write_avg_face.cpp)
   target_link_libraries(write_avg_face bfm_manager)
endif()