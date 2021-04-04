project(bfm_manager) # Project name: Base Face Model Manager
cmake_minimum_required(VERSION 2.8.12)

add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 17)
if(UNIX)
   add_compile_options(-fPIC)
endif()

add_library(bfm_manager SHARED src/bfm_manager.cpp)
target_compile_features(bfm_manager PRIVATE cxx_std_17)

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
target_include_directories(bfm_manager PUBLIC ${OpenCV_INCLUDE_DIRS})

find_package(Glog REQUIRED)
include_directories(${GLOG_INCLUDE_DIRS})

find_package(Boost REQUIRED system filesystem program_options)
include_directories(${Boost_INCLUDE_DIRS})

if(UNIX)
    set(LIBS 
        /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.65.1
        /usr/lib/x86_64-linux-gnu/libboost_system.so.1.65.1
    )
endif()

# Bfm Manager
target_include_directories(bfm_manager PUBLIC ./include)
target_link_libraries(bfm_manager m ${OpenCV_LIBS} ${HDF5_LIBRARIES} ${HDF5_CXX_LIBRARIES} ${Boost_LIBRARIES} glog boost_filesystem)


###############################################################################################
######################################## Options ##############################################
###############################################################################################


OPTION(GEN_BFM_EXAMPLES "Generate usage examples of Basel Face Moldel manager." ON)
if(GEN_BFM_EXAMPLES)
   add_executable(gen_faces example/ex_gen_faces.cpp)
   target_link_libraries(gen_faces bfm_manager)
endif()