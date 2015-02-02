cmake_minimum_required(VERSION 2.8)
project(BallKalman)


find_package(OpenCV REQUIRED )
find_package(Boost COMPONENTS system filesystem REQUIRED)
find_package(CUDA REQUIRED )

INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )

#SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -stdlib=libstdc++" )

set( NAME_SRC
    src/BallKalman.cpp    
)

CUDA_add_executable(BallKalman src/BallKalman.cpp)
target_link_libraries(BallKalman ${OpenCV_LIBS})
target_link_libraries(BallKalman ${Boost_LIBRARIES}  ${Boost_FILESYSTEM_LIBRARY}
  ${Boost_SYSTEM_LIBRARY} )
