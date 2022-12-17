project(DOGM_PY LANGUAGES CXX CUDA)
cmake_minimum_required(VERSION 3.6)
project( dogm_py LANGUAGES CXX CUDA )

find_package(CUDA REQUIRED)
find_package(PythonInterp 3.6 REQUIRED)
find_package(PythonLibs 3.6 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/thirdParty/dynamic-occupancy-grid-map/dogm)

include_directories(
    ${PYTHON_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/pybind11/include
    SYSTEM
    ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${DOGM_INCLUDE_DIRS}
    )

link_directories(
    /usr/local/cuda/lib64
    )

set(CMAKE_CXX_STANDARD 14)
set(OpenGL_GL_PREFERENCE "GLVND")
set( CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS}" "--shared --compiler-options '-fPIC'" )

string(APPEND CMAKE_CUDA_FLAGS " --expt-extended-lambda")
set(CUDA_PROPAGATE_HOST_FLAGS ON)

add_library(${PROJECT_NAME} SHARED
        src/dogm_py.cpp
    )

target_link_libraries(dogm_py
    ${PYTHON_LIBRARIES}
    ${OPENGL_LIBRARY}
    ${GLFW3_LIBRARY}
    ${GLEW_LIBRARIES}
    ${OpenCV_LIBS}
    cudart
    dogm
)

set_target_properties(dogm_py PROPERTIES PREFIX "")
