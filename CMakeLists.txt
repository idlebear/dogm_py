project(DOGM_PY LANGUAGES CXX CUDA)
cmake_minimum_required(VERSION 3.6)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package(PythonInterp 3.6 REQUIRED)
find_package(PythonLibs 3.6 REQUIRED)

include_directories(
    ${PYTHON_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/pybind11/include
    SYSTEM
    ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}
    ${GLFW3_INCLUDE_DIR}
    ${GLEW_INCLUDE_DIRS}
    ${GLM_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    )

link_directories(
    /usr/local/cuda/lib64
    )

set(CMAKE_CXX_STANDARD 14)
set(OpenGL_GL_PREFERENCE "GLVND")
set( CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS}" "--shared --compiler-options '-fPIC'" )

string(APPEND CMAKE_CUDA_FLAGS " --expt-extended-lambda")
set(CUDA_PROPAGATE_HOST_FLAGS ON)

add_library(dogm_py SHARED
        dogm_py.cpp
    ${HEADER_FILES}
    ${SRC_FILES}
    )

target_link_libraries(dogm_py
    ${PYTHON_LIBRARIES}
    ${OPENGL_LIBRARY}
    ${GLFW3_LIBRARY}
    ${GLEW_LIBRARIES}
    cudart
)

set_target_properties(dogm_py PROPERTIES PREFIX "")
