# DOGM_PY

A pybind11 based wrapper for the [Dynamic-Obstacle-Grid library](https://github.com/idlebear/dynamic-occupancy-grid-map), based on a template forked from [here](https://github.com/PWhiddy/pybind11-cuda)

## Prerequisites

This module is intended to be installed as a submodule to the DOGM build, becoming a CMake build target.  It does not build on its own, though it may be useful as a reference.

Python 3.6 or greater 

Cmake 3.6 or greater 

## To build 

Download and build DOGM and this submodule.  

## To run

Copy `dogm_py.so` to a directory on your $PYTHONPATH.  Then from python
```
$ python
Python 3.8.10 (default, Sep 28 2021, 16:10:42) 
[GCC 9.3.0] on linux
Type "help", "copyright", "credits" or "license" for more information.
>>> import dogm_py as dogm
>>> help( dogm )
Help on module dogm_py:

NAME
    dogm_py - Python bindings for the Dynamic Occupancy Grid Map Library

CLASSES
    pybind11_builtins.pybind11_object(builtins.object)
        DOGM
        DOGMParams
        GridCell
        LaserMeasurementGrid
        LaserMeasurementGridParams
        MeasurementCellPtr
        VectorFloat
        VectorGridCell
    
    class DOGM(pybind11_builtins.pybind11_object)
     |  Method resolution order:
     |      DOGM
     |      pybind11_builtins.pybind11_object
     |      builtins.object
     |  
```
