# DOGM_PY

A pybind11 based wrapper for the [Dynamic-Obstacle-Grid (DOGM) library](https://github.com/idlebear/dynamic-occupancy-grid-map), based on a template forked from [here](https://github.com/PWhiddy/pybind11-cuda).  The DOGM library accepts 2D LaserScan formated input (a list of ranges) and outputs either a rendered probabilistic occupancy or free/occupied as a function of Demster-Shafer masses.  Here's a brief demo of the client processing data from the [Carla](www.carla.org) simulator showing free space in green, occupied in red, and orange/yellow a combination of the two.

<p align="center">
  <img src="./doc/occupancy-demo.gif">
</p>

## Prerequisites

* Python 3.6 or greater 
* Cmake 3.6 or greater 
* nVidia CUDA 

## To build 

Clone this repository and the required submodules using
```bash
-> git clone --recurse_submodules https://github.com/idlebear/dogm_py.git
```  
then building follows the standard recipe:
```bash
-> cd build
-> cmake ..
-> make -j8
```

## To install and run

Run 
```bash
-> python setup.py -e dogm_py
```
Alternatively, just copy `dogm_py.so` to a directory on your $PYTHONPATH.  Then from python
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
